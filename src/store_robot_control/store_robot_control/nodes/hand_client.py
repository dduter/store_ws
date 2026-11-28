import argparse
import json
import os
import select
import sys
import termios
import tty
from math import dist, isfinite
from time import monotonic, sleep


def landmarks_to_hand_command(landmarks, frame_width, frame_height) -> dict:
    palm = [landmarks[index] for index in (0, 5, 9, 13, 17)]
    center_x, center_y = (sum(axis) / len(palm) for axis in zip(*palm))
    # 화면의 종횡비를 반영한 픽셀 거리로 손가락이 접혔는지 판정합니다.
    points = [(x * frame_width, y * frame_height) for x, y in landmarks]
    wrist = points[0]
    folded = sum(
        dist(points[tip], wrist) <= 1.1 * dist(points[tip - 2], wrist)
        for tip in (8, 12, 16, 20)
    )
    return {
        "dx_px": (center_x - 0.5) * frame_width,
        "dy_px": (0.5 - center_y) * frame_height,
        "gripper_closed": folded >= 3,
    }


def keyboard_key_to_command(key, step_px=40.0, gripper_closed=False) -> tuple[dict | None, bool, bool]:
    key = key.lower()
    if key == "q":
        return None, gripper_closed, True
    direction = {
        "w": (0, 1, 0), "s": (0, -1, 0), "a": (-1, 0, 0), "d": (1, 0, 0),
        "r": (0, 0, 1), "f": (0, 0, -1), "g": (0, 0, 0),
        " ": (0, 0, 0), "x": (0, 0, 0),
    }.get(key)
    if direction is None:
        return None, gripper_closed, False
    if key == "g":
        gripper_closed = not gripper_closed
    dx, dy, dz = (axis * step_px for axis in direction)
    return {"dx_px": dx, "dy_px": dy, "dz_px": dz, "gripper_closed": gripper_closed}, gripper_closed, False


def serialize_hand_command(
    dx_px, dy_px, dz_px=0.0, gripper_closed=False, stop=False,
) -> str:
    payload = {
        "dx_px": dx_px, "dy_px": dy_px, "dz_px": dz_px, "gripper_closed": gripper_closed,
    }
    if stop:
        payload["stop"] = True
    return json.dumps(payload, allow_nan=False)


class KeyboardCommandHolder:
    def __init__(self, hold_sec=0.25):
        if not isfinite(hold_sec) or hold_sec < 0:
            raise ValueError("이동 유지 시간은 유한한 0 이상의 값이어야 합니다.")
        self.hold_sec = hold_sec
        self.gripper_closed = False
        self._movement = None
        self._expires_at = 0.0
        self._fresh_movement = False
        self._gripper_changed = False
        self._stop_requested = False

    def press(self, key, now, step_px=40.0):
        key = key.lower()
        command, self.gripper_closed, should_quit = keyboard_key_to_command(
            key, step_px, self.gripper_closed,
        )
        if key == "g":
            self._gripper_changed = True
        elif key in {" ", "x"}:
            self._movement = None
            self._fresh_movement = False
            self._stop_requested = True
        elif command is not None:
            self._movement = command
            self._expires_at = now + self.hold_sec
            self._fresh_movement = True
            self._stop_requested = False
        return should_quit

    def current_command(self, now):
        if self._stop_requested:
            self._stop_requested = False
            self._gripper_changed = False
            return {
                "dx_px": 0.0, "dy_px": 0.0, "dz_px": 0.0,
                "gripper_closed": self.gripper_closed, "stop": True,
            }
        # 그리퍼 전환 프레임에는 중립 이동을 보내고 기존 이동 만료 시각은 유지합니다.
        gripper_changed = self._gripper_changed
        self._gripper_changed = False
        fresh = self._fresh_movement
        self._fresh_movement = False
        if not gripper_changed and self._movement is not None and (fresh or now < self._expires_at):
            return {**self._movement, "gripper_closed": self.gripper_closed}
        return {"dx_px": 0.0, "dy_px": 0.0, "dz_px": 0.0, "gripper_closed": self.gripper_closed}


def _apply_key_batch(holder, keys, now, step_px):
    should_quit = False
    stop_pressed = False
    gripper_toggles = 0
    last_move = ""
    for raw in keys:
        key = raw.lower()
        if key == "q":
            should_quit = True
        elif key in {" ", "x"}:
            stop_pressed = True
        elif key == "g":
            gripper_toggles += 1
        elif key in {"w", "s", "a", "d", "r", "f"}:
            last_move = key
    if should_quit:
        return True
    if stop_pressed:
        # 같은 주기 안에 정지 키가 포함되면 뒤따른 이동 키보다 정지를 우선 적용합니다.
        holder.press(" ", now, step_px)
    elif last_move:
        holder.press(last_move, now, step_px)
    for _ in range(gripper_toggles):
        holder.press("g", now, step_px)
    return False


def _run_keyboard(publish, running, rate_hz, step_px, hold_sec=0.25):
    if not sys.stdin.isatty():
        raise RuntimeError("키보드 모드는 입력 가능한 터미널이 필요합니다.")
    fd = sys.stdin.fileno()
    attributes = termios.tcgetattr(fd)
    holder = KeyboardCommandHolder(hold_sec)
    print("키보드 조작 (W/S·A/D 이동, R/F 높이, G 그리퍼, Space/X 정지, Q 종료)", flush=True)
    try:
        tty.setcbreak(fd)
        while running():
            keys = []
            while select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 1)
                if not data:
                    return
                decoded = data.decode("ascii", errors="ignore")
                if decoded:
                    keys.append(decoded)
            now = monotonic()
            if keys and _apply_key_batch(holder, keys, now, step_px):
                break
            publish(holder.current_command(now))
            sleep(1.0 / rate_hz)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, attributes)


def _open_webcam(camera_index):
    try:
        import cv2
        import mediapipe
    except ImportError as exc:
        raise RuntimeError(
            "웹캠 모드에는 opencv-python과 mediapipe가 필요합니다. "
            "설치하거나 --mode keyboard를 사용하세요."
        ) from exc
    if not hasattr(mediapipe, "solutions"):
        raise RuntimeError(
            "현재 MediaPipe에는 Hands Solutions API가 없습니다. 해당 API를 제공하는 버전을 "
            "설치하거나 --mode keyboard를 사용하세요."
        )
    camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"카메라 {camera_index}를 열 수 없습니다.")
    return cv2, mediapipe.solutions.hands, camera


def _draw_hand_preview(cv2, frame, command):
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    cx, cy = center
    white = (255, 255, 255)
    cv2.line(frame, (cx - 8, cy), (cx + 8, cy), white, 1)
    cv2.line(frame, (cx, cy - 8), (cx, cy + 8), white, 1)
    cv2.circle(frame, center, 15, white, 1)
    if command is None:
        status = "dx: --  dy: --  GRIPPER: --"
    else:
        dx, dy = command["dx_px"], command["dy_px"]
        palm = (round(width / 2 + dx), round(height / 2 - dy))
        green = (0, 255, 0)
        cv2.arrowedLine(frame, center, palm, green, 2)
        cv2.circle(frame, palm, 4, green, -1)
        gripper = "CLOSED" if command["gripper_closed"] else "OPEN"
        status = f"dx: {dx:+.0f}  dy: {dy:+.0f}  GRIPPER: {gripper}"
    cv2.putText(frame, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, white, 2)
    cv2.putText(frame, "Q/ESC: quit", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, white, 2)


def _run_webcam(webcam, publish, running, rate_hz, no_preview=False):
    cv2, hands, camera = webcam
    preview = not no_preview and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    preview_used = False
    try:
        with hands.Hands(max_num_hands=1) as detector:
            while running():
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("카메라 프레임을 읽을 수 없습니다.")
                frame = cv2.flip(frame, 1)
                result = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                command = None
                if result.multi_hand_landmarks:
                    height, width = frame.shape[:2]
                    landmarks = [(p.x, p.y) for p in result.multi_hand_landmarks[0].landmark]
                    command = landmarks_to_hand_command(landmarks, width, height)
                if preview:
                    _draw_hand_preview(cv2, frame, command)
                    cv2.imshow("Store Remote Hand Control", frame)
                    preview_used = True
                    if cv2.waitKey(1) in (ord("q"), ord("Q"), 27):
                        break
                if command is not None:
                    publish(command)
                sleep(1.0 / rate_hz)
    finally:
        if preview_used:
            cv2.destroyAllWindows()


def main(args=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String

    argv = sys.argv if args is None else ["hand_client", *args]
    parser = argparse.ArgumentParser(description="웹캠 또는 키보드 원격 조작 입력")
    parser.add_argument("--mode", choices=("auto", "webcam", "keyboard"), default="auto")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=15.0)
    parser.add_argument("--step-px", type=float, default=40.0)
    parser.add_argument("--hold-sec", type=float, default=0.25)
    parser.add_argument("--no-preview", action="store_true")
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not all(isfinite(value) and value > 0 for value in (options.rate_hz, options.step_px)):
        parser.error("--rate-hz와 --step-px는 유한한 양수여야 합니다.")

    if not isfinite(options.hold_sec) or options.hold_sec < 0:
        parser.error("--hold-sec는 유한한 0 이상의 값이어야 합니다.")

    webcam = None
    if options.mode != "keyboard":
        try:
            webcam = _open_webcam(options.camera_index)
        except RuntimeError as exc:
            if options.mode == "webcam":
                print(exc, file=sys.stderr)
                return 1
            print(f"{exc} 키보드 모드로 전환합니다.", file=sys.stderr)

    node = None
    try:
        rclpy.init(args=argv)
        node = rclpy.create_node("hand_client")
        publisher = node.create_publisher(String, "/store/remote/hand_cmd", 1)

        def publish(command):
            publisher.publish(String(data=serialize_hand_command(**command)))

        if webcam is None:
            _run_keyboard(publish, rclpy.ok, options.rate_hz, options.step_px, options.hold_sec)
        else:
            _run_webcam(webcam, publish, rclpy.ok, options.rate_hz, no_preview=options.no_preview)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if rclpy.ok():
            print(exc, file=sys.stderr)
            return 1
    finally:
        if webcam is not None:
            webcam[2].release()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0
