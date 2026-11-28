import argparse
import json
import os
import select
import sys
import termios
import tty
from math import dist, isfinite
from time import monotonic, sleep


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def count_extended_fingers(landmarks, frame_width, frame_height) -> int:
    """검지~소지 4개 손가락 중 펴진 개수를 반환합니다."""
    points = [(x * frame_width, y * frame_height) for x, y in landmarks]
    wrist = points[0]
    return sum(
        dist(points[tip], wrist) > 1.15 * dist(points[tip - 2], wrist)
        for tip in (8, 12, 16, 20)
    )


def _edge_pan_axis(pos_norm: float, margin: float, deadzone_px: float, max_cmd_px: float) -> float:
    """화면 가장자리 [0..margin] 또는 [1-margin..1] 구간에서만 0~max_cmd_px를 생성합니다."""
    span = max(margin, 1e-6)
    if pos_norm < margin:
        ratio = min(1.0, (margin - pos_norm) / span)
        return -(deadzone_px + ratio * (max_cmd_px - deadzone_px))
    if pos_norm > 1.0 - margin:
        ratio = min(1.0, (pos_norm - (1.0 - margin)) / span)
        return deadzone_px + ratio * (max_cmd_px - deadzone_px)
    return 0.0


def landmarks_to_edge_pan_command(
    landmarks,
    frame_width,
    frame_height,
    *,
    prev_gripper_closed: bool = False,
    edge_margin: float = 0.25,
    z_margin: float = 0.30,
    deadzone_px: float = 15.0,
    max_cmd_px: float = 60.0,
) -> tuple[dict, dict]:
    """LoL 가장자리 패닝(XY) + 검지/브이(1~2손가락) 상하 Z 모드 명령을 생성합니다."""
    palm = [landmarks[index] for index in (0, 5, 9, 13, 17)]
    center_x, center_y = (sum(axis) / len(palm) for axis in zip(*palm))
    extended = count_extended_fingers(landmarks, frame_width, frame_height)

    if extended in (1, 2):
        # ☝️ 검지(1) 또는 ✌️ 브이(2): XY 고정, 그리퍼 상태 유지, 손 위/아래로 Z 상승/하강
        gripper_closed = prev_gripper_closed
        dx_px, dy_px = 0.0, 0.0
        # 화면 위쪽(y < z_margin)은 +Z(상승), 아래쪽(y > 1 - z_margin)은 -Z(하강)
        dz_px = -_edge_pan_axis(center_y, z_margin, deadzone_px, max_cmd_px)
        mode = "Z"
    else:
        # ✊ 주먹(0) = 닫힘+XY이동, 🖐️ 보(3~4) = 열림+XY이동
        gripper_closed = extended == 0
        dx_px = _edge_pan_axis(center_x, edge_margin, deadzone_px, max_cmd_px)
        dy_px = -_edge_pan_axis(center_y, edge_margin, deadzone_px, max_cmd_px)
        dz_px = 0.0
        mode = "XY"

    command = {
        "dx_px": dx_px,
        "dy_px": dy_px,
        "dz_px": dz_px,
        "gripper_closed": gripper_closed,
    }
    hud = {
        "mode": mode,
        "extended": extended,
        "center_norm": (center_x, center_y),
        "edge_margin": edge_margin,
        "z_margin": z_margin,
    }
    return command, hud


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
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cv2, mediapipe.solutions.hands, camera


def _draw_hand_preview(cv2, frame, command, landmarks=None, hud=None):
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    cx, cy = center
    white = (255, 255, 255)
    green = (0, 255, 0)
    cyan = (255, 220, 0)
    yellow = (0, 220, 255)

    mode = (hud or {}).get("mode", "XY")
    edge_margin = (hud or {}).get("edge_margin", 0.25)
    z_margin = (hud or {}).get("z_margin", 0.30)

    # 모드별 조작 영역 가이드 (XY: 중앙 정지 사각형 / Z: 상단 UP · 중앙 STOP · 하단 DOWN)
    if mode == "Z":
        y_top = round(height * z_margin)
        y_bot = round(height * (1.0 - z_margin))
        cv2.line(frame, (0, y_top), (width, y_top), yellow, 2)
        cv2.line(frame, (0, y_bot), (width, y_bot), yellow, 2)
        cv2.putText(frame, "UP (+Z)", (cx - 40, max(24, y_top - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, yellow, 2)
        cv2.putText(frame, "Z STOP", (cx - 38, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, white, 1)
        cv2.putText(frame, "DOWN (-Z)", (cx - 55, min(height - 10, y_bot + 26)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, yellow, 2)
    else:
        x1, y1 = round(width * edge_margin), round(height * edge_margin)
        x2, y2 = round(width * (1.0 - edge_margin)), round(height * (1.0 - edge_margin))
        if hasattr(cv2, "rectangle"):
            cv2.rectangle(frame, (x1, y1), (x2, y2), white, 2)
        cv2.line(frame, (cx - 8, cy), (cx + 8, cy), white, 1)
        cv2.line(frame, (cx, cy - 8), (cx, cy + 8), white, 1)
        cv2.circle(frame, center, 15, white, 1)

    # 21개 손 관절 + 손가락 뼈대 스켈레톤 그리기
    if landmarks and len(landmarks) >= 21:
        pts = [(round(x * width), round(y * height)) for x, y in landmarks]
        bone_color = yellow if mode == "Z" else cyan
        for start, end in HAND_CONNECTIONS:
            cv2.line(frame, pts[start], pts[end], bone_color, 2)
        for idx, pt in enumerate(pts):
            radius = 5 if idx in (4, 8, 12, 16, 20) else 3
            cv2.circle(frame, pt, radius, green if idx in (8, 12) else white, -1)

    if command is None:
        status = "NO HAND  |  5:XY+OPEN  0:XY+CLOSE  1-2:Z UP/DOWN"
    else:
        dx, dy, dz = command["dx_px"], command["dy_px"], command.get("dz_px", 0.0)
        if hud and "center_norm" in hud:
            px, py = hud["center_norm"]
            palm = (round(px * width), round(py * height))
        else:
            palm = (round(width / 2 + dx), round(height / 2 - dy))
        arrow_color = yellow if mode == "Z" else green
        cv2.arrowedLine(frame, center, palm, arrow_color, 2)
        cv2.circle(frame, palm, 6, arrow_color, -1)
        gripper = "CLOSED" if command["gripper_closed"] else "OPEN"
        fingers = (hud or {}).get("extended", "-")
        status = f"[{mode}] F:{fingers} dx:{dx:+.0f} dy:{dy:+.0f} dz:{dz:+.0f} GRIP:{gripper}"
    cv2.putText(frame, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, white, 2)
    cv2.putText(frame, "5:XY+OPEN | 0:XY+CLOSE | 1-2:Z UP/DOWN | Q/ESC: quit", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.48, white, 1)


def _run_webcam(webcam, publish, running, rate_hz, no_preview=False):
    cv2, hands, camera = webcam
    preview = not no_preview and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    preview_used = False
    prev_gripper_closed = False
    try:
        with hands.Hands(max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5) as detector:
            while running():
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("카메라 프레임을 읽을 수 없습니다.")
                frame = cv2.flip(frame, 1)
                result = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                command, landmarks, hud = None, None, None
                if result.multi_hand_landmarks:
                    height, width = frame.shape[:2]
                    landmarks = [(p.x, p.y) for p in result.multi_hand_landmarks[0].landmark]
                    command, hud = landmarks_to_edge_pan_command(
                        landmarks, width, height, prev_gripper_closed=prev_gripper_closed,
                    )
                    prev_gripper_closed = command["gripper_closed"]
                if preview:
                    _draw_hand_preview(cv2, frame, command, landmarks=landmarks, hud=hud)
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
