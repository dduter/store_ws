import argparse
import json
import os
import select
import sys
import termios
import tty
from math import dist, isfinite
from time import sleep


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
    }.get(key)
    if direction is None:
        return None, gripper_closed, False
    if key == "g":
        gripper_closed = not gripper_closed
    dx, dy, dz = (axis * step_px for axis in direction)
    return {"dx_px": dx, "dy_px": dy, "dz_px": dz, "gripper_closed": gripper_closed}, gripper_closed, False


def serialize_hand_command(dx_px, dy_px, dz_px=0.0, gripper_closed=False) -> str:
    return json.dumps({
        "dx_px": dx_px, "dy_px": dy_px, "dz_px": dz_px, "gripper_closed": gripper_closed,
    }, allow_nan=False)


def _run_keyboard(publish, running, rate_hz, step_px):
    if not sys.stdin.isatty():
        raise RuntimeError("키보드 모드는 입력 가능한 터미널이 필요합니다.")
    fd = sys.stdin.fileno()
    attributes = termios.tcgetattr(fd)
    closed = False
    print("키보드 조작: W/S·A/D 이동, R/F 높이, G 그리퍼, Q 종료", flush=True)
    try:
        tty.setcbreak(fd)
        while running():
            key = os.read(fd, 1).decode("ascii", errors="ignore") if select.select([fd], [], [], 0)[0] else ""
            command, closed, should_quit = keyboard_key_to_command(key, step_px, closed)
            if should_quit:
                break
            # 입력이 없는 동안에도 중립 이동과 그리퍼 상태를 보내 디바운싱을 유지합니다.
            publish(command or {"dx_px": 0.0, "dy_px": 0.0, "gripper_closed": closed})
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


def _run_webcam(webcam, publish, running, rate_hz):
    cv2, hands, camera = webcam
    with hands.Hands(max_num_hands=1) as detector:
        while running():
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("카메라 프레임을 읽을 수 없습니다.")
            frame = cv2.flip(frame, 1)
            result = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if result.multi_hand_landmarks:
                height, width = frame.shape[:2]
                landmarks = [(p.x, p.y) for p in result.multi_hand_landmarks[0].landmark]
                publish(landmarks_to_hand_command(landmarks, width, height))
            sleep(1.0 / rate_hz)


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
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not all(isfinite(value) and value > 0 for value in (options.rate_hz, options.step_px)):
        parser.error("--rate-hz와 --step-px는 유한한 양수여야 합니다.")

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
            _run_keyboard(publish, rclpy.ok, options.rate_hz, options.step_px)
        else:
            _run_webcam(webcam, publish, rclpy.ok, options.rate_hz)
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
