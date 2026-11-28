import json
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
from threading import RLock
from time import monotonic

import cv2
import numpy as np


def parse_vest_payload(raw: str) -> tuple[bool, float, str]:
    value = raw.strip().lower()
    if value in ("true", "1"):
        return True, 1.0, "vision"
    if value in ("false", "0"):
        return False, 0.0, "vision"
    try:
        payload = json.loads(raw)
    except (ValueError, RecursionError):
        return False, 0.0, "vision"
    if not isinstance(payload, dict) or type(payload.get("detected")) is not bool:
        return False, 0.0, "vision"
    confidence = payload.get("confidence", 0.0)
    if type(confidence) not in (int, float) or not 0 <= confidence <= 1:
        confidence = 0.0
    source = payload.get("source")
    return payload["detected"], float(confidence), source if isinstance(source, str) and source else "vision"


class StateStore:
    def __init__(self, stale_timeout_sec=6.0):
        self._lock = RLock()
        self._version = 0
        self._stale_timeout = stale_timeout_sec
        self._mode = "CUSTOMER"
        self._remote_override = False
        self._emergency_stop = False
        self._mode_topic_seen = False
        self._ui_events = deque(maxlen=50)
        self._health = {}
        self._vest_status = {"detected": False, "confidence": 0.0, "source": ""}
        self._vest_received = None
        self._remote_request = {"active": False, "item": "", "message": "", "requested_at": ""}
        self._payload = {
            "inventory": [], "disposal_history": [], "agent_log": [],
            "kg_graph": {"nodes": [], "links": [], "stats": {}},
        }

    @property
    def version(self):
        with self._lock:
            return self._version

    @property
    def remote_override(self):
        with self._lock:
            return self._remote_override

    @property
    def emergency_stop(self):
        with self._lock:
            return self._emergency_stop

    def update_snapshot(self, payload):
        try:
            # JSON에 허용되지 않는 NaN, Infinity, 범위 초과 실수를 걸러냅니다.
            parsed = json.loads(payload)
            json.dumps(parsed, allow_nan=False, ensure_ascii=False).encode("utf-8")
        except (ValueError, TypeError, RecursionError):
            return
        if not isinstance(parsed, dict):
            return
        with self._lock:
            changed = False
            for key in ("inventory", "disposal_history", "agent_log"):
                value = parsed.get(key)
                if isinstance(value, list) and value != self._payload[key]:
                    self._payload[key] = value
                    changed = True
            graph = parsed.get("kg_graph")
            if isinstance(graph, dict):
                current = self._payload["kg_graph"]
                for key, kind in (("nodes", list), ("links", list), ("stats", dict)):
                    value = graph.get(key)
                    if isinstance(value, kind) and value != current[key]:
                        current[key] = value
                        changed = True
            mode = parsed.get("mode")
            if not self._mode_topic_seen and isinstance(mode, str) and mode.strip():
                changed |= self._set("_mode", mode.strip())
            self._version += int(changed)

    def _set(self, field, value):
        if getattr(self, field) == value:
            return False
        setattr(self, field, value)
        return True

    def set_mode(self, mode):
        if mode.strip():
            with self._lock:
                self._mode_topic_seen = True
                self._version += int(self._set("_mode", mode.strip()))

    def set_remote_override(self, enabled):
        with self._lock:
            self._version += int(self._set("_remote_override", enabled))

    def set_emergency_stop(self, stop):
        with self._lock:
            self._version += int(self._set("_emergency_stop", stop))

    def record_event(self, action, message):
        with self._lock:
            self._ui_events.append({
                "action": action, "message": message,
                "time": datetime.now(timezone.utc).isoformat(),
            })
            self._version += 1

    def update_node_health(self, node_name, alive, state, message, stamp):
        with self._lock:
            self._health[node_name] = (monotonic(), {
                "node_name": node_name, "alive": alive, "state": state,
                "message": message, "stamp": deepcopy(stamp),
            })
            self._version += 1

    def update_vest_detection(self, detected: bool, confidence: float, source: str = "vision"):
        with self._lock:
            self._vest_status = {"detected": detected, "confidence": confidence, "source": source}
            self._vest_received = monotonic()
            self._version += 1

    def set_remote_request(self, active: bool, item: str = "", message: str = ""):
        with self._lock:
            self._remote_request = {
                "active": active,
                "item": item if active else "",
                "message": message if active else "",
                "requested_at": datetime.now(timezone.utc).isoformat() if active else "",
            }
            self._version += 1

    def snapshot(self):
        with self._lock:
            now = monotonic()
            health = {}
            for name, (received, status) in self._health.items():
                age = now - received
                health[name] = {
                    **deepcopy(status), "age_sec": age, "stale": age > self._stale_timeout,
                }
            vest_age = now - self._vest_received if self._vest_received is not None else None
            return {
                "vest_status": {
                    **self._vest_status, "age_sec": vest_age,
                    "live": vest_age is not None and vest_age <= self._stale_timeout,
                },
                "remote_request": dict(self._remote_request),
                **deepcopy(self._payload), "version": self._version,
                "mode": "REMOTE" if self._remote_override else self._mode,
                "remote_override": self._remote_override,
                "emergency_stop": self._emergency_stop,
                "ui_events": deepcopy(list(self._ui_events)), "node_health": health,
            }


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    timestamp: float
    counter: int
    fps: float


class FrameBuffer:
    CAMERAS = ("realsense", "webcam")
    STALE_SEC = 3.0

    def __init__(self):
        self._lock = RLock()
        self._frames = {}

    def update(self, camera_name, jpeg):
        with self._lock:
            now = monotonic()
            previous = self._frames.get(camera_name)
            counter, fps = 1, 0.0
            if previous:
                counter = previous.counter + 1
                elapsed = now - previous.timestamp
                if 0 < elapsed <= self.STALE_SEC:
                    fps = 1 / elapsed
                    if previous.fps:
                        fps = 0.8 * previous.fps + 0.2 * fps
            self._frames[camera_name] = Frame(bytes(jpeg), now, counter, fps)

    def latest(self, camera_name):
        with self._lock:
            return self._frames.get(camera_name)

    def status(self):
        with self._lock:
            now = monotonic()
            result = {}
            for name in self.CAMERAS:
                frame = self._frames.get(name)
                age = now - frame.timestamp if frame else None
                live = age is not None and age <= self.STALE_SEC
                result[name] = {
                    "live": live, "fps": round(frame.fps, 1) if live else 0.0,
                    "age_sec": age, "counter": frame.counter if frame else 0,
                }
            return result


_ENCODINGS = {
    "rgb8": (3, cv2.COLOR_RGB2BGR), "bgr8": (3, None),
    "rgba8": (4, cv2.COLOR_RGBA2BGR), "bgra8": (4, cv2.COLOR_BGRA2BGR),
    "mono8": (1, None),
}


def ros_image_to_jpeg(width, height, encoding, data, max_width=640, quality=75, *, step=None):
    if min(width, height, max_width) <= 0 or encoding not in _ENCODINGS:
        return None
    channels, conversion = _ENCODINGS[encoding]
    row_bytes = width * channels
    try:
        pixels = np.frombuffer(data, dtype=np.uint8)
        if step is None:
            step, remainder = divmod(pixels.size, height)
            if remainder:
                return None
        if step < row_bytes or step * height != pixels.size:
            return None
        image = pixels.reshape(height, step)[:, :row_bytes]
        if channels > 1:
            image = image.reshape(height, width, channels)
        if conversion is not None:
            image = cv2.cvtColor(image, conversion)
        if width > max_width:
            image = cv2.resize(
                image, (max_width, max(1, round(height * max_width / width))),
                interpolation=cv2.INTER_AREA,
            )
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return encoded.tobytes() if ok else None
    except (cv2.error, TypeError, ValueError, OverflowError, BufferError):
        return None


def make_placeholder_jpeg(title, subtitle, width=640, height=360):
    image = np.full((height, width, 3), (29, 23, 16), dtype=np.uint8)
    for x in range(0, width, 32):
        cv2.line(image, (x, 0), (x, height), (42, 34, 25), 1)
    for y in range(0, height, 32):
        cv2.line(image, (0, y), (width, y), (42, 34, 25), 1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.circle(image, (width // 2, int(height * 0.34)), max(2, height // 45), (212, 174, 73), 2)
    for value, scale, baseline, color in (
        (title, 0.85, 0.52, (239, 229, 210)), (subtitle, 0.55, 0.64, (178, 158, 133)),
    ):
        label = value.encode("ascii", "ignore").decode()[:100]
        text_width = cv2.getTextSize(label, font, scale, 1)[0][0]
        scale *= min(1, width * 0.9 / max(1, text_width))
        text_width = cv2.getTextSize(label, font, scale, 1)[0][0]
        cv2.putText(image, label, ((width - text_width) // 2, int(height * baseline)),
                    font, scale, color, 1, cv2.LINE_AA)
    return cv2.imencode(".jpg", image)[1].tobytes()
