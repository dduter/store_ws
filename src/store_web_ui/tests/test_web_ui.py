import asyncio
import importlib
import json
import sys
from pathlib import Path
from unittest.mock import Mock

import cv2
import httpx
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from store_web_ui.core import state_store as core
from store_web_ui.server import create_app


class ApiClient:
    def __init__(self, app):
        self.app = app

    def request(self, method, path, **kwargs):
        async def send():
            transport = httpx.ASGITransport(app=self.app)
            async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
                return await client.request(method, path, **kwargs)

        # 샌드박스에서 asyncio.run의 기본 실행기 종료 대기가 멈출 수 있습니다.
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(asyncio.wait_for(send(), 2))
        finally:
            loop.close()

    def get(self, path):
        return self.request("GET", path)

    def post(self, path, **kwargs):
        return self.request("POST", path, **kwargs)


@pytest.fixture
def dashboard():
    state = core.StateStore()
    frames = core.FrameBuffer()
    bridge = Mock()
    bridge.verify_remote_key.return_value = (True, "승인되었습니다.")
    app = create_app(state, bridge, frames)
    return ApiClient(app), state, bridge, frames, app


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)


def test_state_merge_and_isolation():
    state = core.StateStore()
    state.update_snapshot('{"mode":"EMPLOYEE","inventory":[{"track_id":"a"}],"kg_graph":{"nodes":[{"id":"a"}]}}')
    original = state.snapshot()
    for payload in ("{", "[]", '{"inventory":null}', '{"mode":4}', '{"inventory":[1e999]}'):
        state.update_snapshot(payload)
    assert state.snapshot() == original
    state.update_snapshot('{"kg_graph":{"stats":{"catalog_count":1}}}')
    view = state.snapshot()
    assert view["inventory"] == [{"track_id": "a"}]
    assert view["kg_graph"] == {"nodes": [{"id": "a"}], "links": [], "stats": {"catalog_count": 1}}
    view["inventory"].clear()
    assert state.snapshot()["inventory"]
    state.set_mode("CUSTOMER")
    state.update_snapshot('{"mode":"EMPLOYEE"}')
    state.set_remote_override(True)
    assert state.snapshot()["mode"] == "REMOTE"
    state.set_remote_override(False)
    assert state.snapshot()["mode"] == "CUSTOMER"
    assert state.version > original["version"]


def test_stale_health(monkeypatch):
    state = core.StateStore(stale_timeout_sec=6)
    monkeypatch.setattr(core, "monotonic", lambda: 100.0)
    state.update_node_health("test", True, "OK", "정상", {"sec": 9, "nanosec": 0})
    assert state.snapshot()["node_health"]["test"]["stale"] is False
    version = state.version
    monkeypatch.setattr(core, "monotonic", lambda: 106.25)
    health = state.snapshot()["node_health"]["test"]
    assert health["stale"] is True
    assert health["age_sec"] == 6.25
    assert health["stamp"]["sec"] == 9
    assert state.version == version


@pytest.mark.parametrize(("encoding", "pixel", "expected"), [
    ("rgb8", [220, 80, 20], [20, 80, 220]),
    ("bgr8", [20, 80, 220], [20, 80, 220]),
    ("rgba8", [220, 80, 20, 255], [20, 80, 220]),
    ("bgra8", [20, 80, 220, 255], [20, 80, 220]),
    ("mono8", [90], [90, 90, 90]),
])
def test_image_color_stride_and_resize(encoding, pixel, expected):
    row = bytes(pixel * 16 + [255] * 5)
    jpeg = core.ros_image_to_jpeg(16, 8, encoding, row * 8, max_width=8, step=len(row))
    image = decode(jpeg)
    assert image.shape == (4, 8, 3)
    np.testing.assert_allclose(image[2, 3], expected, atol=3)


@pytest.mark.parametrize(("width", "height", "encoding", "size", "step"), [
    (0, 2, "bgr8", 0, 0), (2, -1, "bgr8", 0, 6),
    (2, 2, "unknown", 12, 6), (2, 2, "bgr8", 10, 5),
    (2, 2, "bgr8", 12, 7), (2, 2, "bgr8", 13, None),
])
def test_invalid_image(width, height, encoding, size, step):
    assert core.ros_image_to_jpeg(width, height, encoding, bytes(size), step=step) is None


def test_frame_status(monkeypatch):
    buffer = core.FrameBuffer()
    monkeypatch.setattr(core, "monotonic", lambda: 20)
    buffer.update("webcam", b"one")
    monkeypatch.setattr(core, "monotonic", lambda: 20.125)
    buffer.update("webcam", b"two")
    assert buffer.latest("webcam").jpeg == b"two"
    assert buffer.status()["webcam"] == {"counter": 2, "live": True, "age_sec": 0, "fps": 8}
    monkeypatch.setattr(core, "monotonic", lambda: 24)
    assert buffer.status()["webcam"]["live"] is False


def test_index_static_and_state(dashboard):
    client, state, *_ = dashboard
    html = client.get("/").text
    assert "24시 무인편의점" in html
    assert 'data-vector="0,1,0"' in html and 'data-vector="0,-1,0"' in html
    assert "http://" not in html and "https://" not in html
    for name in ("app.js", "style.css"):
        assert client.get(f"/static/{name}").status_code == 200
    state.set_mode("EMPLOYEE")
    body = client.get("/api/state").json()
    assert body["mode"] == "EMPLOYEE"
    assert set(body["cameras"]) == {"realsense", "webcam"}


def test_text_and_estop(dashboard):
    client, state, bridge, *_ = dashboard
    assert client.post("/api/text_cmd", json={"text": "  \n "}).status_code == 400
    assert client.post("/api/text_cmd", json={"text": "  빼빼로 하나 줘  "}).json() == {
        "ok": True, "text": "빼빼로 하나 줘",
    }
    bridge.publish_text_cmd.assert_called_once_with("빼빼로 하나 줘")
    assert client.post("/api/emergency_stop", json={"stop": 1}).status_code == 422
    for stop in (True, False):
        assert client.post("/api/emergency_stop", json={"stop": stop}).json() == {
            "ok": True, "emergency_stop": stop,
        }
        bridge.publish_emergency_stop.assert_called_with(stop)
        assert state.snapshot()["emergency_stop"] is stop
    assert [event["action"] for event in state.snapshot()["ui_events"]] == [
        "text_cmd", "emergency_stop", "emergency_stop",
    ]


def test_remote_verify(dashboard):
    client, state, bridge, *_ = dashboard
    body = {"admin_key": "test-secret", "enable_remote": True}
    resp = client.post("/api/remote/verify", json=body).json()
    assert resp["granted"] is True and resp["reason"] == "승인되었습니다."
    assert isinstance(resp["remote_token"], str) and len(resp["remote_token"]) >= 16
    bridge.verify_remote_key.assert_called_once_with("test-secret", True)
    assert "test-secret" not in str(state.snapshot())
    assert state.snapshot()["remote_override"] is False
    bridge.verify_remote_key.return_value = (False, "거절")
    assert client.post("/api/remote/verify", json=body).json() == {
        "granted": False, "reason": "거절", "remote_token": None,
    }
    assert client.post("/api/remote/verify", json={**body, "enable_remote": "true"}).status_code == 422


def test_hand_cmd(dashboard):
    client, state, bridge, *_ = dashboard
    url = "/api/remote/hand_cmd"
    command = {"dx_px": 20, "dy_px": 0, "dz_px": 0, "stop": False}
    assert client.post(url, json=command).status_code == 409
    state.set_remote_override(True)
    # 관리자 웹 인증 토큰 없이는 원격 조작을 차단합니다.
    assert client.post(url, json=command).status_code == 403
    token = client.post("/api/remote/verify", json={"admin_key": "k", "enable_remote": True}).json()["remote_token"]
    headers = {"X-Remote-Token": token}
    assert client.post(url, json=command, headers={"X-Remote-Token": "wrong"}).status_code == 403
    for invalid in ({"dx_px": "20"}, {"dy_px": True}, {"gripper_closed": 0}, {"stop": "false"}):
        assert client.post(url, json={**command, **invalid}, headers=headers).status_code == 422
    for value in ("NaN", "Infinity", "1e999"):
        response = client.post(url, content=f'{{"dx_px":{value},"dy_px":0,"dz_px":0}}', headers=headers)
        assert response.status_code in (400, 422)
    bridge.publish_hand_cmd.assert_not_called()
    assert client.post(url, json={**command, "gripper_closed": True}, headers=headers).json() == {"ok": True}
    bridge.publish_hand_cmd.assert_called_once_with(20.0, 0.0, 0.0, gripper_closed=True, stop=False)
    state.set_emergency_stop(True)
    assert client.post(url, json=command, headers=headers).status_code == 409
    assert client.post(url, json={**command, "dx_px": 0, "stop": True}, headers=headers).status_code == 200
    bridge.publish_hand_cmd.assert_called_with(0.0, 0.0, 0.0, gripper_closed=None, stop=True)


def test_websocket_initial_changed_and_periodic(dashboard):
    _, state, _, _, app = dashboard
    state.update_node_health("test", True, "OK", "", {})

    async def exercise():
        incoming, outgoing = asyncio.Queue(), asyncio.Queue()
        await incoming.put({"type": "websocket.connect"})
        scope = {
            "type": "websocket", "path": "/ws/state", "root_path": "",
            "scheme": "ws", "headers": [], "query_string": b"", "subprotocols": [],
        }
        task = asyncio.create_task(app(scope, incoming.get, outgoing.put))
        try:
            assert (await asyncio.wait_for(outgoing.get(), 2))["type"] == "websocket.accept"

            async def next_snapshot():
                message = await asyncio.wait_for(outgoing.get(), 2)
                return json.loads(message["text"])

            initial = await next_snapshot()
            assert initial["mode"] == "CUSTOMER"
            state.set_emergency_stop(True)
            updated = await next_snapshot()
            assert updated["version"] > initial["version"]
            assert updated["emergency_stop"] is True
            periodic = await next_snapshot()
            assert periodic["version"] == updated["version"]
            assert periodic["node_health"]["test"]["age_sec"] > updated["node_health"]["test"]["age_sec"]
        finally:
            await incoming.put({"type": "websocket.disconnect", "code": 1000})
            await asyncio.wait_for(task, 2)

    asyncio.run(exercise())


@pytest.mark.parametrize("frame_status", ["idle", "live", "stale"])
def test_mjpeg_stream(dashboard, monkeypatch, frame_status):
    client, _, _, frames, app = dashboard
    assert client.get("/stream/unknown").status_code == 404
    live = core.make_placeholder_jpeg("LIVE", "", width=64, height=36)
    if frame_status != "idle":
        frames.update("realsense", live)
    if frame_status == "stale":
        server = importlib.import_module("store_web_ui.server")
        monkeypatch.setattr(server, "monotonic", lambda: frames.latest("realsense").timestamp + 4)

    async def first_part():
        # 무한 응답을 버퍼링하지 않고 첫 프레임 전송 직후 ASGI 연결을 닫습니다.
        messages = []
        sent_frame = asyncio.Event()

        async def receive():
            await sent_frame.wait()
            return {"type": "http.disconnect"}

        async def send(message):
            messages.append(message)
            if message.get("body"):
                sent_frame.set()

        scope = {
            "type": "http", "asgi": {"version": "3.0", "spec_version": "2.3"},
            "http_version": "1.1", "method": "GET", "path": "/stream/realsense",
            "root_path": "", "query_string": b"", "headers": [],
        }
        await asyncio.wait_for(app(scope, receive, send), timeout=2)
        return messages

    start, part = asyncio.run(first_part())
    assert start["status"] == 200
    assert b"multipart/x-mixed-replace; boundary=frame" == dict(start["headers"])[b"content-type"]
    headers, data = part["body"].split(b"\r\n\r\n", 1)
    assert headers.startswith(b"--frame\r\nContent-Type: image/jpeg")
    assert data.endswith(b"\r\n")
    assert decode(data[:-2]).shape == ((36, 64, 3) if frame_status == "live" else (360, 640, 3))
