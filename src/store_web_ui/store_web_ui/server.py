import asyncio
import math
import secrets
from pathlib import Path
from time import monotonic

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse

from .core.state_store import make_placeholder_jpeg


async def read_object(request):
    try:
        body = await request.json()
    except (ValueError, RecursionError):
        raise HTTPException(400, "올바른 JSON 요청이 필요합니다.") from None
    if not isinstance(body, dict):
        raise HTTPException(422, "JSON 객체가 필요합니다.")
    return body


def finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


async def stream_frames(frame_buffer, camera_name, placeholder):
    last_counter = 0
    while True:
        frame = frame_buffer.latest(camera_name)
        live = frame and monotonic() - frame.timestamp <= frame_buffer.STALE_SEC
        if not live or frame.counter != last_counter:
            jpeg = frame.jpeg if live else placeholder
            if live:
                last_counter = frame.counter
            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n"
            )
        await asyncio.sleep(1 / 15 if live else 0.5)


def create_app(state_store, bridge, frame_buffer, *, allow_vest_simulation: bool = True):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    static_dir = Path(__file__).resolve().parent / "static"
    static = {
        name: ((static_dir / name).read_bytes(), media_type)
        for name, media_type in (
            ("index.html", "text/html"), ("style.css", "text/css"), ("app.js", "text/javascript"),
        )
    }
    placeholders = {
        name: make_placeholder_jpeg(title, "Waiting for camera")
        for name, title in (("realsense", "RealSense Eye-in-Hand"), ("webcam", "Store Webcam"))
    }
    active_remote_token = {"value": None}
    vest_simulation_lock = asyncio.Lock()

    def snapshot():
        return {
            **state_store.snapshot(),
            "cameras": frame_buffer.status(),
            "allow_vest_simulation": allow_vest_simulation,
        }

    @app.get("/")
    async def index():
        return Response(static["index.html"][0], media_type="text/html")

    @app.get("/static/{path:path}")
    async def static_file(path: str):
        if path not in static:
            raise HTTPException(404, "정적 파일을 찾을 수 없습니다.")
        content, media_type = static[path]
        return Response(content, media_type=media_type)

    @app.get("/api/state")
    async def state():
        return snapshot()

    @app.websocket("/ws/state")
    async def state_socket(socket: WebSocket):
        await socket.accept()
        last_version, last_sent = -1, 0.0
        try:
            while True:
                if state_store.version != last_version or monotonic() - last_sent >= 1:
                    view = snapshot()
                    await socket.send_json(view)
                    last_version, last_sent = view["version"], monotonic()
                try:
                    message = await asyncio.wait_for(socket.receive(), timeout=0.1)
                except asyncio.TimeoutError:
                    continue
                if message["type"] == "websocket.disconnect":
                    break
        except (WebSocketDisconnect, OSError):
            pass

    @app.get("/stream/{camera_name}")
    async def stream(camera_name: str):
        if camera_name not in placeholders:
            raise HTTPException(404, "등록되지 않은 카메라입니다.")
        return StreamingResponse(
            stream_frames(frame_buffer, camera_name, placeholders[camera_name]),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @app.post("/api/text_cmd")
    async def text_cmd(request: Request):
        text = (await read_object(request)).get("text")
        if not isinstance(text, str):
            raise HTTPException(422, "명령은 문자열이어야 합니다.")
        text = text.strip()
        if not text:
            raise HTTPException(400, "명령을 입력하세요.")
        bridge.publish_text_cmd(text)
        state_store.record_event("text_cmd", text)
        return {"ok": True, "text": text}

    @app.post("/api/voice/ptt")
    async def voice_ptt():
        bridge.publish_ptt_trigger()
        state_store.record_event("voice_ptt", "음성 입력(PTT) 시작 요청")
        return {"ok": True}

    @app.post("/api/remote_request")
    async def remote_request(request: Request):
        body = await read_object(request)
        active = body.get("active")
        item = body.get("item", "모형 담배")
        message = body.get("message", "")
        send_text_cmd = body.get("send_text_cmd", False)
        if (type(active) is not bool or type(send_text_cmd) is not bool
                or not isinstance(item, str) or not isinstance(message, str)):
            raise HTTPException(422, "원격 요청의 활성 상태, 상품명, 메시지와 명령 전송 설정을 확인하세요.")
        cleaned_item = item.strip()
        cleaned_message = message.strip()
        state_store.set_remote_request(active, cleaned_item, cleaned_message)
        if active and send_text_cmd:
            requested_item = (body.get("item") or "").strip() if isinstance(body.get("item"), str) else ""
            bridge.publish_text_cmd(f"{requested_item or '담배'} 하나 주세요")
        state_store.record_event(
            "remote_request",
            (cleaned_message or f"{cleaned_item or '상품'} 구매 요청") if active else "원격 요청 처리 완료",
        )
        return {"ok": True, "remote_request": state_store.snapshot()["remote_request"]}

    @app.post("/api/vest_simulate")
    async def vest_simulate(request: Request):
        if not allow_vest_simulation:
            raise HTTPException(403, "형광조끼 자동 인식 전용 모드에서는 수동 조끼 시뮬레이션이 비활성화되어 있습니다.")
        detected = (await read_object(request)).get("detected")
        if type(detected) is not bool:
            raise HTTPException(422, "조끼 감지 상태는 불리언이어야 합니다.")
        async with vest_simulation_lock:
            sent = await asyncio.to_thread(bridge.publish_vest_simulation, detected)
            if sent is False:
                raise HTTPException(409, "실시간 형광조끼 감지가 유지 중이거나 종료 중이어서 시뮬레이션을 완료하지 못했습니다.")
            state_store.update_vest_detection(detected, 0.99 if detected else 0.0, source="web_ui_sim")
            state_store.record_event(
                "vest_sim",
                "직원 모드 전환 요청 (조끼 시뮬)" if detected else "고객 모드 복귀 요청 (조끼 해제 시뮬)",
            )
        return {"ok": True, "detected": detected}

    @app.post("/api/emergency_stop")
    async def emergency_stop(request: Request):
        stop = (await read_object(request)).get("stop")
        if type(stop) is not bool:
            raise HTTPException(422, "정지 상태는 불리언이어야 합니다.")
        state_store.set_emergency_stop(stop)
        bridge.publish_emergency_stop(stop)
        state_store.record_event("emergency_stop", "비상 정지" if stop else "비상 정지 해제")
        return {"ok": True, "emergency_stop": stop}

    @app.post("/api/remote/verify")
    async def verify(request: Request):
        body = await read_object(request)
        key, enabled = body.get("admin_key"), body.get("enable_remote")
        if not isinstance(key, str) or type(enabled) is not bool:
            raise HTTPException(422, "관리자 키와 원격 모드 설정을 확인하세요.")
        granted, reason = await asyncio.to_thread(bridge.verify_remote_key, key, enabled)
        token = None
        if granted:
            token = secrets.token_urlsafe(24) if enabled else None
            active_remote_token["value"] = token
            state_store.record_event("remote_override", "원격 모드 활성화" if enabled else "원격 모드 해제")
        return {"granted": granted, "reason": reason, "remote_token": token}

    @app.post("/api/remote/hand_cmd")
    async def hand_cmd(request: Request):
        body = await read_object(request)
        deltas = [body.get(axis) for axis in ("dx_px", "dy_px", "dz_px")]
        gripper, stop = body.get("gripper_closed"), body.get("stop", False)
        if (not all(map(finite_number, deltas)) or type(stop) is not bool
                or (gripper is not None and type(gripper) is not bool)):
            raise HTTPException(422, "이동량은 유한한 숫자여야 하며 정지와 그리퍼 상태는 불리언이어야 합니다.")
        if not state_store.remote_override:
            active_remote_token["value"] = None
            raise HTTPException(409, "원격 모드가 비활성 상태입니다.")
        expected = active_remote_token["value"]
        provided = request.headers.get("x-remote-token", "")
        if not expected or not secrets.compare_digest(expected, provided):
            raise HTTPException(403, "웹 원격 조작 권한이 없습니다. 관리자 키를 다시 인증하세요.")
        if state_store.emergency_stop and not stop:
            raise HTTPException(409, "비상 정지가 활성 상태입니다.")
        bridge.publish_hand_cmd(*map(float, deltas), gripper_closed=gripper, stop=stop)
        return {"ok": True}

    return app
