"""Mobile-friendly FastAPI web server for live & photo YOLOv8 testing from a phone."""

from __future__ import annotations

import argparse
import base64
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import uvicorn
from fastapi import FastAPI, File, Form, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from ultralytics import YOLO

MODEL_REGISTRY: dict[str, dict[str, str]] = {
    "store_5classes": {
        "label": "5종 통합 모델 (포스틱·빼빼로·음료·뒷면·조끼)",
        "path": "/home/ms-04/store_ws/runs/store_5classes_train/weights/best.pt",
    },
}

DEFAULT_MODEL_KEY = "store_5classes"
CERT_DIR = Path("/home/ms-04/store_ws/.certs")

app = FastAPI(title="StoreBot Mobile YOLO Live Tester")

_device = "cuda:0" if torch.cuda.is_available() else "cpu"
_models: dict[str, YOLO] = {}


def load_models() -> dict[str, YOLO]:
    """Load all available models listed in MODEL_REGISTRY."""
    if not _models:
        for key, info in MODEL_REGISTRY.items():
            path = Path(info["path"])
            if path.exists():
                _models[key] = YOLO(str(path))
    return _models


@app.on_event("startup")
def _warmup() -> None:
    models = load_models()
    dummy = np.zeros((640, 640, 3), dtype=np.uint8)
    for model in models.values():
        model.predict(dummy, device=_device, verbose=False)


def _run_inference(img: np.ndarray, model_key: str, conf: float) -> tuple[list[dict], float]:
    """Run YOLO inference on a BGR image and return detections + elapsed ms."""
    models = load_models()
    target_keys = list(models.keys()) if model_key == "all" else [model_key]
    target_keys = [k for k in target_keys if k in models]
    if not target_keys:
        raise ValueError(f"Unknown model: {model_key}")

    t0 = time.perf_counter()
    detections: list[dict] = []
    for key in target_keys:
        model = models[key]
        res = model.predict(img, conf=conf, device=_device, verbose=False)[0]
        if res.boxes is not None:
            for box in res.boxes:
                cls_id = int(box.cls[0].item())
                detections.append(
                    {
                        "model_key": key,
                        "class_id": cls_id,
                        "class_name": str(model.names.get(cls_id, cls_id)),
                        "conf": round(float(box.conf[0].item()), 4),
                        "bbox_xyxy": [round(float(v), 1) for v in box.xyxy[0].tolist()],
                    }
                )
    elapsed_ms = round((time.perf_counter() - t0) * 1000, 1)
    detections.sort(key=lambda x: x["conf"], reverse=True)
    return detections, elapsed_ms


HTML_PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <title>StoreBot YOLO 실시간 테스터</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0; padding: 14px;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #0f172a; color: #f8fafc;
    }
    .container { max-width: 640px; margin: 0 auto; }
    h1 { font-size: 1.25rem; margin: 0 0 6px; }
    .sub { font-size: 0.82rem; color: #94a3b8; margin-bottom: 12px; display: flex; gap: 6px; flex-wrap: wrap; }
    .card {
      background: #1e293b; border-radius: 14px; padding: 12px;
      margin-bottom: 12px; border: 1px solid #334155;
    }
    .model-tabs {
      display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; margin-bottom: 10px;
    }
    .tab-btn {
      padding: 9px 6px; border-radius: 10px; border: 1px solid #334155;
      background: #0f172a; color: #cbd5e1; font-weight: 700; font-size: 0.82rem;
      cursor: pointer;
    }
    .tab-btn.active {
      background: #0284c7; color: #fff; border-color: #38bdf8;
    }
    .mode-row {
      display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 10px;
    }
    .btn {
      display: flex; align-items: center; justify-content: center; gap: 6px;
      padding: 12px 10px; border-radius: 12px; font-weight: 700; font-size: 0.95rem;
      border: none; cursor: pointer; text-align: center;
    }
    .btn-live { background: #16a34a; color: #fff; }
    .btn-live.running { background: #dc2626; }
    .btn-flip { background: #334155; color: #f8fafc; }
    .btn-cam { background: #2563eb; color: #fff; }
    .btn-gal { background: #334155; color: #f8fafc; }
    input[type="file"] { display: none; }
    .slider-wrap { margin-top: 8px; display: flex; align-items: center; gap: 10px; }
    input[type="range"] { flex: 1; }
    .badge {
      display: inline-block; padding: 3px 8px; border-radius: 999px;
      font-size: 0.76rem; font-weight: 600; background: #0f172a; color: #38bdf8;
      border: 1px solid #1e40af;
    }
    .badge.fps { color: #4ade80; border-color: #166534; }
    #status { margin-top: 8px; font-size: 0.86rem; color: #cbd5e1; }
    .video-stage {
      position: relative; width: 100%; border-radius: 12px; overflow: hidden;
      background: #020617; border: 1px solid #475569; margin-top: 8px;
      display: none;
    }
    .video-stage video {
      width: 100%; height: auto; display: block;
    }
    .video-stage canvas {
      position: absolute; top: 0; left: 0; width: 100%; height: 100%;
      pointer-events: none;
    }
    #preview {
      width: 100%; border-radius: 10px; display: none; margin-top: 10px;
      border: 1px solid #475569;
    }
    .det-item {
      display: flex; justify-content: space-between; align-items: center;
      padding: 7px 10px; border-radius: 8px; background: #0f172a; margin-top: 6px;
      font-size: 0.88rem;
    }
    .det-name { font-weight: 700; color: #4ade80; }
    .det-model { font-size: 0.72rem; color: #94a3b8; margin-left: 6px; }
    .det-conf { font-weight: 600; color: #facc15; }
    details summary { cursor: pointer; color: #94a3b8; font-size: 0.82rem; }
    .class-list {
      display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px;
      font-size: 0.76rem; color: #cbd5e1;
    }
    .class-chip {
      background: #0f172a; padding: 3px 8px; border-radius: 6px; border: 1px solid #334155;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>📦 StoreBot 실시간 YOLO 테스터</h1>
    <div class="sub">
      <span class="badge" id="deviceBadge">-</span>
      <span class="badge" id="modelsBadge">로딩 중...</span>
      <span class="badge fps" id="fpsBadge">대기 중</span>
    </div>

    <div class="card">
      <div class="mode-row">
        <button class="btn btn-live" id="liveBtn">🎥 실시간 시작</button>
        <button class="btn btn-flip" id="flipBtn">🔄 전/후면 전환</button>
      </div>

      <div class="mode-row" style="margin-bottom:4px;">
        <label class="btn btn-cam">
          📷 사진 찍기
          <input id="camInput" type="file" accept="image/*" capture="environment" />
        </label>
        <label class="btn btn-gal">
          🖼️ 앨범 선택
          <input id="galInput" type="file" accept="image/*" />
        </label>
      </div>

      <div class="slider-wrap">
        <span style="font-size:0.82rem;color:#cbd5e1;">신뢰도:</span>
        <input id="confSlider" type="range" min="0.15" max="0.85" step="0.05" value="0.40" />
        <strong id="confVal">0.40</strong>
      </div>
      <div id="status">🎥 실시간 시작을 누르면 카메라 화면 위에 바로 박스가 표시됩니다.</div>
    </div>

    <div class="card" id="resultCard" style="display:none;">
      <div style="display:flex;justify-content:space-between;align-items:center;">
        <strong id="resultTitle">실시간 검출</strong>
        <span id="latencyBadge" class="badge">-</span>
      </div>
      <div class="video-stage" id="videoStage">
        <video id="video" autoplay playsinline muted></video>
        <canvas id="overlay"></canvas>
      </div>
      <img id="preview" alt="YOLO Result" />
      <div id="detList"></div>
    </div>

    <div class="card">
      <details>
        <summary id="classSummary">학습된 클래스 목록 보기</summary>
        <div class="class-list" id="classList"></div>
      </details>
    </div>
  </div>

  <script>
    const camInput = document.getElementById("camInput");
    const galInput = document.getElementById("galInput");
    const liveBtn = document.getElementById("liveBtn");
    const flipBtn = document.getElementById("flipBtn");
    const confSlider = document.getElementById("confSlider");
    const confVal = document.getElementById("confVal");
    const statusEl = document.getElementById("status");
    const resultCard = document.getElementById("resultCard");
    const resultTitle = document.getElementById("resultTitle");
    const videoStage = document.getElementById("videoStage");
    const video = document.getElementById("video");
    const overlay = document.getElementById("overlay");
    const preview = document.getElementById("preview");
    const detList = document.getElementById("detList");
    const latencyBadge = document.getElementById("latencyBadge");
    const fpsBadge = document.getElementById("fpsBadge");
    const tabBtns = document.querySelectorAll(".tab-btn");

    const CLASS_COLORS = {
      postick_front: "#eab308",
      pepero_almond_front: "#22c55e",
      drink_front: "#a855f7",
      product_back: "#f97316",
      vest: "#38bdf8",
      ciga: "#ef4444",
    };

    let selectedModel = "store_5classes";
    let lastBlob = null;
    let stream = null;
    let ws = null;
    let isLive = false;
    let inFlight = false;
    let facingMode = "environment";
    let frameTimes = [];
    const sendCanvas = document.createElement("canvas");
    const sendCtx = sendCanvas.getContext("2d");

    tabBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        tabBtns.forEach((b) => b.classList.remove("active"));
        btn.classList.add("active");
        selectedModel = btn.dataset.model;
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ model_key: selectedModel, conf: Number(confSlider.value) }));
        }
        if (!isLive && lastBlob) runPredict(lastBlob);
      });
    });

    confSlider.addEventListener("input", () => {
      confVal.textContent = Number(confSlider.value).toFixed(2);
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ model_key: selectedModel, conf: Number(confSlider.value) }));
      }
    });
    confSlider.addEventListener("change", () => {
      if (!isLive && lastBlob) runPredict(lastBlob);
    });

    async function loadHealth() {
      try {
        const res = await fetch("/health");
        const data = await res.json();
        document.getElementById("deviceBadge").textContent = data.device.toUpperCase();
        const keys = Object.keys(data.models || {});
        document.getElementById("modelsBadge").textContent = `모델 ${keys.length}개 로드됨`;
        const allNames = [];
        for (const [k, info] of Object.entries(data.models || {})) {
          for (const cls of Object.values(info.classes || {})) {
            allNames.push(`[${k}] ${cls}`);
          }
        }
        document.getElementById("classSummary").textContent = `학습된 클래스 목록 (${allNames.length}종)`;
        document.getElementById("classList").innerHTML = allNames
          .map((n) => `<span class="class-chip">${n}</span>`)
          .join("");
      } catch (e) {
        statusEl.textContent = "서버 상태 확인 실패: " + e;
      }
    }

    function renderDetList(detections) {
      if (!detections || detections.length === 0) {
        detList.innerHTML = "";
        return;
      }
      detList.innerHTML = detections
        .map(
          (d) =>
            `<div class="det-item"><div><span class="det-name" style="color:${CLASS_COLORS[d.class_name] || "#4ade80"}">${d.class_name}</span><span class="det-model">${d.model_key}</span></div><span class="det-conf">${(d.conf * 100).toFixed(1)}%</span></div>`
        )
        .join("");
    }

    function drawOverlay(detections, srcW, srcH) {
      const w = video.clientWidth;
      const h = video.clientHeight;
      if (!w || !h) return;
      if (overlay.width !== w || overlay.height !== h) {
        overlay.width = w;
        overlay.height = h;
      }
      const ctx = overlay.getContext("2d");
      ctx.clearRect(0, 0, w, h);

      const scaleX = w / srcW;
      const scaleY = h / srcH;

      for (const d of detections) {
        const [x1, y1, x2, y2] = d.bbox_xyxy;
        const bx = x1 * scaleX;
        const by = y1 * scaleY;
        const bw = (x2 - x1) * scaleX;
        const bh = (y2 - y1) * scaleY;
        const color = CLASS_COLORS[d.class_name] || "#38bdf8";

        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.strokeRect(bx, by, bw, bh);

        const label = `${d.class_name} ${(d.conf * 100).toFixed(0)}%`;
        ctx.font = "bold 14px -apple-system, sans-serif";
        const tw = ctx.measureText(label).width + 10;
        const th = 22;
        const ly = Math.max(by - th, 0);

        ctx.fillStyle = color;
        ctx.fillRect(bx, ly, tw, th);
        ctx.fillStyle = "#0f172a";
        ctx.fillText(label, bx + 5, ly + 16);
      }
    }

    function sendNextFrame() {
      if (!isLive || !ws || ws.readyState !== WebSocket.OPEN || inFlight) return;
      if (!video.videoWidth || !video.videoHeight) {
        requestAnimationFrame(sendNextFrame);
        return;
      }

      const maxDim = 640;
      let sw = video.videoWidth;
      let sh = video.videoHeight;
      if (sw > maxDim || sh > maxDim) {
        if (sw >= sh) {
          sh = Math.round((sh * maxDim) / sw);
          sw = maxDim;
        } else {
          sw = Math.round((sw * maxDim) / sh);
          sh = maxDim;
        }
      }
      sendCanvas.width = sw;
      sendCanvas.height = sh;
      sendCtx.drawImage(video, 0, 0, sw, sh);

      inFlight = true;
      sendCanvas.toBlob(
        (blob) => {
          if (!blob || !isLive || !ws || ws.readyState !== WebSocket.OPEN) {
            inFlight = false;
            return;
          }
          blob.arrayBuffer().then((buf) => {
            if (ws && ws.readyState === WebSocket.OPEN) {
              ws.send(buf);
            } else {
              inFlight = false;
            }
          });
        },
        "image/jpeg",
        0.72
      );
    }

    async function startLive() {
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        statusEl.textContent = "⚠️ 실시간 카메라는 HTTPS 접속이 필요합니다. 주소창이 https:// 인지 확인해주세요.";
        return;
      }
      try {
        stream = await navigator.mediaDevices.getUserMedia({
          video: { facingMode, width: { ideal: 1280 }, height: { ideal: 720 } },
          audio: false,
        });
      } catch (err) {
        statusEl.textContent = "카메라 권한 오류: " + err.message;
        return;
      }

      video.srcObject = stream;
      await video.play();

      preview.style.display = "none";
      videoStage.style.display = "block";
      resultCard.style.display = "block";
      resultTitle.textContent = "🎥 실시간 검출";

      const proto = location.protocol === "https:" ? "wss:" : "ws:";
      ws = new WebSocket(`${proto}//${location.host}/ws/predict`);
      ws.onopen = () => {
        isLive = true;
        inFlight = false;
        frameTimes = [];
        liveBtn.textContent = "⏹ 실시간 중지";
        liveBtn.classList.add("running");
        statusEl.textContent = "✅ 실시간 YOLO 동작 중";
        ws.send(JSON.stringify({ model_key: selectedModel, conf: Number(confSlider.value) }));
        sendNextFrame();
      };
      ws.onmessage = (ev) => {
        inFlight = false;
        const data = JSON.parse(ev.data);
        if (data.error) {
          statusEl.textContent = "오류: " + data.error;
          return;
        }
        const now = performance.now();
        frameTimes.push(now);
        while (frameTimes.length > 0 && now - frameTimes[0] > 1000) frameTimes.shift();
        fpsBadge.textContent = `${frameTimes.length} FPS`;
        latencyBadge.textContent = `${data.inference_ms} ms`;

        drawOverlay(data.detections, data.width, data.height);
        renderDetList(data.detections);

        if (isLive) {
          requestAnimationFrame(sendNextFrame);
        }
      };
      ws.onclose = () => {
        if (isLive) stopLive();
      };
    }

    function stopLive() {
      isLive = false;
      inFlight = false;
      if (ws) {
        try { ws.close(); } catch (_) {}
        ws = null;
      }
      if (stream) {
        stream.getTracks().forEach((t) => t.stop());
        stream = null;
      }
      liveBtn.textContent = "🎥 실시간 시작";
      liveBtn.classList.remove("running");
      fpsBadge.textContent = "대기 중";
      statusEl.textContent = "실시간 스트림이 중지되었습니다.";
    }

    liveBtn.addEventListener("click", () => {
      if (isLive) stopLive();
      else startLive();
    });

    flipBtn.addEventListener("click", async () => {
      facingMode = facingMode === "environment" ? "user" : "environment";
      if (isLive) {
        stopLive();
        await startLive();
      } else {
        statusEl.textContent = `카메라 방향 전환됨: ${facingMode === "environment" ? "후면" : "전면"}`;
      }
    });

    function resizeImage(file, maxDim = 1280) {
      return new Promise((resolve, reject) => {
        const img = new Image();
        const url = URL.createObjectURL(file);
        img.onload = () => {
          URL.revokeObjectURL(url);
          let { width, height } = img;
          if (width > maxDim || height > maxDim) {
            if (width >= height) {
              height = Math.round((height * maxDim) / width);
              width = maxDim;
            } else {
              width = Math.round((width * maxDim) / height);
              height = maxDim;
            }
          }
          const canvas = document.createElement("canvas");
          canvas.width = width;
          canvas.height = height;
          canvas.getContext("2d").drawImage(img, 0, 0, width, height);
          canvas.toBlob((blob) => resolve(blob), "image/jpeg", 0.88);
        };
        img.onerror = reject;
        img.src = url;
      });
    }

    async function runPredict(blob) {
      if (isLive) stopLive();
      statusEl.textContent = "⏳ GPU 추론 중...";
      const form = new FormData();
      form.append("file", blob, "capture.jpg");
      form.append("conf", confSlider.value);
      form.append("model_key", selectedModel);

      try {
        const res = await fetch("/predict", { method: "POST", body: form });
        const data = await res.json();
        if (!res.ok) {
          statusEl.textContent = "오류: " + (data.error || res.statusText);
          return;
        }
        videoStage.style.display = "none";
        resultCard.style.display = "block";
        resultTitle.textContent = "📷 사진 검출 결과";
        preview.style.display = "block";
        preview.src = "data:image/jpeg;base64," + data.image_b64;
        latencyBadge.textContent = `${data.inference_ms} ms`;

        if (data.detections.length === 0) {
          statusEl.textContent = "검출된 상품이 없습니다. 신뢰도 기준을 낮추거나 더 가까이 찍어보세요.";
          detList.innerHTML = "";
        } else {
          statusEl.textContent = `✅ ${data.detections.length}개 인식 완료`;
          renderDetList(data.detections);
        }
      } catch (e) {
        statusEl.textContent = "전송 오류: " + e;
      }
    }

    async function handleFile(e) {
      const file = e.target.files?.[0];
      if (!file) return;
      statusEl.textContent = "📐 이미지 최적화 중...";
      lastBlob = await resizeImage(file);
      await runPredict(lastBlob);
      e.target.value = "";
    }

    camInput.addEventListener("change", handleFile);
    galInput.addEventListener("change", handleFile);
    loadHealth();
  </script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return HTML_PAGE


@app.get("/health")
def health() -> JSONResponse:
    models = load_models()
    return JSONResponse(
        {
            "status": "ok",
            "device": _device,
            "models": {
                key: {
                    "label": MODEL_REGISTRY[key]["label"],
                    "classes": model.names,
                }
                for key, model in models.items()
            },
        }
    )


def _draw_boxes(img: np.ndarray, detections: list[dict]) -> np.ndarray:
    """Draw bounding boxes with model-specific colors."""
    canvas = img.copy()
    palette = {
        "store_5classes": (34, 197, 94),
        "combined": (16, 185, 129),
        "snack": (56, 189, 248),
        "pepero": (74, 222, 128),
    }
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det["bbox_xyxy"]]
        color = palette.get(det["model_key"], (0, 255, 255))
        label = f"{det['class_name']} {det['conf'] * 100:.0f}%"
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 3)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        y_top = max(y1 - th - 10, 0)
        cv2.rectangle(canvas, (x1, y_top), (x1 + tw + 8, y_top + th + 8), color, -1)
        cv2.putText(
            canvas,
            label,
            (x1 + 4, y_top + th + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (15, 23, 42),
            2,
            cv2.LINE_AA,
        )
    return canvas


@app.post("/predict")
async def predict(
    file: UploadFile = File(...),
    conf: float = Form(0.40),
    model_key: str = Form(DEFAULT_MODEL_KEY),
) -> JSONResponse:
    raw = await file.read()
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return JSONResponse({"error": "Invalid image file"}, status_code=400)

    try:
        detections, elapsed_ms = _run_inference(img, model_key, conf)
    except ValueError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)

    annotated = _draw_boxes(img, detections)
    ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    if not ok:
        return JSONResponse({"error": "Failed to encode output image"}, status_code=500)

    return JSONResponse(
        {
            "inference_ms": elapsed_ms,
            "detections": detections,
            "image_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
        }
    )


@app.websocket("/ws/predict")
async def ws_predict(websocket: WebSocket) -> None:
    await websocket.accept()
    model_key = DEFAULT_MODEL_KEY
    conf = 0.40
    try:
        while True:
            msg = await websocket.receive()
            if msg.get("type") == "websocket.disconnect":
                break
            if msg.get("text") is not None:
                cfg = json.loads(msg["text"])
                model_key = str(cfg.get("model_key", model_key))
                conf = float(cfg.get("conf", conf))
                continue
            raw = msg.get("bytes")
            if not raw:
                continue
            arr = np.frombuffer(raw, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                await websocket.send_json({"error": "Invalid frame"})
                continue
            h, w = img.shape[:2]
            detections, elapsed_ms = _run_inference(img, model_key, conf)
            await websocket.send_json(
                {
                    "inference_ms": elapsed_ms,
                    "width": w,
                    "height": h,
                    "detections": detections,
                }
            )
    except WebSocketDisconnect:
        pass


def ensure_self_signed_cert() -> tuple[Path, Path]:
    """Generate a self-signed TLS certificate for mobile getUserMedia HTTPS access."""
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    cert_path = CERT_DIR / "cert.pem"
    key_path = CERT_DIR / "key.pem"
    if not (cert_path.exists() and key_path.exists()):
        subprocess.run(
            [
                "openssl",
                "req",
                "-x509",
                "-newkey",
                "rsa:2048",
                "-keyout",
                str(key_path),
                "-out",
                str(cert_path),
                "-days",
                "365",
                "-nodes",
                "-subj",
                "/CN=storebot-yolo",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    return cert_path, key_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Run mobile YOLO live tester.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--http", action="store_true", help="Disable HTTPS (live camera blocked on LAN IP)")
    args = parser.parse_args()

    if args.http:
        uvicorn.run(app, host=args.host, port=args.port)
    else:
        cert_path, key_path = ensure_self_signed_cert()
        uvicorn.run(
            app,
            host=args.host,
            port=args.port,
            ssl_certfile=str(cert_path),
            ssl_keyfile=str(key_path),
        )


if __name__ == "__main__":
    main()
