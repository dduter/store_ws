"""Mobile-friendly FastAPI web server for testing multiple YOLOv8 models from a phone."""

from __future__ import annotations

import base64
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import uvicorn
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import HTMLResponse, JSONResponse
from ultralytics import YOLO

MODEL_REGISTRY: dict[str, dict[str, str]] = {
    "combined": {
        "label": "통합 모델 (아몬드 빼빼로 + 포스틱)",
        "path": "/home/ms-04/store_ws/runs/pepero_postick_train/weights/best.pt",
    },
    "snack": {
        "label": "봉지과자 (새우깡·포카칩·양파링)",
        "path": "/home/ms-04/Downloads/snack_best.pt",
    },
    "pepero": {
        "label": "박스과자 (아몬드 빼빼로)",
        "path": "/home/ms-04/store_ws/runs/pepero_train/weights/best.pt",
    },
}

app = FastAPI(title="StoreBot Mobile YOLO Tester")

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


HTML_PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=5.0" />
  <title>과자 YOLO 현장 테스터</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0; padding: 16px;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #0f172a; color: #f8fafc;
    }
    .container { max-width: 640px; margin: 0 auto; }
    h1 { font-size: 1.35rem; margin: 0 0 6px; }
    .sub { font-size: 0.85rem; color: #94a3b8; margin-bottom: 14px; display: flex; gap: 6px; flex-wrap: wrap; }
    .card {
      background: #1e293b; border-radius: 14px; padding: 14px;
      margin-bottom: 14px; border: 1px solid #334155;
    }
    .model-tabs {
      display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-bottom: 12px;
    }
    .tab-btn {
      padding: 10px 6px; border-radius: 10px; border: 1px solid #334155;
      background: #0f172a; color: #cbd5e1; font-weight: 700; font-size: 0.85rem;
      cursor: pointer;
    }
    .tab-btn.active {
      background: #0284c7; color: #fff; border-color: #38bdf8;
    }
    .btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .btn {
      display: flex; align-items: center; justify-content: center; gap: 6px;
      padding: 14px 10px; border-radius: 12px; font-weight: 700; font-size: 1rem;
      border: none; cursor: pointer; text-align: center;
    }
    .btn-cam { background: #2563eb; color: #fff; }
    .btn-gal { background: #334155; color: #f8fafc; }
    input[type="file"] { display: none; }
    .slider-wrap { margin-top: 12px; display: flex; align-items: center; gap: 10px; }
    input[type="range"] { flex: 1; }
    .badge {
      display: inline-block; padding: 3px 8px; border-radius: 999px;
      font-size: 0.78rem; font-weight: 600; background: #0f172a; color: #38bdf8;
      border: 1px solid #1e40af;
    }
    #status { margin-top: 10px; font-size: 0.9rem; color: #cbd5e1; }
    #preview {
      width: 100%; border-radius: 10px; display: none; margin-top: 10px;
      border: 1px solid #475569;
    }
    .det-item {
      display: flex; justify-content: space-between; align-items: center;
      padding: 8px 10px; border-radius: 8px; background: #0f172a; margin-top: 6px;
    }
    .det-name { font-weight: 700; color: #4ade80; }
    .det-model { font-size: 0.75rem; color: #94a3b8; margin-left: 6px; }
    .det-conf { font-weight: 600; color: #facc15; }
    details summary { cursor: pointer; color: #94a3b8; font-size: 0.85rem; }
    .class-list {
      display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px;
      font-size: 0.78rem; color: #cbd5e1;
    }
    .class-chip {
      background: #0f172a; padding: 3px 8px; border-radius: 6px; border: 1px solid #334155;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>📦 편의점 과자 YOLO 테스터</h1>
    <div class="sub">
      <span class="badge" id="deviceBadge">-</span>
      <span class="badge" id="modelsBadge">로딩 중...</span>
    </div>

    <div class="card">
      <div class="model-tabs">
        <button class="tab-btn active" data-model="combined">⭐ 빼빼로+포스틱</button>
        <button class="tab-btn" data-model="snack">🍿 기존 봉지3종</button>
        <button class="tab-btn" data-model="all">✨ 전체 모델</button>
      </div>
      <div class="btn-row">
        <label class="btn btn-cam">
          📷 카메라로 찍기
          <input id="camInput" type="file" accept="image/*" capture="environment" />
        </label>
        <label class="btn btn-gal">
          🖼️ 앨범에서 선택
          <input id="galInput" type="file" accept="image/*" />
        </label>
      </div>
      <div class="slider-wrap">
        <span style="font-size:0.85rem;color:#cbd5e1;">신뢰도 기준:</span>
        <input id="confSlider" type="range" min="0.15" max="0.85" step="0.05" value="0.40" />
        <strong id="confVal">0.40</strong>
      </div>
      <div id="status">모델을 고르고 사진을 찍으면 바로 바운딩박스를 표시합니다.</div>
    </div>

    <div class="card" id="resultCard" style="display:none;">
      <div style="display:flex;justify-content:space-between;align-items:center;">
        <strong>검출 결과</strong>
        <span id="latencyBadge" class="badge">-</span>
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
    const confSlider = document.getElementById("confSlider");
    const confVal = document.getElementById("confVal");
    const statusEl = document.getElementById("status");
    const resultCard = document.getElementById("resultCard");
    const preview = document.getElementById("preview");
    const detList = document.getElementById("detList");
    const latencyBadge = document.getElementById("latencyBadge");
    const tabBtns = document.querySelectorAll(".tab-btn");

    let selectedModel = "combined";
    let lastBlob = null;

    tabBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        tabBtns.forEach((b) => b.classList.remove("active"));
        btn.classList.add("active");
        selectedModel = btn.dataset.model;
        if (lastBlob) runPredict(lastBlob);
      });
    });

    confSlider.addEventListener("input", () => {
      confVal.textContent = Number(confSlider.value).toFixed(2);
    });
    confSlider.addEventListener("change", () => {
      if (lastBlob) runPredict(lastBlob);
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
        document.getElementById("classSummary").textContent =
          `학습된 클래스 목록 (${allNames.length}종)`;
        document.getElementById("classList").innerHTML = allNames
          .map((n) => `<span class="class-chip">${n}</span>`)
          .join("");
      } catch (e) {
        statusEl.textContent = "서버 상태 확인 실패: " + e;
      }
    }

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
        resultCard.style.display = "block";
        preview.style.display = "block";
        preview.src = "data:image/jpeg;base64," + data.image_b64;
        latencyBadge.textContent = `${data.inference_ms} ms`;

        if (data.detections.length === 0) {
          statusEl.textContent = "검출된 과자가 없습니다. 신뢰도 기준을 낮추거나 더 가까이 찍어보세요.";
          detList.innerHTML = "";
        } else {
          statusEl.textContent = `✅ ${data.detections.length}개 인식 완료`;
          detList.innerHTML = data.detections
            .map(
              (d) =>
                `<div class="det-item"><div><span class="det-name">${d.class_name}</span><span class="det-model">${d.model_key}</span></div><span class="det-conf">${(d.conf * 100).toFixed(1)}%</span></div>`
            )
            .join("");
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
        "combined": (34, 197, 94),   # BGR emerald green
        "snack": (56, 189, 248),     # BGR sky/cyan
        "pepero": (74, 222, 128),    # BGR green
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
    model_key: str = Form("all"),
) -> JSONResponse:
    raw = await file.read()
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return JSONResponse({"error": "Invalid image file"}, status_code=400)

    models = load_models()
    target_keys = list(models.keys()) if model_key == "all" else [model_key]
    target_keys = [k for k in target_keys if k in models]
    if not target_keys:
        return JSONResponse({"error": f"Unknown model: {model_key}"}, status_code=400)

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


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
