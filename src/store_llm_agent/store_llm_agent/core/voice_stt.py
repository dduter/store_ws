"""호출어(openWakeWord / hello_rokey) + PTT 감지 및 OpenAI whisper-1 STT 모듈."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile
import time
from typing import Any, Callable

from dotenv import load_dotenv
from openai import OpenAI

from store_llm_agent.core.paths import find_workspace_root

SAMPLE_RATE = 16000
FRAME_SIZE = 1280
DEFAULT_MODEL_PATH = find_workspace_root(__file__) / "config" / "voice" / "hello_rokey_8332_32.tflite"


def resolve_input_device() -> int | str | None:
    """환경변수 VOICE_MIC_DEVICE 또는 16kHz 지원 입력 장치를 자동 탐색한다."""
    override = os.environ.get("VOICE_MIC_DEVICE")
    if override:
        return int(override) if override.isdigit() else override

    try:
        import sounddevice as sd

        for idx, dev in enumerate(sd.query_devices()):
            if dev["max_input_channels"] > 0 and int(round(dev["default_samplerate"])) == SAMPLE_RATE:
                return idx
    except Exception:
        pass
    return None


class WakeWordDetector:
    """openWakeWord 기반 호출어 감지기 (corecode/VoiceProcessing 검증 로직 기반)."""

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        threshold: float = 0.3,
        sample_rate: int = SAMPLE_RATE,
        frame_size: int = FRAME_SIZE,
    ) -> None:
        self.model_path = Path(model_path)
        self.model_name = self.model_path.stem
        self.threshold = threshold
        self.sample_rate = sample_rate
        self.frame_size = frame_size
        self._model: Any = None
        self._stream: Any = None

    def open(self) -> None:
        import openwakeword
        from openwakeword.model import Model
        import sounddevice as sd

        if not self.model_path.exists():
            raise FileNotFoundError(f"호출어 모델 파일을 찾을 수 없습니다: {self.model_path}")

        openwakeword.utils.download_models()
        self._model = Model(wakeword_models=[str(self.model_path)])
        self._stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="int16",
            blocksize=self.frame_size,
            device=resolve_input_device(),
        )
        self._stream.start()

    def is_wakeup(self) -> bool:
        if self._stream is None or self._model is None:
            raise RuntimeError("WakeWordDetector.open()을 먼저 호출해야 합니다.")
        audio_chunk, _ = self._stream.read(self.frame_size)
        prediction = self._model.predict(audio_chunk.flatten())
        confidence = float(prediction.get(self.model_name, 0.0))
        return confidence > self.threshold

    def close(self) -> None:
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None


class WhisperSTT:
    """마이크 녹음 후 OpenAI whisper-1 API로 음성을 텍스트로 변환한다."""

    def __init__(
        self,
        client: Any | None = None,
        duration_sec: float = 5.0,
        samplerate: int = SAMPLE_RATE,
        env_path: str | Path | None = None,
        language: str = "ko",
        prompt: str | None = None,
    ) -> None:
        if env_path is not None:
            load_dotenv(dotenv_path=str(env_path))
        else:
            default_env = find_workspace_root(__file__) / ".env"
            if default_env.exists():
                load_dotenv(dotenv_path=str(default_env))
            else:
                load_dotenv()

        self._client = client
        self.duration_sec = duration_sec
        self.samplerate = samplerate
        self.language = language
        self.prompt = prompt

    @property
    def client(self) -> Any:
        if self._client is None:
            self._client = OpenAI()
        return self._client

    def transcribe_file(self, wav_path: str | Path) -> str:
        """WAV 파일을 OpenAI whisper-1으로 변환한다."""
        kwargs: dict[str, Any] = {
            "model": "whisper-1",
            "language": self.language,
        }
        if self.prompt:
            kwargs["prompt"] = self.prompt
        with open(wav_path, "rb") as f:
            transcript = self.client.audio.transcriptions.create(file=f, **kwargs)
        return str(transcript.text).strip()

    def speech2text(self, on_record_done: Callable[[float], None] | None = None) -> str:
        """마이크에서 duration_sec 동안 녹음 후 whisper-1으로 텍스트 변환한다."""
        import scipy.io.wavfile as wav
        import sounddevice as sd

        t0 = time.perf_counter()
        audio = sd.rec(
            int(self.duration_sec * self.samplerate),
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
            device=resolve_input_device(),
        )
        sd.wait()
        rec_elapsed = time.perf_counter() - t0
        if on_record_done is not None:
            on_record_done(rec_elapsed)

        tmp_path: str | None = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
                tmp_path = temp_wav.name
            wav.write(tmp_path, self.samplerate, audio)
            return self.transcribe_file(tmp_path)
        finally:
            if tmp_path and os.path.exists(tmp_path):
                os.remove(tmp_path)
