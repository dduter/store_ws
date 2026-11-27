"""VoiceSTT / WakeWordDetector 단위 테스트 (오프라인 Mock)."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from store_llm_agent.core.voice_stt import WakeWordDetector, WhisperSTT


def test_whisper_stt_transcribe_file(tmp_path: Path) -> None:
    dummy_wav = tmp_path / "sample.wav"
    dummy_wav.write_bytes(b"RIFF0000WAVEfmt ")

    mock_client = SimpleNamespace(
        audio=SimpleNamespace(
            transcriptions=SimpleNamespace(
                create=lambda **_kwargs: SimpleNamespace(text="  포스틱 하나 줘  ")
            )
        )
    )
    stt = WhisperSTT(client=mock_client)
    assert stt.transcribe_file(dummy_wav) == "포스틱 하나 줘"


def test_wakeword_detector_requires_open() -> None:
    detector = WakeWordDetector(model_path="/nonexistent/model.tflite")
    with pytest.raises(RuntimeError):
        detector.is_wakeup()
