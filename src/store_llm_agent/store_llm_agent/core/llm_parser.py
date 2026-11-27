import json
from typing import Any, Dict


def parse_intent_json(raw: str) -> Dict[str, Any]:
    """Parse structured JSON output from LLM."""
    data = json.loads(raw)
    if 'intent' not in data:
        raise ValueError("Missing required 'intent' key")
    return data
