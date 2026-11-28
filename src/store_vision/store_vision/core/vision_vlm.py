import base64
import json
import os
import re
from concurrent.futures import ThreadPoolExecutor
from datetime import date


def parse_expiration_date(raw: str) -> date | None:
    match = re.fullmatch(r'(\d{4}|\d{2})([.\-/])(\d{1,2})\2(\d{1,2})', raw.strip())
    if not match:
        return None
    year, _, month, day = match.groups()
    try:
        return date(int('20' + year if len(year) == 2 else year), int(month), int(day))
    except ValueError:
        return None


def is_expired(raw: str, today: date | None = None) -> bool | None:
    exp_date = parse_expiration_date(raw)
    return exp_date < (today or date.today()) if exp_date is not None else None


def classify_inspection_result(
    expiration_raw: str, marker_contaminated: bool, *, is_backside: bool = False, today=None
) -> tuple[str, str]:
    if is_backside:
        return 'NeedsReorientation', ''
    if marker_contaminated:
        return 'MarkerContaminated', '검정 마카 오염'
    expired = is_expired(expiration_raw, today=today)
    if expired is True:
        return 'Expired', f'유통기한 만료 ({expiration_raw.strip()})'
    return ('Normal' if expired is False else 'Unreadable'), ''


def parse_vlm_json(text: str) -> dict:
    data = {}
    if isinstance(text, str):
        cleaned = text.strip()
        fenced = re.fullmatch(r'```(?:json)?\s*(.*?)\s*```', cleaned, re.DOTALL | re.IGNORECASE)
        try:
            parsed = json.loads(fenced[1] if fenced else cleaned)
            if isinstance(parsed, dict):
                data = parsed
        except ValueError:
            pass

    expiration = data.get('expiration_raw')
    hint = data.get('product_hint')
    return {
        'expiration_raw': expiration.strip() if isinstance(expiration, str) else '',
        'marker_contaminated': data.get('marker_contaminated') is True,
        'product_hint': hint.strip() if isinstance(hint, str) else '',
    }


def inspect_crop_with_openai(
    crop_bgr, *, api_key=None, model='gpt-4o', timeout_sec=12.0, today=None, client=None
) -> dict:
    import cv2

    def inspect(api_client, image_url):
        return api_client.chat.completions.create(
            model=model,
            timeout=timeout_sec,
            response_format={'type': 'json_object'},
            messages=[{'role': 'user', 'content': [
                {'type': 'text', 'text': (
                    '상품 사진의 유통기한과 검정 마카 오염을 판독하세요. '
                    'expiration_raw(읽힌 유통기한 문자열), marker_contaminated(불리언), '
                    'product_hint(상품명 문자열)를 JSON으로 반환하세요. '
                    '제조일자와 인쇄된 디자인은 제외하고, 읽을 수 없는 문자열은 빈 값으로 반환하세요.'
                )},
                {'type': 'image_url', 'image_url': {'url': image_url}},
            ]}],
        )

    result = parse_vlm_json('')
    try:
        if crop_bgr is not None and crop_bgr.size:
            ok, jpeg = cv2.imencode('.jpg', crop_bgr)
            if ok:
                image_url = 'data:image/jpeg;base64,' + base64.b64encode(jpeg.tobytes()).decode('ascii')
                if client is None:
                    from openai import OpenAI

                    with OpenAI(api_key=api_key or os.environ.get('OPENAI_API_KEY'),
                                timeout=timeout_sec, max_retries=0) as api_client:
                        response = inspect(api_client, image_url)
                else:
                    response = inspect(client, image_url)
                result = parse_vlm_json(response.choices[0].message.content)
    except Exception:
        # API, 응답 형식, 인코딩 오류는 미판독으로 처리한다.
        pass
    result['status'], result['disposal_reason'] = classify_inspection_result(
        result['expiration_raw'], result['marker_contaminated'], today=today,
    )
    return result


def inspect_crops_parallel(items, inspect_fn, max_workers=4) -> list:
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        return list(executor.map(inspect_fn, items))
