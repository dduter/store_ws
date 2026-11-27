from datetime import date, datetime
from typing import Optional


def parse_expiration_date(raw: str) -> Optional[date]:
    """Parse YYYY.MM.DD (or YYYY-MM-DD / YYYY/MM/DD) date string."""
    cleaned = raw.strip().replace('-', '.').replace('/', '.')
    try:
        return datetime.strptime(cleaned, '%Y.%m.%d').date()
    except ValueError:
        return None


def is_expired(raw: str, today: Optional[date] = None) -> Optional[bool]:
    """Return True if expired, False if valid, None if unreadable."""
    exp_date = parse_expiration_date(raw)
    if exp_date is None:
        return None
    ref_date = today or date.today()
    return exp_date < ref_date
