DISPOSAL_STATUSES = {'Expired', 'MarkerContaminated', 'Unreadable', 'Unjudgeable'}


def decide_product_action(status: str, requested: bool) -> str:
    """Deterministic classification rule (Section 6.2)."""
    if status in DISPOSAL_STATUSES:
        return 'disposal_bin'
    if status == 'NeedsReorientation':
        return 'reorient'
    if status == 'Normal' and requested:
        return 'checkout_bin'
    return 'keep'
