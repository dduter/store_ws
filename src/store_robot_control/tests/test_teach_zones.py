import pytest

from store_robot_control.nodes.teach_zones import _update_workspace


@pytest.mark.parametrize(("choice", "entries", "zone", "original", "expected"), [
    ("4", [""], "buffer_slots", [], [[10.0, -20.0, 30.0, 25.0]]),
    ("4", ["1"], "buffer_slots", [[0.0] * 4], [[10.0, -20.0, 30.0, 25.0]]),
    ("5", [], "flip_zone", [], [10.0, -20.0, 30.0, 25.0]),
    ("6", ["2", ""], "display_tiers", {}, {"tier_2": [[10.0, -20.0, 30.0, 25.0]]}),
    ("6", ["3", "1"], "display_tiers", {"tier_3": [[0.0] * 4]},
     {"tier_3": [[10.0, -20.0, 30.0, 25.0]]}),
])
def test_update_workspace_4d(monkeypatch, choice, entries, zone, original, expected):
    responses = iter(entries)
    monkeypatch.setattr("builtins.input", lambda _: next(responses))
    workspace = {"zones": {zone: original}}
    assert _update_workspace(choice, workspace, [10, -20, 30, 15, 180, 40], [])
    assert workspace["zones"][zone] == expected


@pytest.mark.parametrize("choice", ["4", "5", "6"])
def test_update_workspace_rejects_non_top_down_pose(choice):
    workspace = {"zones": {}}
    with pytest.raises(ValueError, match="하향 자세"):
        _update_workspace(choice, workspace, [10, -20, 30, 15, 90, 40], [])
    assert workspace["zones"] == {}


@pytest.mark.parametrize("zone", ["inbound_box", "display_area"])
def test_update_workspace_sorts_both_xy_ranges(monkeypatch, zone):
    responses = iter([zone, ""])
    monkeypatch.setattr("builtins.input", lambda _: next(responses))
    workspace = {"zones": {zone: {"existing": True}}}
    corners = iter([[10, -20, 30, 0, 90, 0], [-10, 20, 40, 0, 90, 0]])
    assert _update_workspace("7", workspace, None, None, read_pose=lambda: next(corners))
    assert workspace["zones"][zone] == {
        "existing": True, "x_range": [-10.0, 10.0], "y_range": [-20.0, 20.0],
    }


def test_display_tier_rejects_fourth_slot_append(monkeypatch):
    responses = iter(["1", ""])
    monkeypatch.setattr("builtins.input", lambda _: next(responses))
    workspace = {"zones": {"display_tiers": {"tier_1": [[0.0] * 4, [1.0] * 4, [2.0] * 4]}}}
    with pytest.raises(ValueError, match="최대 3슬롯"):
        _update_workspace("6", workspace, [10, -20, 30, 15, 180, 40], [])
    assert len(workspace["zones"]["display_tiers"]["tier_1"]) == 3
