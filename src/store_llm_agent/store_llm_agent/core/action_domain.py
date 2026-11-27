"""STOREPLAN Action Domain — 불변 WorldState, 구역 분류, 4종 액션 Precondition/Effect 및 RobotActionStep 그라운딩."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Any, Final

DISPOSAL_STATUSES: Final[frozenset[str]] = frozenset({
    "Expired",
    "MarkerContaminated",
    "Unreadable",
    "Unjudgeable",
})

# 로봇 제어 회신(5df9574/8a86401): edge_pivot_flip 직접 호출 안전 상한 (Z_surface <= 200mm)
MAX_REORIENT_Z_MM: Final[float] = 200.0

DEFAULT_BUFFER_SLOTS: list[list[float]] = [
    [300.0, -280.0, 40.0, 0.0],
    [400.0, -280.0, 40.0, 0.0],
    [500.0, -280.0, 40.0, 0.0],
]


@dataclass
class RobotActionStep:
    """ExecuteRobotTask.action 1회 호출에 대응하는 단위 행동 계획."""

    action_type: str
    track_id: str = ""
    target_pose: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    dest_pose: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    product_category: str = ""
    grasp_width_mm: float = 0.0
    grasp_force_n: float = 0.0
    reason: str = ""


def is_valid_pose(pose: Any, min_len: int = 3) -> bool:
    """좌표 시퀀스가 유효한 유한 숫자로 구성되어 있는지 검사한다."""
    if not isinstance(pose, (list, tuple)) or len(pose) < min_len:
        return False
    return all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in pose[:min_len])


def pad_pose4(pose: list[float] | tuple[float, ...]) -> tuple[float, float, float, float]:
    """3D/4D 좌표를 불변 4D 튜플(x, y, z, rz)로 정규화한다."""
    vals = [float(v) for v in pose]
    while len(vals) < 4:
        vals.append(0.0)
    return (vals[0], vals[1], vals[2], vals[3])


@dataclass(frozen=True)
class ProductState:
    """단일 상품 인스턴스의 불변 Symbolic 상태."""

    track_id: str
    name: str
    yolo_class: str
    robot_category: str
    status: str
    is_backside: bool
    depth_layer: int
    location: str
    pose: tuple[float, float, float, float]
    pose_valid: bool
    expiration_raw: str = ""
    disposal_reason: str = ""
    reorient_count: int = 0
    robot_product_key: str = ""


@dataclass(frozen=True)
class BufferEntry:
    """임시 보관 버퍼로 이동된 상품의 LIFO 복원 메타데이터."""

    track_id: str
    buffer_slot: str
    original_location: str
    original_pose: tuple[float, float, float, float]
    purpose: str = "unblock"  # "unblock" | "lower_for_reorient"


@dataclass(frozen=True)
class ContainerState:
    """결제통/폐기통 컨테이너 상태."""

    container_type: str
    valid: bool
    drop_pose: tuple[float, float, float, float]
    pose_valid: bool


@dataclass(frozen=True)
class WorldState:
    """BFS 탐색 및 2차 LLM 입력용 불변(hashable) 매장 상태 스냅샷."""

    products: tuple[ProductState, ...] = ()
    containers: tuple[ContainerState, ...] = ()
    buffer_stack: tuple[BufferEntry, ...] = ()
    occupied_slots: frozenset[str] = frozenset()

    def get_product(self, track_id: str) -> ProductState | None:
        for p in self.products:
            if p.track_id == track_id:
                return p
        return None

    def get_container(self, container_type: str) -> ContainerState | None:
        for c in self.containers:
            if c.container_type == container_type:
                return c
        return None

    def replace_product(self, updated: ProductState) -> WorldState:
        new_prods = tuple(updated if p.track_id == updated.track_id else p for p in self.products)
        return replace(self, products=new_prods)

    def to_prompt_dict(self, all_tier_slots: list[str] | None = None) -> dict[str, Any]:
        """2차 LLM(TaskFormalizer) 프롬프트 주입용 요약 딕셔너리로 변환한다."""
        tier_slots = list(all_tier_slots or [])
        free_tiers = [s for s in tier_slots if s not in self.occupied_slots]
        return {
            "observed_products": [
                {
                    "track_id": p.track_id,
                    "name": p.name,
                    "yolo_class": p.yolo_class,
                    "robot_category": p.robot_category,
                    "status": p.status,
                    "is_backside": p.is_backside,
                    "depth_layer": p.depth_layer,
                    "location": p.location,
                    "z_surface_mm": round(p.pose[2], 1) if p.pose_valid else None,
                    "expiration_raw": p.expiration_raw,
                    "disposal_reason": p.disposal_reason,
                    "reorient_count": p.reorient_count,
                }
                for p in self.products
            ],
            "containers": {
                c.container_type: {"valid": c.valid, "pose_valid": c.pose_valid}
                for c in self.containers
            },
            "buffer_stack": [
                {
                    "track_id": b.track_id,
                    "buffer_slot": b.buffer_slot,
                    "original_location": b.original_location,
                    "purpose": b.purpose,
                }
                for b in self.buffer_stack
            ],
            "occupied_tier_slots": sorted(s for s in self.occupied_slots if s.startswith("tier_")),
            "free_tier_slots": free_tiers,
        }


@dataclass(frozen=True)
class SymbolicAction:
    """ActionDomain 내 기호적 단위 행동."""

    action_type: str  # "pick_place" | "move_to_buffer" | "reorient" | "restore_from_buffer"
    track_id: str
    dest_id: str = ""
    purpose: str = ""
    needs_rescan: bool = False
    reason: str = ""


class ActionDomain:
    """STOREPLAN의 4종 로봇 행동 Precondition/Effect 및 물리 좌표 그라운딩을 담당한다."""

    def __init__(
        self,
        display_tiers: dict[str, list[list[float]]] | None = None,
        buffer_slots: list[list[float]] | None = None,
        zone_bounds: dict[str, dict[str, list[float]]] | None = None,
        flip_zone: list[float] | tuple[float, ...] | None = None,
        max_reorient: int = 1,
        max_reorient_z_mm: float = MAX_REORIENT_Z_MM,
        default_grasp_force_n: float = 0.0,
    ) -> None:
        self.max_reorient = max_reorient
        self.max_reorient_z_mm = max_reorient_z_mm
        self.default_grasp_force_n = default_grasp_force_n
        self.zone_bounds = dict(zone_bounds or {})
        self.flip_zone_pose: tuple[float, float, float, float] | None = (
            pad_pose4(flip_zone) if flip_zone is not None and is_valid_pose(flip_zone, min_len=3) else None
        )

        self.tier_slot_poses: dict[str, tuple[float, float, float, float]] = {}
        for tier_name in sorted((display_tiers or {}).keys()):
            for idx, pose in enumerate((display_tiers or {})[tier_name]):
                if is_valid_pose(pose, min_len=3):
                    self.tier_slot_poses[f"{tier_name}_{idx}"] = pad_pose4(pose)

        raw_buffers = buffer_slots if buffer_slots is not None else DEFAULT_BUFFER_SLOTS
        self.buffer_slot_poses: dict[str, tuple[float, float, float, float]] = {}
        for idx, pose in enumerate(raw_buffers):
            if is_valid_pose(pose, min_len=3):
                self.buffer_slot_poses[f"buffer_{idx}"] = pad_pose4(pose)

    @property
    def all_tier_slot_ids(self) -> list[str]:
        return list(self.tier_slot_poses.keys())

    @property
    def all_buffer_slot_ids(self) -> list[str]:
        return list(self.buffer_slot_poses.keys())

    def classify_location(
        self,
        track_id: str,
        pose: tuple[float, float, float, float],
        pose_valid: bool,
        buffer_stack: tuple[BufferEntry, ...] = (),
    ) -> str:
        """상품 좌표(x, y, z)와 버퍼 스택으로 현재 논리 구역(location)을 판별한다."""
        for entry in buffer_stack:
            if entry.track_id == track_id:
                return entry.buffer_slot

        if not pose_valid:
            return "unknown"

        x, y, z, _ = pose
        # 0) 고정 Flip Zone 근접 확인 (XY <= 55mm)
        if self.flip_zone_pose is not None:
            if math.hypot(x - self.flip_zone_pose[0], y - self.flip_zone_pose[1]) <= 55.0:
                return "flip_zone"

        # 1) 버퍼 슬롯 근접 확인 (XY <= 55mm)
        for buf_id, bpose in self.buffer_slot_poses.items():
            if math.hypot(x - bpose[0], y - bpose[1]) <= 55.0:
                return buf_id

        # 2) 3단 진열대 슬롯 근접 확인 (XY <= 55mm, Z <= 45mm)
        for slot_id, spose in self.tier_slot_poses.items():
            if math.hypot(x - spose[0], y - spose[1]) <= 55.0 and abs(z - spose[2]) <= 45.0:
                return slot_id

        # 3) workspace_zones.yaml의 x/y 경계 확인
        for zone_name in ("inbound_box", "display_area"):
            zb = self.zone_bounds.get(zone_name)
            if not zb:
                continue
            xr = zb.get("x_range", [])
            yr = zb.get("y_range", [])
            if len(xr) >= 2 and len(yr) >= 2:
                if xr[0] <= x <= xr[1] and yr[0] <= y <= yr[1]:
                    return zone_name

        return "unknown"

    def build_world_state(
        self,
        snapshot: dict[str, Any],
        reorient_counts: dict[str, int] | None = None,
        buffer_stack: list[BufferEntry] | tuple[BufferEntry, ...] = (),
    ) -> WorldState:
        """KGManager.get_world_state_snapshot() 결과를 불변 WorldState로 변환한다."""
        counts = reorient_counts or {}
        b_stack = tuple(buffer_stack)
        occupied: set[str] = {entry.buffer_slot for entry in b_stack}

        prod_states: list[ProductState] = []
        for raw in snapshot.get("observed_products", []):
            yolo_cls = str(raw.get("yolo_class", ""))
            if yolo_cls.strip().lower() == "vest":
                continue

            tid = str(raw.get("track_id", ""))
            raw_pose = raw.get("pose_3d_base")
            p_valid = is_valid_pose(raw_pose, min_len=3)
            pose4 = pad_pose4(raw_pose) if p_valid else (0.0, 0.0, 0.0, 0.0)
            loc = self.classify_location(tid, pose4, p_valid, b_stack)
            if loc.startswith("tier_") or loc.startswith("buffer_") or loc == "flip_zone":
                occupied.add(loc)

            is_back = bool(raw.get("is_backside", False)) or (yolo_cls.lower() == "product_back")
            prod_states.append(
                ProductState(
                    track_id=tid,
                    name=str(raw.get("name", "")),
                    yolo_class=yolo_cls,
                    robot_category=str(raw.get("robot_category", "snack_bag")),
                    status=str(raw.get("status", "Unjudgeable")),
                    is_backside=is_back,
                    depth_layer=int(raw.get("depth_layer", 1)),
                    location=loc,
                    pose=pose4,
                    pose_valid=p_valid,
                    expiration_raw=str(raw.get("expiration_raw", "")),
                    disposal_reason=str(raw.get("disposal_reason", "")),
                    reorient_count=int(counts.get(tid, 0)),
                    robot_product_key=str(raw.get("robot_product_key", "")),
                )
            )

        cont_states: list[ContainerState] = []
        for c_type, cdata in (snapshot.get("containers") or {}).items():
            raw_drop = cdata.get("drop_pose_base")
            c_valid_pose = is_valid_pose(raw_drop, min_len=3)
            cont_states.append(
                ContainerState(
                    container_type=str(c_type),
                    valid=bool(cdata.get("valid", False)),
                    drop_pose=pad_pose4(raw_drop) if c_valid_pose else (0.0, 0.0, 0.0, 0.0),
                    pose_valid=c_valid_pose,
                )
            )

        return WorldState(
            products=tuple(prod_states),
            containers=tuple(cont_states),
            buffer_stack=b_stack,
            occupied_slots=frozenset(occupied),
        )

    def check_precondition(self, state: WorldState, action: SymbolicAction) -> tuple[bool, str]:
        """주어진 WorldState에서 SymbolicAction 실행 가능 여부(Precondition)를 검사한다."""
        prod = state.get_product(action.track_id)
        if prod is None:
            return False, f"상품 '{action.track_id}'이(가) 현재 WorldState에 없습니다."
        if not prod.pose_valid:
            return False, f"SAFETY_STOP: 상품 '{prod.track_id}'의 파지 좌표(pose_3d_base)가 누락/손상되었습니다."

        if action.action_type == "pick_place":
            dest = action.dest_id
            if prod.location in ("checkout", "disposal") or prod.location == dest:
                return False, f"상품 '{prod.track_id}'은(는) 이미 '{prod.location}'에 있습니다."

            if dest in ("checkout", "disposal"):
                cont = state.get_container(dest)
                if cont is None or not cont.valid or not cont.pose_valid:
                    return False, f"SAFETY_STOP: '{dest}' 컨테이너가 유효하지 않거나 좌표가 누락되었습니다."
                if dest == "checkout":
                    if prod.is_backside or prod.status != "Normal":
                        return False, f"상품 '{prod.track_id}'은(는) Normal 앞면 상태가 아니므로 결제통으로 이동할 수 없습니다."
                else:
                    can_dispose = (
                        prod.status in DISPOSAL_STATUSES
                        or (prod.is_backside and prod.reorient_count >= self.max_reorient)
                    )
                    if not can_dispose:
                        return False, f"상품 '{prod.track_id}'은(는) 폐기 대상 상태가 아닙니다."
                return True, "OK"

            if dest == "flip_zone":
                if self.flip_zone_pose is None:
                    return False, "고정 Flip Zone 좌표가 설정되지 않았습니다."
                if "flip_zone" in state.occupied_slots:
                    return False, "Flip Zone이 이미 점유되어 있습니다."
                if not (prod.is_backside or prod.status == "NeedsReorientation"):
                    return False, f"상품 '{prod.track_id}'은(는) 뒷면 상태가 아니므로 Flip Zone으로 이동할 수 없습니다."
                if self.flip_zone_pose[2] > self.max_reorient_z_mm:
                    return False, f"Flip Zone 높이도 reorient 상한({self.max_reorient_z_mm:.0f}mm)을 초과합니다."
                return True, "OK"

            if dest in self.tier_slot_poses:
                if dest in state.occupied_slots:
                    return False, f"진열 슬롯 '{dest}'이(가) 이미 점유되어 있습니다."
                if prod.is_backside or prod.status != "Normal":
                    return False, f"상품 '{prod.track_id}'은(는) Normal 앞면 상태가 아니므로 진열대에 배치할 수 없습니다."
                return True, "OK"

            return False, f"알 수 없는 목적지 '{dest}'입니다."

        if action.action_type == "move_to_buffer":
            buf_id = action.dest_id
            if buf_id not in self.buffer_slot_poses:
                return False, f"유효하지 않은 버퍼 슬롯 '{buf_id}'입니다."
            if buf_id in state.occupied_slots:
                return False, f"버퍼 슬롯 '{buf_id}'이(가) 이미 점유되어 있습니다."
            if prod.location in ("checkout", "disposal") or prod.location.startswith("buffer_"):
                return False, f"상품 '{prod.track_id}'은(는) 버퍼로 이동할 수 없는 위치('{prod.location}')입니다."

            if action.purpose == "lower_for_reorient":
                is_need_flip = prod.is_backside or prod.status == "NeedsReorientation"
                if not is_need_flip or prod.pose[2] <= self.max_reorient_z_mm:
                    return False, f"상품 '{prod.track_id}'은(는) reorient용 버퍼 하강 조건에 해당하지 않습니다."
                if self.buffer_slot_poses[buf_id][2] > self.max_reorient_z_mm:
                    return False, f"버퍼 슬롯 '{buf_id}' 높이도 reorient 상한({self.max_reorient_z_mm:.0f}mm)을 초과합니다."
                return True, "OK"

            # 기본 purpose == "unblock"
            if prod.depth_layer != 2:
                return False, f"상품 '{prod.track_id}'은(는) 2층 방해물(depth_layer=2)이 아닙니다."
            return True, "OK"

        if action.action_type == "reorient":
            if not (prod.is_backside or prod.status == "NeedsReorientation"):
                return False, f"상품 '{prod.track_id}'은(는) 뒷면 상태가 아닙니다."
            if prod.reorient_count >= self.max_reorient:
                return False, f"상품 '{prod.track_id}'의 최대 reorient 횟수({self.max_reorient})를 초과했습니다."
            if self.flip_zone_pose is not None and prod.location != "flip_zone":
                return False, f"상품 '{prod.track_id}'은(는) 고정 Flip Zone('flip_zone')으로 이동한 뒤 뒤집어야 합니다."
            if prod.pose[2] > self.max_reorient_z_mm:
                return False, (
                    f"상품 '{prod.track_id}' 높이(Z={prod.pose[2]:.1f}mm)가 "
                    f"reorient 상한({self.max_reorient_z_mm:.0f}mm)을 초과합니다."
                )
            return True, "OK"

        if action.action_type == "restore_from_buffer":
            if not state.buffer_stack or state.buffer_stack[-1].track_id != prod.track_id:
                return False, f"상품 '{prod.track_id}'은(는) LIFO 버퍼 스택 최상단이 아닙니다."
            entry = state.buffer_stack[-1]
            if not is_valid_pose(entry.original_pose, min_len=3):
                return False, f"SAFETY_STOP: '{prod.track_id}'의 복원 원위치 좌표가 유효하지 않습니다."
            return True, "OK"

        return False, f"지원하지 않는 액션 타입입니다: {action.action_type}"

    def apply(self, state: WorldState, action: SymbolicAction) -> WorldState:
        """Precondition을 통과한 SymbolicAction을 적용해 다음 불변 WorldState를 반환한다."""
        prod = state.get_product(action.track_id)
        if prod is None:
            raise ValueError(f"상품 '{action.track_id}'을(를) 찾을 수 없습니다.")

        occupied = set(state.occupied_slots)
        b_stack = list(state.buffer_stack)

        if action.action_type == "pick_place":
            dest = action.dest_id
            if prod.location.startswith("tier_") or prod.location.startswith("buffer_") or prod.location == "flip_zone":
                occupied.discard(prod.location)
                b_stack = [e for e in b_stack if e.track_id != prod.track_id]

            if dest in ("checkout", "disposal"):
                cont = state.get_container(dest)
                dest_pose = cont.drop_pose if cont is not None else (0.0, 0.0, 0.0, 0.0)
            elif dest == "flip_zone":
                assert self.flip_zone_pose is not None
                dest_pose = (self.flip_zone_pose[0], self.flip_zone_pose[1], self.flip_zone_pose[2], 0.0)
                occupied.add("flip_zone")
                # 진열대에서 Flip Zone으로 옮겨진 경우 검수 후 원위치 복원이 가능하도록 이력 보관
                if prod.location.startswith("tier_"):
                    b_stack.append(
                        BufferEntry(
                            track_id=prod.track_id,
                            buffer_slot="flip_zone",
                            original_location=prod.location,
                            original_pose=prod.pose,
                            purpose="lower_for_reorient",
                        )
                    )
            else:
                dest_pose = self.tier_slot_poses[dest]
                occupied.add(dest)

            updated = replace(prod, location=dest, pose=dest_pose, depth_layer=1)
            return replace(
                state.replace_product(updated),
                buffer_stack=tuple(b_stack),
                occupied_slots=frozenset(occupied),
            )

        if action.action_type == "move_to_buffer":
            buf_id = action.dest_id
            buf_pose = self.buffer_slot_poses[buf_id]
            if prod.location.startswith("tier_"):
                occupied.discard(prod.location)
            occupied.add(buf_id)
            b_stack.append(
                BufferEntry(
                    track_id=prod.track_id,
                    buffer_slot=buf_id,
                    original_location=prod.location,
                    original_pose=prod.pose,
                    purpose=action.purpose or "unblock",
                )
            )
            updated = replace(prod, location=buf_id, pose=buf_pose, depth_layer=1)
            return replace(
                state.replace_product(updated),
                buffer_stack=tuple(b_stack),
                occupied_slots=frozenset(occupied),
            )

        if action.action_type == "reorient":
            updated = replace(
                prod,
                is_backside=False,
                status="Normal" if prod.status == "NeedsReorientation" else prod.status,
                reorient_count=prod.reorient_count + 1,
            )
            return state.replace_product(updated)

        if action.action_type == "restore_from_buffer":
            entry = b_stack.pop()
            occupied.discard(entry.buffer_slot)
            if entry.original_location.startswith("tier_"):
                occupied.add(entry.original_location)
            updated = replace(prod, location=entry.original_location, pose=entry.original_pose)
            return replace(
                state.replace_product(updated),
                buffer_stack=tuple(b_stack),
                occupied_slots=frozenset(occupied),
            )

        raise ValueError(f"지원하지 않는 액션 타입입니다: {action.action_type}")

    def ground_to_robot_step(
        self,
        state: WorldState,
        action: SymbolicAction,
    ) -> tuple[RobotActionStep | None, str]:
        """SymbolicAction을 실제 ExecuteRobotTask 호출용 RobotActionStep(mm+deg)으로 변환한다."""
        ok, reason = self.check_precondition(state, action)
        if not ok:
            return None, reason

        prod = state.get_product(action.track_id)
        assert prod is not None
        target_pose = list(prod.pose)

        if action.action_type == "pick_place":
            if action.dest_id in ("checkout", "disposal"):
                cont = state.get_container(action.dest_id)
                assert cont is not None
                dest_pose = list(cont.drop_pose)
            elif action.dest_id == "flip_zone":
                assert self.flip_zone_pose is not None
                dest_pose = [self.flip_zone_pose[0], self.flip_zone_pose[1], self.flip_zone_pose[2], 0.0]
            else:
                dest_pose = list(self.tier_slot_poses[action.dest_id])
            return (
                RobotActionStep(
                    action_type="pick_place",
                    track_id=prod.track_id,
                    target_pose=target_pose,
                    dest_pose=dest_pose,
                    product_category=prod.robot_product_key or prod.robot_category,
                    grasp_width_mm=0.0,
                    grasp_force_n=self.default_grasp_force_n,
                    reason=action.reason or f"pick_place ({prod.name or prod.track_id} -> {action.dest_id})",
                ),
                "OK",
            )

        if action.action_type == "move_to_buffer":
            dest_pose = list(self.buffer_slot_poses[action.dest_id])
            return (
                RobotActionStep(
                    action_type="move_to_buffer",
                    track_id=prod.track_id,
                    target_pose=target_pose,
                    dest_pose=dest_pose,
                    product_category=prod.robot_product_key or prod.robot_category,
                    grasp_width_mm=0.0,
                    grasp_force_n=self.default_grasp_force_n,
                    reason=action.reason or f"move_to_buffer ({prod.track_id} -> {action.dest_id}, {action.purpose})",
                ),
                "OK",
            )

        if action.action_type == "reorient":
            return (
                RobotActionStep(
                    action_type="reorient",
                    track_id=prod.track_id,
                    target_pose=target_pose,
                    dest_pose=[0.0, 0.0, 0.0, 0.0],
                    product_category=prod.robot_product_key or prod.robot_category,
                    grasp_width_mm=0.0,
                    grasp_force_n=self.default_grasp_force_n,
                    reason=action.reason or f"reorient ({prod.track_id})",
                ),
                "OK",
            )

        if action.action_type == "restore_from_buffer":
            entry = state.buffer_stack[-1]
            buf_pose = list(self.buffer_slot_poses[entry.buffer_slot])
            return (
                RobotActionStep(
                    action_type="restore_from_buffer",
                    track_id=prod.track_id,
                    target_pose=buf_pose,
                    dest_pose=list(entry.original_pose),
                    product_category=prod.robot_product_key or prod.robot_category,
                    grasp_width_mm=0.0,
                    grasp_force_n=self.default_grasp_force_n,
                    reason=action.reason or f"restore_from_buffer ({prod.track_id} -> {entry.original_location})",
                ),
                "OK",
            )

        return None, f"지원하지 않는 액션 타입입니다: {action.action_type}"

    def first_free_buffer_slot(self, state: WorldState, max_z_mm: float | None = None) -> str | None:
        """비어 있는 첫 번째 버퍼 슬롯 ID를 반환한다."""
        for buf_id, bpose in self.buffer_slot_poses.items():
            if buf_id in state.occupied_slots:
                continue
            if max_z_mm is not None and bpose[2] > max_z_mm:
                continue
            return buf_id
        return None
