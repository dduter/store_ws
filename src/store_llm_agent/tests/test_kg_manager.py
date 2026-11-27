"""KGManager 단위 테스트 (확정 3종 × 각 3개 실물 = 총 9개 상품 + 모형 담배, mm+deg 기준)."""

from pathlib import Path

import pytest

from store_llm_agent.core.kg_manager import KGManager

WS_ROOT = Path(__file__).resolve().parents[3]
TTL_PATH = WS_ROOT / "config" / "ontology" / "store_ontology.ttl"
CATALOG_PATH = WS_ROOT / "config" / "products_catalog.yaml"
ZONES_PATH = WS_ROOT / "config" / "workspace_zones.yaml"


@pytest.fixture
def kg() -> KGManager:
    manager = KGManager()
    manager.load_initial_graph(TTL_PATH, CATALOG_PATH)
    return manager


def test_query_catalog_by_slots_synonyms_and_exclude(kg: KGManager) -> None:
    all_normal = kg.query_catalog_by_slots({})
    assert {item["name"] for item in all_normal} == {"포스틱", "아몬드 빼빼로", "병음료"}

    # 브랜드 / 색상(유의어 포함) / 맛(유의어 포함) / 통칭 슬롯 매칭
    assert [item["name"] for item in kg.query_catalog_by_slots({"brand": "농심"})] == ["포스틱"]
    assert [item["name"] for item in kg.query_catalog_by_slots({"color": "파란"})] == ["포스틱"]
    assert [item["name"] for item in kg.query_catalog_by_slots({"taste": "짭짤"})] == ["포스틱"]
    assert [item["name"] for item in kg.query_catalog_by_slots({"name": "빼빼로"})] == ["아몬드 빼빼로"]
    assert [item["name"] for item in kg.query_catalog_by_slots({"taste": "달달"})] == ["아몬드 빼빼로"]
    assert [item["name"] for item in kg.query_catalog_by_slots({"taste": "아몬드"})] == ["아몬드 빼빼로"]
    assert [item["name"] for item in kg.query_catalog_by_slots({"name": "음료수"})] == ["병음료"]

    # 상위 카테고리 '과자'는 봉지과자·종이상자(Snack)만 반환하고 병음료는 제외
    snacks = kg.query_catalog_by_slots({"category": "과자"})
    assert {item["name"] for item in snacks} == {"포스틱", "아몬드 빼빼로"}

    non_choco_snacks = kg.query_catalog_by_slots({"category": "과자", "exclude": ["초콜릿"]})
    assert [item["name"] for item in non_choco_snacks] == ["포스틱"]

    # 로봇 파지 카테고리 매핑 (별칭 포함) 및 로봇 구체 품목 키 조회
    assert kg.get_robot_category("postick_front") == "snack_bag"
    assert kg.get_robot_category("pepero_almond_front") == "paper_box"
    assert kg.get_robot_category("pepero_front") == "paper_box"
    assert kg.get_robot_category("drink_front") == "bottle"
    assert kg.get_robot_category("bottle") == "bottle"
    assert kg.get_robot_category("ciga") == "ciga"

    assert kg.get_robot_product_key("postick_front") == "postick"
    assert kg.get_robot_product_key("pepero_almond_front") == "pepero_almond"
    assert kg.get_robot_product_key("drink_front") == "drink"
    assert kg.get_robot_product_key("ciga") == "ciga"
    # product_back 이어도 product_name이 주어지면 구체 품목 키를 반환
    assert kg.get_robot_product_key("product_back", "아몬드 빼빼로") == "pepero_almond"

    # '미정' 플레이스홀더 속성은 개념 노드를 생성하지 않아 억지 피어 매칭을 유발하지 않음
    assert kg.query_by_relation("same_brand", name="병음료") == []
    assert kg.query_by_relation("same_taste", name="병음료") == []
    assert kg.query_by_relation("same_color", name="병음료") == []
    assert kg.infer_ontology_relations({}, raw_text="미정 하나 줘")["match_type"] == "none"

    tobacco = kg.query_catalog_by_slots({"category": "담배", "include_remote_auth": True})
    assert len(tobacco) == 1
    assert tobacco[0]["name"] == "모형 담배"
    assert tobacco[0]["robot_category"] == "ciga"


def test_sync_from_scan_and_query_sellable(kg: KGManager) -> None:
    products = [
        {
            "track_id": "t1",
            "product_name": "아몬드 빼빼로",
            "yolo_class": "pepero_almond_front",
            "is_backside": False,
            "confidence": 0.95,
            "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
            "grasp_width_mm": 0.0,
            "depth_layer": 1,
            "expiration_raw": "2027.05.01",
            "status": "Normal",
            "disposal_reason": "",
        },
        {
            "track_id": "t2",
            "product_name": "포스틱",
            "yolo_class": "postick_front",
            "is_backside": False,
            "confidence": 0.91,
            "pose_3d_base": [400.0, 180.0, 50.0, 0.0],
            "grasp_width_mm": 0.0,
            "depth_layer": 1,
            "expiration_raw": "2025.01.01",
            "status": "Expired",
            "disposal_reason": "유통기한 만료",
        },
        {
            "track_id": "t3",
            "product_name": "",
            "yolo_class": "product_back",
            "is_backside": True,
            "confidence": 0.88,
            "pose_3d_base": [450.0, 150.0, 50.0, 0.0],
            "grasp_width_mm": 0.0,
            "depth_layer": 1,
            "expiration_raw": "",
            "status": "NeedsReorientation",
            "disposal_reason": "",
        },
        {
            "track_id": "t_vest",
            "product_name": "조끼",
            "yolo_class": "vest",
            "is_backside": False,
            "confidence": 0.99,
            "pose_3d_base": [400.0, 200.0, 50.0, 0.0],
            "grasp_width_mm": 0.0,
            "depth_layer": 1,
            "expiration_raw": "",
            "status": "Normal",
            "disposal_reason": "",
        },
    ]
    containers = [
        {"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.98},
        {"container_type": "disposal", "valid": True, "drop_pose_base": [300.0, 250.0, 80.0], "confidence": 0.97},
    ]

    kg.sync_from_scan(products, containers)

    # vest 비상품은 필터링되어 재고·관측에 포함되지 않음
    snap_first = kg.get_world_state_snapshot()
    assert all(p["yolo_class"] != "vest" for p in snap_first["observed_products"])

    # t1이 다음 스캔에서 product_back으로 바뀌어도 직전 품목(pepero_almond) 기억 유지
    kg.sync_from_scan(
        [
            {
                "track_id": "t1",
                "product_name": "",
                "yolo_class": "product_back",
                "is_backside": True,
                "confidence": 0.90,
                "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
                "grasp_width_mm": 0.0,
                "depth_layer": 1,
                "expiration_raw": "",
                "status": "NeedsReorientation",
                "disposal_reason": "",
            }
        ],
        containers,
    )
    assert kg.get_world_state_snapshot()["observed_products"][0]["robot_product_key"] == "pepero_almond"
    kg.sync_from_scan(products, containers)

    sellable = kg.query_sellable_by_slots({})
    assert [item["track_id"] for item in sellable] == ["t1"]
    assert sellable[0]["pose_3d_base"] == [400.0, 100.0, 50.0, 0.0]
    assert sellable[0]["robot_category"] == "paper_box"

    # 재스캔 시 이전 인스턴스 제거 및 정적 카탈로그 3종 유지
    kg.sync_from_scan(
        [
            {
                "track_id": "t99",
                "product_name": "병음료",
                "yolo_class": "drink_front",
                "is_backside": False,
                "confidence": 0.93,
                "pose_3d_base": [350.0, 0.0, 40.0, 10.0],
                "grasp_width_mm": 0.0,
                "depth_layer": 1,
                "expiration_raw": "2027.08.01",
                "status": "Normal",
                "disposal_reason": "",
            }
        ],
        containers,
    )
    sellable_after = kg.query_sellable_by_slots({})
    assert [item["track_id"] for item in sellable_after] == ["t99"]
    assert sellable_after[0]["robot_category"] == "bottle"
    assert len(kg.query_catalog_by_slots({})) == 3


def test_query_alternatives_and_networkx_json(kg: KGManager) -> None:
    kg.sync_from_scan(
        [
            {
                "track_id": "t1",
                "product_name": "포스틱",
                "yolo_class": "postick_front",
                "is_backside": False,
                "confidence": 0.95,
                "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
                "grasp_width_mm": 0.0,
                "depth_layer": 1,
                "status": "Expired",
                "disposal_reason": "만료",
            },
            {
                "track_id": "t2",
                "product_name": "포스틱",
                "yolo_class": "postick_front",
                "is_backside": False,
                "confidence": 0.94,
                "pose_3d_base": [400.0, 0.0, 50.0, 0.0],
                "grasp_width_mm": 0.0,
                "depth_layer": 1,
                "status": "Normal",
                "disposal_reason": "",
            },
            {
                "track_id": "t3",
                "product_name": "아몬드 빼빼로",
                "yolo_class": "pepero_almond_front",
                "is_backside": False,
                "confidence": 0.92,
                "pose_3d_base": [420.0, 100.0, 50.0, 0.0],
                "grasp_width_mm": 0.0,
                "depth_layer": 1,
                "status": "Normal",
                "disposal_reason": "",
            },
        ],
        [{"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.9}],
    )

    # 1) 동일 품목(포스틱)의 정상 재고가 남아 있으면 우선 반환
    same_alts = kg.query_alternatives({"name": "포스틱", "category": "봉지과자"}, exclude_track_ids={"t1"})
    assert [a["track_id"] for a in same_alts] == ["t2"]

    # 2) 동일 품목 재고가 모두 소진/제외되면 상위 과자(Snack) 계열인 아몬드 빼빼로로 폴백
    snack_alts = kg.query_alternatives({"name": "포스틱", "category": "봉지과자"}, exclude_track_ids={"t1", "t2"})
    assert [a["track_id"] for a in snack_alts] == ["t3"]

    kg.record_disposal("t1", "유통기한 만료")
    nx_json = kg.to_networkx_json()
    assert nx_json["stats"]["instance_count"] == 3
    assert nx_json["stats"]["container_count"] == 1
    assert nx_json["disposal_history"] == [{"track_id": "t1", "reason": "유통기한 만료"}]
    assert len(nx_json["links"]) == 3


def test_world_state_snapshot_and_same_species_fefo_ordering(kg: KGManager) -> None:
    from store_llm_agent.core.pipeline_engine import load_display_tiers, load_zone_bounds

    tiers = load_display_tiers(ZONES_PATH)
    assert set(tiers.keys()) == {"tier_1", "tier_2", "tier_3"}
    assert all(len(slots) == 3 for slots in tiers.values())

    bounds = load_zone_bounds(ZONES_PATH)
    assert set(bounds.keys()) == {"display_area", "inbound_box"}
    assert len(bounds["display_area"]["x_range"]) == 2

    # 동일 품목(포스틱 3개) 중 유통기한 임박순(FEFO) 정렬 확인
    kg.sync_from_scan(
        [
            {
                "track_id": "t_late", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [400.0, 100.0, 50.0, 0.0],
                "depth_layer": 1, "expiration_raw": "2027.12.01", "status": "Normal",
            },
            {
                "track_id": "t_soon", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [400.0, 0.0, 50.0, 0.0],
                "depth_layer": 1, "expiration_raw": "2026.12.01", "status": "Normal",
            },
            {
                "track_id": "t_mid", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [400.0, -100.0, 50.0, 0.0],
                "depth_layer": 1, "expiration_raw": "2027.05.01", "status": "Normal",
            },
        ],
        [{"container_type": "checkout", "valid": True, "drop_pose_base": [300.0, -250.0, 80.0], "confidence": 0.99}],
    )
    sellable = kg.query_sellable_by_slots({"name": "포스틱"})
    assert [s["track_id"] for s in sellable] == ["t_soon", "t_mid", "t_late"]
    assert sellable[0]["sweetness_rank"] == 1

    snap = kg.get_world_state_snapshot(display_tiers=tiers)
    assert len(snap["observed_products"]) == 3
    assert "checkout" in snap["containers"]
    assert "tier_1" in snap["display_tiers"]


def test_infer_ontology_relations(kg: KGManager) -> None:
    # 1) 정확 매칭 (과자 상위 카테고리 -> 2종 후보)
    exact_snack = kg.infer_ontology_relations({"category": "과자"}, raw_text="과자 하나 줘")
    assert exact_snack["match_type"] == "exact"
    assert {c["name"] for c in exact_snack["normal_candidates"]} == {"포스틱", "아몬드 빼빼로"}

    # 2) 유사 발음/연관 속성 추론 ("코스틱" -> 포스틱, "아몬드" -> 아몬드 빼빼로)
    rel_postick = kg.infer_ontology_relations({"name": "코스틱"}, raw_text="코스틱 하나 줘")
    assert rel_postick["match_type"] == "related"
    assert rel_postick["related_candidates"][0]["name"] == "포스틱"

    rel_almond = kg.infer_ontology_relations({}, raw_text="아몬드 맛 하나 줘")
    assert rel_almond["match_type"] == "related"
    assert rel_almond["related_candidates"][0]["name"] == "아몬드 빼빼로"

    # 3) 무관한 잡음은 온톨로지 추론 결과 없음 (억지 매핑 금지)
    rel_noise = kg.infer_ontology_relations({}, raw_text="기가 막힐 toys")
    assert rel_noise["match_type"] == "none"
    assert rel_noise["related_candidates"] == []

    # 4) OWL 클래스 계층('과자' -> Snack 2종, '음료수' -> Beverage 1종)
    rel_snack = kg.infer_ontology_relations({}, raw_text="과자 아무거나")
    assert rel_snack["match_type"] == "related"
    assert {c["name"] for c in rel_snack["related_candidates"]} == {"포스틱", "아몬드 빼빼로"}

    rel_compound = kg.infer_ontology_relations({}, raw_text="음식과자 하나만 줘볼래?")
    assert rel_compound["match_type"] == "related"
    assert {c["name"] for c in rel_compound["related_candidates"]} == {"포스틱", "아몬드 빼빼로"}

    rel_bev = kg.infer_ontology_relations({}, raw_text="음료수 아무거나")
    assert rel_bev["match_type"] == "related"
    assert [c["name"] for c in rel_bev["related_candidates"]] == ["병음료"]

    for unrelated in ("김진곤 아버지 김진곤 아버지", "부산기사 tour", "눈치껏 하나 줘"):
        assert kg.infer_ontology_relations({}, raw_text=unrelated)["match_type"] == "none"


def test_owlrl_reasoning_and_sparql_relations() -> None:
    kg = KGManager(enable_owlrl=True)
    kg.load_initial_graph(TTL_PATH, CATALOG_PATH, zones_yaml=ZONES_PATH)

    # 1) 정적 OWL 클래스/Restriction 추론: Snack 2종(포스틱·아몬드 빼빼로), Beverage 1종(병음료), RemoteRestrictedProduct(담배)
    snacks = kg.query_by_relation("by_class", class_name="Snack")
    assert {s["name"] for s in snacks} == {"포스틱", "아몬드 빼빼로"}
    bags = kg.query_by_relation("by_class", class_name="BagSnack")
    assert [b["name"] for b in bags] == ["포스틱"]
    boxes = kg.query_by_relation("by_class", class_name="BoxSnack")
    assert [b["name"] for b in boxes] == ["아몬드 빼빼로"]
    beverages = kg.query_by_relation("by_class", class_name="Beverage")
    assert [b["name"] for b in beverages] == ["병음료"]
    restricted = kg.query_by_relation("remote_restricted")
    assert [r["name"] for r in restricted] == ["모형 담배"]

    # 2) 동적 스캔 후 OWL 2 RL 전방 연쇄 추론 검증
    kg.sync_from_scan(
        [
            {
                "track_id": "t_disp", "product_name": "아몬드 빼빼로", "yolo_class": "pepero_almond_front",
                "is_backside": False, "pose_3d_base": [300.0, -150.0, 40.0, 0.0],  # tier_1_0
                "depth_layer": 1, "status": "Expired", "disposal_reason": "만료",
            },
            {
                "track_id": "t_back", "product_name": "", "yolo_class": "product_back",
                "is_backside": True, "pose_3d_base": [520.0, 0.0, 240.0, 0.0],  # tier_3_1
                "depth_layer": 1, "status": "NeedsReorientation",
            },
            {
                "track_id": "t_top", "product_name": "포스틱", "yolo_class": "postick_front",
                "is_backside": False, "pose_3d_base": [350.0, 260.0, 90.0, 0.0],  # inbound_box 2층
                "depth_layer": 2, "status": "Normal",
            },
            {
                "track_id": "t_low", "product_name": "병음료", "yolo_class": "drink_front",
                "is_backside": False, "pose_3d_base": [350.0, 260.0, 45.0, 0.0],  # inbound_box 1층
                "depth_layer": 1, "status": "Normal",
            },
        ],
        [{"container_type": "checkout", "valid": True, "drop_pose_base": [260.0, -240.0, 70.0], "confidence": 0.99}],
    )

    # 2-a) OWL Restriction: DisposalCandidate, NeedsReorientProduct 자동 분류
    assert [d["track_id"] for d in kg.query_by_relation("disposal_candidates")] == ["t_disp"]
    assert [r["track_id"] for r in kg.query_by_relation("needs_reorient")] == ["t_back"]

    # 2-b) Transitive locatedIn + propertyChainAxiom placedOnTier (slot -> tier -> display_area)
    display_items = kg.query_by_relation("in_zone", zone="display_area")
    assert {i["track_id"] for i in display_items} == {"t_disp", "t_back"}
    inbound_items = kg.query_by_relation("in_zone", zone="inbound_box")
    assert {i["track_id"] for i in inbound_items} == {"t_top", "t_low"}
    tier3_items = kg.query_by_relation("on_tier", tier="tier_3")
    assert [i["track_id"] for i in tier3_items] == ["t_back"]

    # 2-c) propertyChainAxiom (instanceOf o ofBrand/ofTaste/ofColor -> hasEffective*) 및 '미정' 제외 확인
    attrs_top = kg.query_by_relation("instance_attributes", track_id="t_top")
    assert attrs_top[0]["brands"] == ["농심"]
    assert set(attrs_top[0]["tastes"]) == {"짠맛", "감자"}
    assert attrs_top[0]["colors"] == ["파랑"]

    attrs_low = kg.query_by_relation("instance_attributes", track_id="t_low")
    assert attrs_low[0]["brands"] == []
    assert attrs_low[0]["tastes"] == []
    assert attrs_low[0]["colors"] == []

    # 2-d) inverseOf (blocks <-> blockedBy)
    blocks = kg.query_by_relation("blocking_pairs")
    assert len(blocks) == 1
    assert blocks[0]["upper_track_id"] == "t_top"
    assert blocks[0]["lower_track_id"] == "t_low"

    # 2-e) 스냅샷 & NetworkX JSON에도 OWL 추론 태그 포함
    snap = kg.get_world_state_snapshot()
    assert snap["owlrl"]["enabled"] is True
    disp_meta = next(p for p in snap["observed_products"] if p["track_id"] == "t_disp")
    assert "DisposalCandidate" in disp_meta["entailed_types"]
    assert kg.to_networkx_json()["stats"]["owlrl_triples"] > 100
