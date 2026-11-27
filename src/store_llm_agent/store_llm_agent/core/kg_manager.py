"""RDFLib + OWL 2 RL 추론기 기반 매장 지식 그래프 관리, SPARQL 관계 질의 및 NetworkX JSON 변환."""

from __future__ import annotations

from difflib import SequenceMatcher
import json
import math
from pathlib import Path
import re
import time
from typing import Any

import networkx as nx
import owlrl
from rdflib import Graph, Literal, Namespace, RDF, RDFS, URIRef
from rdflib.namespace import XSD
import yaml

STORE = Namespace("http://store.cobot.org/ontology#")

_ONTOLOGY_CLASSES = {
    "Snack": STORE.Snack,
    "BoxSnack": STORE.BoxSnack,
    "BagSnack": STORE.BagSnack,
    "Candy": STORE.Candy,
    "Beverage": STORE.Beverage,
    "Tobacco": STORE.Tobacco,
}

_CHOSEONG = "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ"
_JUNGSEONG = "ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"
_JONGSEONG = (
    "", "ㄱ", "ㄲ", "ㄳ", "ㄴ", "ㄵ", "ㄶ", "ㄷ", "ㄹ", "ㄺ", "ㄻ", "ㄼ",
    "ㄽ", "ㄾ", "ㄿ", "ㅀ", "ㅁ", "ㅂ", "ㅄ", "ㅅ", "ㅆ", "ㅇ", "ㅈ", "ㅊ",
    "ㅋ", "ㅌ", "ㅍ", "ㅎ",
)
_FILLER_TOKENS = frozenset({
    "하나", "한개", "두개", "세개", "상품", "물건", "재고", "매장",
    "주세요", "있어요", "있어", "남았어", "팔아요", "빼고", "말고", "제외", "아무거나",
})
_PLACEHOLDER_PROP_VALUES = frozenset({
    "", "기타", "해당없음", "없음", "무맛", "미정", "tbd", "unknown", "none",
})
_CLASS_KEYWORD_MAP = {
    "과자": "Snack",
    "스낵": "Snack",
    "봉지": "BagSnack",
    "봉지과자": "BagSnack",
    "박스": "BoxSnack",
    "상자": "BoxSnack",
    "박스과자": "BoxSnack",
    "상자과자": "BoxSnack",
    "종이상자": "BoxSnack",
    "캔디": "Candy",
    "사탕": "Candy",
    "병": "Beverage",
    "병음료": "Beverage",
    "음료": "Beverage",
    "음료수": "Beverage",
    "마실것": "Beverage",
    "담배": "Tobacco",
}

_SLOT_SYNONYM_MAP: dict[str, dict[str, str]] = {
    "name": {
        "빼빼로": "아몬드 빼빼로",
        "아몬드빼빼로": "아몬드 빼빼로",
        "음료": "병음료",
        "음료수": "병음료",
        "물": "병음료",
        "bottle": "병음료",
    },
    "color": {
        "파란": "파랑",
        "파란색": "파랑",
        "파랑색": "파랑",
        "초록색": "초록",
        "녹색": "초록",
        "하얀": "하양",
        "하얀색": "하양",
        "흰색": "하양",
    },
    "taste": {
        "짭짤": "짠맛",
        "짭짤한": "짠맛",
        "짭짤한맛": "짠맛",
        "짠": "짠맛",
        "달달": "단맛",
        "달달한": "단맛",
        "달달한맛": "단맛",
        "달콤": "단맛",
        "달콤한": "단맛",
        "달콤한맛": "단맛",
        "단": "단맛",
    },
    "category": {
        "봉지": "봉지과자",
        "박스": "종이상자",
        "상자": "종이상자",
        "박스과자": "종이상자",
        "상자과자": "종이상자",
        "병": "병음료",
        "음료": "병음료",
        "음료수": "병음료",
    },
}
_OWL_USER_CLASSES = (
    "DisposalCandidate",
    "NeedsReorientProduct",
    "RemoteRestrictedProduct",
    "Snack",
    "BagSnack",
    "BoxSnack",
    "Candy",
    "Beverage",
    "Tobacco",
)


def _to_jamo(text: str) -> str:
    """한글 음절을 자모 단위로 분해해 발음 유사도 비교에 사용한다."""
    out: list[str] = []
    for ch in text.lower():
        code = ord(ch)
        if 0xAC00 <= code <= 0xD7A3:
            offset = code - 0xAC00
            cho = offset // (21 * 28)
            jung = (offset % (21 * 28)) // 28
            jong = offset % 28
            out.append(_CHOSEONG[cho])
            out.append(_JUNGSEONG[jung])
            if jong:
                out.append(_JONGSEONG[jong])
        elif ch.isalnum():
            out.append(ch)
    return "".join(out)


def _ontology_similarity(query_token: str, prop_value: str) -> float:
    """질의 토큰과 온톨로지 속성 문자열 간의 포함·자모 유사도를 계산한다."""
    q = query_token.strip().lower()
    p = prop_value.strip().lower()
    if not q or not p or p in _PLACEHOLDER_PROP_VALUES:
        return 0.0
    if q in p or p in q:
        return 1.0
    if len(q) < 2:
        return 0.0
    sub_parts = [part for part in re.split(r"[(),/\s]+", p) if len(part) >= 2]
    q_jamo = _to_jamo(q)
    # 조사·어미가 붙은 한국어 어절(예: '눈심껏' -> '눈심')도 비교할 수 있도록 앞부분 어근 후보 생성
    q_prefixes = [q[:k] for k in range(2, len(q))] if len(q) >= 3 else []
    best = 0.0
    for target in [p, *sub_parts]:
        if q in target or target in q:
            return 0.95
        t_jamo = _to_jamo(target)
        # 짧은 1~2음절 오탐을 막기 위해 퍼지 자모 매칭은 자모 길이 5 이상부터 허용
        if len(q_jamo) >= 5 and len(t_jamo) >= 5:
            best = max(best, SequenceMatcher(None, q_jamo, t_jamo).ratio())
        for pref in q_prefixes:
            pref_jamo = _to_jamo(pref)
            if len(pref_jamo) >= 5 and len(t_jamo) >= 5:
                pref_ratio = SequenceMatcher(None, pref_jamo, t_jamo).ratio()
                if pref_ratio >= 0.66:
                    best = max(best, 0.75)
    return best


def _extract_concept_tokens(raw: str) -> list[str]:
    """'짭짤한맛(양파, 감자)' 같은 복합 속성 문자열에서 온톨로지 개념 토큰을 분리한다."""
    tokens: list[str] = []
    for part in re.split(r"[(),/\s·]+", raw or ""):
        cleaned = part.strip()
        if cleaned and cleaned.lower() not in _PLACEHOLDER_PROP_VALUES and cleaned not in tokens:
            tokens.append(cleaned)
    return tokens


def _safe_uri_id(prefix: str, value: str) -> URIRef:
    slug = re.sub(r"[^0-9A-Za-z가-힣_]+", "_", value.strip())
    return STORE[f"{prefix}_{slug}"]


def _escape_sparql_str(value: str) -> str:
    """SPARQL 문자열 리터럴 내부의 특수문자를 이스케이프한다."""
    return (
        str(value)
        .replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", " ")
        .replace("\r", " ")
    )


def _get_field(obj: Any, key: str, default: Any = None) -> Any:
    """dict 또는 ROS 메시지 객체에서 필드 값을 안전하게 추출한다."""
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _normalize_slot_value(slot_key: str, raw_val: str) -> str:
    cleaned = str(raw_val).strip()
    low = cleaned.lower()
    table = _SLOT_SYNONYM_MAP.get(slot_key)
    if table and low in table:
        return table[low]
    return cleaned


def _normalize_exclude_term(raw_exc: str) -> str:
    cleaned = str(raw_exc).strip()
    low = cleaned.lower()
    for table in _SLOT_SYNONYM_MAP.values():
        if low in table:
            return table[low]
    return cleaned


def _build_slot_filters(slots: dict[str, Any]) -> str:
    """슬롯 딕셔너리를 SPARQL FILTER 절로 변환한다."""
    filters: list[str] = []
    field_var_map = {
        "name": "?name",
        "brand": "?brand",
        "color": "?color",
        "taste": "?taste",
        "category": "?category",
    }
    for slot_key, var_name in field_var_map.items():
        val = slots.get(slot_key)
        if val:
            norm_val = _normalize_slot_value(slot_key, str(val))
            # 상위 카테고리 "과자"/"스낵"은 봉지과자·종이상자 등 Snack 계열 전체 매칭
            if slot_key == "category" and norm_val in ("과자", "스낵"):
                filters.append(
                    'FILTER(CONTAINS(LCASE(STR(?category)), "과자") '
                    '|| CONTAINS(LCASE(STR(?category)), "상자") '
                    '|| CONTAINS(LCASE(STR(?category)), "캔디"))'
                )
            else:
                escaped = _escape_sparql_str(norm_val).lower()
                filters.append(f'FILTER(CONTAINS(LCASE(STR({var_name})), "{escaped}"))')

    excludes = slots.get("exclude") or []
    for exc in excludes:
        if not exc:
            continue
        norm_exc = _normalize_exclude_term(str(exc))
        escaped = _escape_sparql_str(norm_exc).lower()
        concat_expr = (
            'CONCAT(LCASE(STR(?name)), " ", LCASE(STR(?brand)), " ", '
            'LCASE(STR(?color)), " ", LCASE(STR(?taste)), " ", LCASE(STR(?category)))'
        )
        filters.append(f'FILTER(!CONTAINS({concat_expr}, "{escaped}"))')

    if not slots.get("include_remote_auth", False):
        filters.append("FILTER(?remoteAuth = false)")

    return "\n        ".join(filters)


class KGManager:
    """RDFLib + OWL 2 RL 추론기 기반 온톨로지 그래프와 SPARQL 관계 질의를 관리한다."""

    def __init__(self, enable_owlrl: bool = True) -> None:
        self.enable_owlrl = enable_owlrl
        self.graph: Graph = Graph()
        self.graph.bind("store", STORE)
        self._static_asserted: Graph = Graph()
        self._static_asserted.bind("store", STORE)
        self._catalog_by_yolo: dict[str, URIRef] = {}
        self._catalog_by_name: dict[str, URIRef] = {}
        self._catalog_robot_key: dict[URIRef, str] = {}
        self._last_known_product_by_track: dict[str, tuple[str, str]] = {}
        self._disposal_history: list[dict[str, str]] = []
        self._display_tiers: dict[str, list[list[float]]] = {}
        self._buffer_slots: list[list[float]] = []
        self._zone_bounds: dict[str, dict[str, list[float]]] = {}
        self.last_reasoning_ms: float = 0.0

    def _clone_graph(self, src: Graph) -> Graph:
        dst = Graph()
        dst.bind("store", STORE)
        for triple in src:
            dst.add(triple)
        return dst

    def _run_owlrl(self, target: Graph) -> float:
        if not self.enable_owlrl:
            return 0.0
        t0 = time.perf_counter()
        owlrl.DeductiveClosure(owlrl.OWLRL_Semantics).expand(target)
        elapsed = (time.perf_counter() - t0) * 1000.0
        self.last_reasoning_ms = elapsed
        return elapsed

    def _add_attribute_concepts(
        self,
        graph: Graph,
        prod_uri: URIRef,
        prop: URIRef,
        cls: URIRef,
        prefix: str,
        raw_value: str,
    ) -> None:
        for token in _extract_concept_tokens(raw_value):
            concept_uri = _safe_uri_id(prefix, token)
            graph.add((concept_uri, RDF.type, cls))
            graph.add((concept_uri, STORE.hasLabel, Literal(token, datatype=XSD.string)))
            graph.add((prod_uri, prop, concept_uri))

    def load_initial_graph(
        self,
        ttl_path: str | Path,
        catalog_yaml: str | Path | None = None,
        zones_yaml: str | Path | None = None,
    ) -> None:
        """온톨로지 스키마(.ttl), 정적 카탈로그(.yaml), 구역 토폴로지(.yaml)를 로드하고 OWL 2 RL 추론을 초기화한다."""
        self._static_asserted = Graph()
        self._static_asserted.bind("store", STORE)
        self._static_asserted.parse(str(ttl_path), format="turtle")
        self._catalog_by_yolo.clear()
        self._catalog_by_name.clear()
        self._catalog_robot_key.clear()
        self._last_known_product_by_track.clear()
        self._disposal_history.clear()
        self._display_tiers.clear()
        self._buffer_slots.clear()
        self._zone_bounds.clear()

        if catalog_yaml is not None:
            with open(catalog_yaml, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            raw_products = data.get("products", {})
            categories_cfg = data.get("categories", {})

            items: list[dict[str, Any]] = []
            if isinstance(raw_products, dict):
                for yolo_key, cfg in raw_products.items():
                    if not isinstance(cfg, dict):
                        continue
                    entry = dict(cfg)
                    entry.setdefault("id", yolo_key)
                    entry.setdefault("yolo_class", yolo_key)
                    items.append(entry)
            elif isinstance(raw_products, list):
                items = [entry for entry in raw_products if isinstance(entry, dict)]

            for item in items:
                item_id = str(item["id"])
                uri = STORE[f"cat_{item_id}"]
                self._static_asserted.add((uri, RDF.type, STORE.CatalogProduct))

                ont_cls = _ONTOLOGY_CLASSES.get(item.get("ontology_class", ""))
                if ont_cls is not None:
                    self._static_asserted.add((uri, RDF.type, ont_cls))

                name = str(item.get("name", item_id))
                yolo_cls = str(item.get("yolo_class", item_id))
                robot_cat = str(item.get("robot_category") or item.get("category") or "snack_bag")
                cat_meta = categories_cfg.get(robot_cat, {})
                display_cat = str(
                    item.get("display_category")
                    or cat_meta.get("display_name")
                    or item.get("category", "")
                )
                grasp_w = float(item.get("grasp_width_mm", cat_meta.get("grasp_width_mm", 0.0)))
                size_mm = float(item.get("size_mm", 0.0))
                sweetness_rank = int(item.get("sweetness_rank", 0))
                brand_str = str(item.get("brand", ""))
                color_str = str(item.get("color", ""))
                taste_str = str(item.get("taste", ""))

                self._static_asserted.add((uri, STORE.hasName, Literal(name, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasYoloClass, Literal(yolo_cls, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasBrand, Literal(brand_str, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasColor, Literal(color_str, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasTaste, Literal(taste_str, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasCategory, Literal(display_cat, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasRobotCategory, Literal(robot_cat, datatype=XSD.string)))
                self._static_asserted.add((uri, STORE.hasSizeMm, Literal(size_mm, datatype=XSD.float)))
                self._static_asserted.add((uri, STORE.hasSweetnessRank, Literal(sweetness_rank, datatype=XSD.integer)))
                self._static_asserted.add((uri, STORE.hasGraspWidthMm, Literal(grasp_w, datatype=XSD.float)))
                self._static_asserted.add((
                    uri,
                    STORE.requiresRemoteAuth,
                    Literal(bool(item.get("requires_remote_auth", False)), datatype=XSD.boolean),
                ))

                self._add_attribute_concepts(
                    self._static_asserted, uri, STORE.ofBrand, STORE.Brand, "brand", brand_str
                )
                self._add_attribute_concepts(
                    self._static_asserted, uri, STORE.ofColor, STORE.Color, "color", color_str
                )
                self._add_attribute_concepts(
                    self._static_asserted, uri, STORE.ofTaste, STORE.Taste, "taste", taste_str
                )

                self._catalog_robot_key[uri] = item_id
                self._catalog_by_yolo[yolo_cls.lower()] = uri
                self._catalog_by_yolo.setdefault(item_id.lower(), uri)
                self._catalog_by_name[name.lower()] = uri
                self._catalog_by_name.setdefault(item_id.lower(), uri)
                for alias in item.get("aliases") or []:
                    alias_clean = str(alias).strip().lower()
                    if alias_clean:
                        self._catalog_by_yolo.setdefault(alias_clean, uri)
                        self._catalog_by_name.setdefault(alias_clean, uri)

        if zones_yaml is not None:
            self.register_topology(zones_yaml=zones_yaml)
        else:
            self.graph = self._clone_graph(self._static_asserted)
            self._run_owlrl(self.graph)

    def register_topology(
        self,
        display_tiers: dict[str, list[list[float]]] | None = None,
        buffer_slots: list[list[float]] | None = None,
        zone_bounds: dict[str, dict[str, list[float]]] | None = None,
        zones_yaml: str | Path | None = None,
    ) -> None:
        """매장 구역(진열대 단·슬롯, 버퍼 슬롯, 입고/진열 경계)을 온톨로지 공간 계층에 등록한다."""
        if zones_yaml is not None:
            path = Path(zones_yaml)
            if path.exists():
                with open(path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                zones = data.get("zones") or {}
                raw_tiers = zones.get("display_tiers") or {}
                if isinstance(raw_tiers, dict) and display_tiers is None:
                    display_tiers = {
                        str(k): [[float(v) for v in p] for p in slots]
                        for k, slots in raw_tiers.items()
                        if isinstance(slots, list)
                    }
                raw_buf = zones.get("buffer_slots") or []
                if isinstance(raw_buf, list) and buffer_slots is None:
                    buffer_slots = [[float(v) for v in p] for p in raw_buf]
                if zone_bounds is None:
                    zone_bounds = {}
                    for zn in ("display_area", "inbound_box"):
                        raw_z = zones.get(zn)
                        if isinstance(raw_z, dict) and "x_range" in raw_z and "y_range" in raw_z:
                            zone_bounds[zn] = {
                                "x_range": [float(v) for v in raw_z["x_range"][:2]],
                                "y_range": [float(v) for v in raw_z["y_range"][:2]],
                            }

        self._display_tiers = dict(display_tiers or {})
        self._buffer_slots = [list(p) for p in (buffer_slots or [])]
        self._zone_bounds = dict(zone_bounds or {})

        # 공간 계층(ShelfSlot -> ShelfTier -> DisplayArea, BufferSlot -> BufferZone, InboundBoxZone)
        self._static_asserted.add((STORE.zone_display_area, RDF.type, STORE.DisplayArea))
        self._static_asserted.add((STORE.zone_display_area, STORE.hasLabel, Literal("display_area", datatype=XSD.string)))
        self._static_asserted.add((STORE.zone_inbound_box, RDF.type, STORE.InboundBoxZone))
        self._static_asserted.add((STORE.zone_inbound_box, STORE.hasLabel, Literal("inbound_box", datatype=XSD.string)))
        self._static_asserted.add((STORE.zone_buffer, RDF.type, STORE.BufferZone))
        self._static_asserted.add((STORE.zone_buffer, STORE.hasLabel, Literal("buffer", datatype=XSD.string)))

        for tier_name, slots in sorted(self._display_tiers.items()):
            tier_uri = STORE[tier_name]
            self._static_asserted.add((tier_uri, RDF.type, STORE.ShelfTier))
            self._static_asserted.add((tier_uri, STORE.locatedIn, STORE.zone_display_area))
            self._static_asserted.add((tier_uri, STORE.hasLabel, Literal(tier_name, datatype=XSD.string)))
            m = re.search(r"(\d+)$", tier_name)
            if m:
                self._static_asserted.add((tier_uri, STORE.hasTierLevel, Literal(int(m.group(1)), datatype=XSD.integer)))
            for idx, pose in enumerate(slots):
                slot_id = f"{tier_name}_{idx}"
                slot_uri = STORE[f"slot_{slot_id}"]
                self._static_asserted.add((slot_uri, RDF.type, STORE.ShelfSlot))
                self._static_asserted.add((slot_uri, STORE.belongsToTier, tier_uri))
                self._static_asserted.add((slot_uri, STORE.hasLabel, Literal(slot_id, datatype=XSD.string)))
                self._static_asserted.add((slot_uri, STORE.hasPose3D, Literal(json.dumps(pose), datatype=XSD.string)))

        for idx, pose in enumerate(self._buffer_slots):
            buf_id = f"buffer_{idx}"
            buf_uri = STORE[f"slot_{buf_id}"]
            self._static_asserted.add((buf_uri, RDF.type, STORE.BufferSlot))
            self._static_asserted.add((buf_uri, STORE.locatedIn, STORE.zone_buffer))
            self._static_asserted.add((buf_uri, STORE.hasLabel, Literal(buf_id, datatype=XSD.string)))
            self._static_asserted.add((buf_uri, STORE.hasPose3D, Literal(json.dumps(pose), datatype=XSD.string)))

        self.graph = self._clone_graph(self._static_asserted)
        self._run_owlrl(self.graph)

    def get_robot_category(self, yolo_class: str = "", product_name: str = "", default: str = "snack_bag") -> str:
        """yolo_class 또는 상품명으로부터 로봇 제어용 영문 대분류 카테고리(paper_box 등)를 조회한다."""
        uri = (
            self._catalog_by_yolo.get((yolo_class or "").lower())
            or self._catalog_by_name.get((product_name or "").lower())
        )
        if uri is not None:
            val = self.graph.value(uri, STORE.hasRobotCategory)
            if val is not None:
                return str(val)
        return default

    def get_robot_product_key(
        self,
        yolo_class: str = "",
        product_name: str = "",
        track_id: str = "",
        default: str = "",
    ) -> str:
        """ExecuteRobotTask.product_category에 전달할 구체 품목 키(pepero_almond, postick, drink, ciga)를 조회한다.

        product_back 감지 시에도 product_name(VLM 판별) 또는 직전 스캔(track_id) 이력이 있으면 해당 품목 키를 유지한다.
        """
        uri = (
            self._catalog_by_yolo.get((yolo_class or "").strip().lower())
            or self._catalog_by_name.get((product_name or "").strip().lower())
        )
        if uri is None and track_id and track_id in self._last_known_product_by_track:
            prev_yolo, prev_name = self._last_known_product_by_track[track_id]
            uri = (
                self._catalog_by_yolo.get(prev_yolo.lower())
                or self._catalog_by_name.get(prev_name.lower())
            )
        if uri is not None:
            return self._catalog_robot_key.get(uri, default)
        return default

    def query_catalog_by_slots(self, slots: dict[str, Any] | None = None) -> list[dict[str, Any]]:
        """스캔 전 1차 정적 카탈로그 후보 상품을 SPARQL로 조회한다."""
        filter_clause = _build_slot_filters(slots or {})
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT ?cat ?name ?yoloClass ?brand ?color ?taste ?category ?robotCategory
               ?sizeMm ?sweetnessRank ?graspWidth ?remoteAuth
        WHERE {{
            ?cat rdf:type store:CatalogProduct ;
                 store:hasName ?name ;
                 store:hasYoloClass ?yoloClass ;
                 store:hasBrand ?brand ;
                 store:hasColor ?color ;
                 store:hasTaste ?taste ;
                 store:hasCategory ?category ;
                 store:hasRobotCategory ?robotCategory ;
                 store:hasSizeMm ?sizeMm ;
                 store:hasSweetnessRank ?sweetnessRank ;
                 store:hasGraspWidthMm ?graspWidth ;
                 store:requiresRemoteAuth ?remoteAuth .
            {filter_clause}
        }}
        ORDER BY ?name
        """
        results: list[dict[str, Any]] = []
        for row in self.graph.query(query):
            results.append({
                "uri": str(row.cat),
                "name": str(row.name),
                "yolo_class": str(row.yoloClass),
                "brand": str(row.brand),
                "color": str(row.color),
                "taste": str(row.taste),
                "category": str(row.category),
                "robot_category": str(row.robotCategory),
                "size_mm": float(row.sizeMm),
                "sweetness_rank": int(row.sweetnessRank),
                "grasp_width_mm": float(row.graspWidth),
                "requires_remote_auth": bool(row.remoteAuth.toPython()),
            })
        return results

    def _resolve_catalog_uri(self, name: str = "", track_id: str = "") -> URIRef | None:
        if name and name.lower() in self._catalog_by_name:
            return self._catalog_by_name[name.lower()]
        if track_id:
            inst_uri = STORE[f"inst_{track_id}"]
            cat = self.graph.value(inst_uri, STORE.instanceOf)
            if isinstance(cat, URIRef):
                return cat
        return None

    def query_by_relation(self, relation_type: str, **params: Any) -> list[dict[str, Any]]:
        """확장형 SPARQL 관계 질의 디스패처 (OWL 2 RL 추론 결과 직접 조회).

        지원 relation_type:
        - same_brand / same_taste / same_color: 지정 상품과 브랜드·맛·색상 개념을 공유하는 피어 상품
        - by_class: OWL 클래스/추론 클래스(Snack, BagSnack, Tobacco, DisposalCandidate 등) 기준 조회
        - in_zone: OWL 이행적(locatedIn) 공간 포함 관계로 구역(display_area, inbound_box, buffer) 내 인스턴스 조회
        - on_tier: propertyChainAxiom(placedOnTier)으로 특정 단(tier_1~tier_3) 위 인스턴스 조회
        - disposal_candidates / needs_reorient / remote_restricted: OWL Restriction 자동 추론 결과 조회
        - blocking_pairs: 2층 가림(blocks / blockedBy inverseOf) 쌍 조회
        - instance_attributes: propertyChainAxiom으로 인스턴스가 상속받은 유효 브랜드/맛/색상 조회
        """
        rel = relation_type.strip().lower()
        handlers = {
            "same_brand": lambda: self._query_peer_by_concept(STORE.ofBrand, **params),
            "same_taste": lambda: self._query_peer_by_concept(STORE.ofTaste, **params),
            "same_color": lambda: self._query_peer_by_concept(STORE.ofColor, **params),
            "by_class": lambda: self._query_by_owl_class(str(params.get("class_name", "Product"))),
            "disposal_candidates": lambda: self._query_by_owl_class("DisposalCandidate"),
            "needs_reorient": lambda: self._query_by_owl_class("NeedsReorientProduct"),
            "remote_restricted": lambda: self._query_by_owl_class("RemoteRestrictedProduct"),
            "in_zone": lambda: self._query_in_zone(str(params.get("zone", "display_area"))),
            "on_tier": lambda: self._query_on_tier(str(params.get("tier", "tier_1"))),
            "blocking_pairs": self._query_blocking_pairs,
            "instance_attributes": lambda: self._query_instance_attributes(str(params.get("track_id", ""))),
        }
        handler = handlers.get(rel)
        if handler is None:
            raise ValueError(f"지원하지 않는 온톨로지 관계 질의입니다: {relation_type}")
        return handler()

    def _query_peer_by_concept(
        self,
        concept_prop: URIRef,
        name: str = "",
        track_id: str = "",
        include_remote_auth: bool = False,
    ) -> list[dict[str, Any]]:
        src_uri = self._resolve_catalog_uri(name=name, track_id=track_id)
        if src_uri is None:
            return []
        prop_name = str(concept_prop).rsplit("#", 1)[-1]
        auth_filter = "" if include_remote_auth else "FILTER(?remoteAuth = false)"
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT DISTINCT ?peer ?name ?yoloClass ?brand ?color ?taste ?category ?remoteAuth ?conceptLabel
        WHERE {{
            <{src_uri}> store:{prop_name} ?concept .
            ?concept store:hasLabel ?conceptLabel .
            ?peer rdf:type store:CatalogProduct ;
                  store:{prop_name} ?concept ;
                  store:hasName ?name ;
                  store:hasYoloClass ?yoloClass ;
                  store:hasBrand ?brand ;
                  store:hasColor ?color ;
                  store:hasTaste ?taste ;
                  store:hasCategory ?category ;
                  store:requiresRemoteAuth ?remoteAuth .
            FILTER(?peer != <{src_uri}>)
            {auth_filter}
        }}
        ORDER BY ?name
        """
        rows: list[dict[str, Any]] = []
        for row in self.graph.query(query):
            rows.append({
                "uri": str(row.peer),
                "name": str(row.name),
                "yolo_class": str(row.yoloClass),
                "brand": str(row.brand),
                "color": str(row.color),
                "taste": str(row.taste),
                "category": str(row.category),
                "shared_concept": str(row.conceptLabel),
                "requires_remote_auth": bool(row.remoteAuth.toPython()),
            })
        return rows

    def _query_by_owl_class(self, class_name: str) -> list[dict[str, Any]]:
        safe_cls = re.sub(r"[^0-9A-Za-z_]", "", class_name)
        if not safe_cls:
            return []
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT DISTINCT ?entity ?name ?trackId ?status ?yoloClass
        WHERE {{
            ?entity rdf:type store:{safe_cls} .
            OPTIONAL {{ ?entity store:hasName ?name . }}
            OPTIONAL {{ ?entity store:hasTrackId ?trackId . }}
            OPTIONAL {{ ?entity store:hasStatus ?status . }}
            OPTIONAL {{ ?entity store:hasYoloClass ?yoloClass . }}
        }}
        ORDER BY ?trackId ?name
        """
        results: list[dict[str, Any]] = []
        for row in self.graph.query(query):
            results.append({
                "uri": str(row.entity),
                "name": str(row.name or ""),
                "track_id": str(row.trackId or ""),
                "status": str(row.status or ""),
                "yolo_class": str(row.yoloClass or ""),
                "owl_class": safe_cls,
            })
        return results

    def _query_in_zone(self, zone: str) -> list[dict[str, Any]]:
        safe_zone = re.sub(r"[^0-9A-Za-z_]", "", zone)
        if not safe_zone:
            return []
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT DISTINCT ?inst ?trackId ?name ?status ?slotLabel
        WHERE {{
            ?inst rdf:type store:ProductInstance ;
                  store:hasTrackId ?trackId ;
                  store:hasName ?name ;
                  store:hasStatus ?status ;
                  store:locatedIn store:zone_{safe_zone} .
            OPTIONAL {{
                ?inst store:placedOnSlot ?slot .
                ?slot store:hasLabel ?slotLabel .
            }}
        }}
        ORDER BY ?trackId
        """
        return [
            {
                "track_id": str(row.trackId),
                "name": str(row.name),
                "status": str(row.status),
                "slot": str(row.slotLabel or safe_zone),
                "zone": safe_zone,
            }
            for row in self.graph.query(query)
        ]

    def _query_on_tier(self, tier: str) -> list[dict[str, Any]]:
        safe_tier = re.sub(r"[^0-9A-Za-z_]", "", tier)
        if not safe_tier:
            return []
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT DISTINCT ?inst ?trackId ?name ?status ?slotLabel
        WHERE {{
            ?inst rdf:type store:ProductInstance ;
                  store:hasTrackId ?trackId ;
                  store:hasName ?name ;
                  store:hasStatus ?status ;
                  store:placedOnTier store:{safe_tier} .
            OPTIONAL {{
                ?inst store:placedOnSlot ?slot .
                ?slot store:hasLabel ?slotLabel .
            }}
        }}
        ORDER BY ?trackId
        """
        return [
            {
                "track_id": str(row.trackId),
                "name": str(row.name),
                "status": str(row.status),
                "slot": str(row.slotLabel or ""),
                "tier": safe_tier,
            }
            for row in self.graph.query(query)
        ]

    def _query_blocking_pairs(self) -> list[dict[str, Any]]:
        query = """
        PREFIX store: <http://store.cobot.org/ontology#>
        SELECT DISTINCT ?upperId ?upperName ?lowerId ?lowerName
        WHERE {
            ?upper store:blocks ?lower ;
                   store:hasTrackId ?upperId ;
                   store:hasName ?upperName .
            ?lower store:blockedBy ?upper ;
                   store:hasTrackId ?lowerId ;
                   store:hasName ?lowerName .
        }
        ORDER BY ?upperId ?lowerId
        """
        return [
            {
                "upper_track_id": str(row.upperId),
                "upper_name": str(row.upperName),
                "lower_track_id": str(row.lowerId),
                "lower_name": str(row.lowerName),
            }
            for row in self.graph.query(query)
        ]

    def _query_instance_attributes(self, track_id: str = "") -> list[dict[str, Any]]:
        tid_filter = f'FILTER(STR(?trackId) = "{_escape_sparql_str(track_id)}")' if track_id else ""
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT ?trackId ?name ?brandLabel ?tasteLabel ?colorLabel
        WHERE {{
            ?inst rdf:type store:ProductInstance ;
                  store:hasTrackId ?trackId ;
                  store:hasName ?name .
            OPTIONAL {{ ?inst store:hasEffectiveBrand/store:hasLabel ?brandLabel . }}
            OPTIONAL {{ ?inst store:hasEffectiveTaste/store:hasLabel ?tasteLabel . }}
            OPTIONAL {{ ?inst store:hasEffectiveColor/store:hasLabel ?colorLabel . }}
            {tid_filter}
        }}
        ORDER BY ?trackId
        """
        grouped: dict[str, dict[str, Any]] = {}
        for row in self.graph.query(query):
            tid = str(row.trackId)
            entry = grouped.setdefault(tid, {
                "track_id": tid,
                "name": str(row.name),
                "brands": [],
                "tastes": [],
                "colors": [],
            })
            if row.brandLabel and str(row.brandLabel) not in entry["brands"]:
                entry["brands"].append(str(row.brandLabel))
            if row.tasteLabel and str(row.tasteLabel) not in entry["tastes"]:
                entry["tastes"].append(str(row.tasteLabel))
            if row.colorLabel and str(row.colorLabel) not in entry["colors"]:
                entry["colors"].append(str(row.colorLabel))
        return list(grouped.values())

    def _sparql_concept_lookup(self, token: str) -> list[dict[str, Any]]:
        """SPARQL을 통해 개념 노드(Brand/Taste/Color) 또는 OWL 클래스 계층과 연결된 상품을 조회한다."""
        escaped = _escape_sparql_str(token).lower()
        results: list[dict[str, Any]] = []
        seen: set[str] = set()

        # 1) 개념 노드(ofBrand/ofTaste/ofColor) 그래프 순회
        concept_query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT DISTINCT ?cat ?name ?brand ?color ?taste ?category ?remoteAuth ?conceptLabel
        WHERE {{
            ?cat rdf:type store:CatalogProduct ;
                 (store:ofBrand|store:ofTaste|store:ofColor) ?concept ;
                 store:hasName ?name ;
                 store:hasBrand ?brand ;
                 store:hasColor ?color ;
                 store:hasTaste ?taste ;
                 store:hasCategory ?category ;
                 store:requiresRemoteAuth ?remoteAuth .
            ?concept store:hasLabel ?conceptLabel .
            FILTER(
                CONTAINS(LCASE(STR(?conceptLabel)), "{escaped}")
                || (STRLEN(STR(?conceptLabel)) >= 2 && CONTAINS("{escaped}", LCASE(STR(?conceptLabel))))
            )
        }}
        ORDER BY ?name
        """
        for row in self.graph.query(concept_query):
            name = str(row.name)
            if name in seen:
                continue
            seen.add(name)
            results.append({
                "name": name,
                "brand": str(row.brand),
                "color": str(row.color),
                "taste": str(row.taste),
                "category": str(row.category),
                "requires_remote_auth": bool(row.remoteAuth.toPython()),
                "reason": f"그래프 개념 '{row.conceptLabel}' 연결",
            })

        # 2) OWL 클래스 계층 매칭 (예: '과자'/'스낵' 및 '음식과자' 같은 복합어 -> store:Snack)
        tok_clean = token.strip()
        cls_name = _CLASS_KEYWORD_MAP.get(tok_clean)
        if not cls_name:
            cls_name = next(
                (cls for kw, cls in _CLASS_KEYWORD_MAP.items() if len(kw) >= 2 and tok_clean.endswith(kw)),
                None,
            )
        if cls_name:
            for item in self._query_by_owl_class(cls_name):
                name = item.get("name", "")
                if name and name not in seen and item["uri"].endswith(tuple(str(u).split("#")[-1] for u in self._catalog_by_name.values())):
                    seen.add(name)
                    results.append({
                        "name": name,
                        "reason": f"OWL 클래스 '{cls_name}' 하위 품목",
                    })
        return results

    def infer_ontology_relations(
        self,
        slots: dict[str, Any] | None = None,
        raw_text: str = "",
        threshold: float = 0.74,
    ) -> dict[str, Any]:
        """정적 SPARQL 슬롯/개념 그래프 질의와 한글 자모 유사도 추론을 통합 수행한다."""
        clean_slots = dict(slots or {})
        has_slot_filter = any(
            clean_slots.get(k) for k in ("name", "brand", "color", "taste", "category", "exclude")
        )
        if has_slot_filter:
            normal_cands = self.query_catalog_by_slots(clean_slots)
            all_cands = self.query_catalog_by_slots({**clean_slots, "include_remote_auth": True})
            if all_cands:
                return {
                    "match_type": "exact",
                    "normal_candidates": normal_cands,
                    "all_candidates": all_cands,
                    "related_candidates": [],
                    "relation_reasons": [],
                }

        tokens: list[str] = []
        for k in ("name", "brand", "color", "taste", "category"):
            val = clean_slots.get(k)
            if val:
                tokens.append(str(val).strip())
        for tok in re.findall(r"[가-힣a-zA-Z0-9]+", raw_text or ""):
            if len(tok) >= 2 and tok not in _FILLER_TOKENS and tok not in tokens:
                tokens.append(tok)

        # 1) SPARQL 개념 노드(ofBrand/ofTaste/ofColor) 및 OWL 클래스 계층 우선 추론
        graph_hits: list[dict[str, Any]] = []
        graph_reasons: list[str] = []
        seen_names: set[str] = set()
        for tok in tokens:
            if len(tok) < 2:
                continue
            for hit in self._sparql_concept_lookup(tok):
                if hit["name"] not in seen_names:
                    seen_names.add(hit["name"])
                    graph_hits.append(hit)
                    graph_reasons.append(f"{hit['name']}: {hit['reason']}")

        if graph_hits:
            return {
                "match_type": "related",
                "normal_candidates": [],
                "all_candidates": [],
                "related_candidates": graph_hits,
                "relation_reasons": graph_reasons,
            }

        # 2) 카탈로그 리터럴 속성 기반 자모/문자열 유사도 추론 (유사 발음 백업)
        catalog_all = self.query_catalog_by_slots({"include_remote_auth": True})
        scored: list[tuple[float, dict[str, Any], str]] = []
        prop_labels = (
            ("name", "상품명"),
            ("brand", "브랜드"),
            ("taste", "맛"),
            ("color", "색상"),
            ("category", "종류"),
        )
        for item in catalog_all:
            best_score = 0.0
            best_reason = ""
            for tok in tokens:
                for prop_key, prop_ko in prop_labels:
                    prop_val = str(item.get(prop_key, ""))
                    if not prop_val:
                        continue
                    score = _ontology_similarity(tok, prop_val)
                    if score >= threshold and score > best_score:
                        best_score = score
                        best_reason = f"{prop_ko}='{prop_val}' (입력 '{tok}')"
            if best_score >= threshold:
                scored.append((best_score, item, best_reason))

        scored.sort(key=lambda x: (-x[0], str(x[1].get("name", ""))))
        related = [item for _, item, _ in scored]
        reasons = [f"{item['name']}: {reason}" for _, item, reason in scored]
        return {
            "match_type": "related" if related else "none",
            "normal_candidates": [],
            "all_candidates": [],
            "related_candidates": related,
            "relation_reasons": reasons,
        }

    def _infer_spatial_anchor(self, pose_mm: list[float]) -> tuple[URIRef | None, str]:
        """좌표로부터 인스턴스가 속한 슬롯/구역 URI를 판별한다."""
        if len(pose_mm) < 3 or not all(math.isfinite(float(v)) for v in pose_mm[:3]):
            return None, "unknown"
        x, y, z = float(pose_mm[0]), float(pose_mm[1]), float(pose_mm[2])

        for idx, b_pose in enumerate(self._buffer_slots):
            if math.hypot(x - b_pose[0], y - b_pose[1]) <= 55.0:
                return STORE[f"slot_buffer_{idx}"], f"buffer_{idx}"

        for tier_name, slots in self._display_tiers.items():
            for idx, s_pose in enumerate(slots):
                if math.hypot(x - s_pose[0], y - s_pose[1]) <= 55.0 and abs(z - s_pose[2]) <= 45.0:
                    return STORE[f"slot_{tier_name}_{idx}"], f"{tier_name}_{idx}"

        ib = self._zone_bounds.get("inbound_box")
        if ib and ib["x_range"][0] <= x <= ib["x_range"][1] and ib["y_range"][0] <= y <= ib["y_range"][1]:
            return STORE.zone_inbound_box, "inbound_box"

        da = self._zone_bounds.get("display_area")
        if da and da["x_range"][0] <= x <= da["x_range"][1] and da["y_range"][0] <= y <= da["y_range"][1]:
            return STORE.zone_display_area, "display_area"

        return None, "unknown"

    def sync_from_scan(
        self,
        products: list[Any],
        containers: list[Any] | None = None,
    ) -> None:
        """최신 ScanWorkspace 결과로 동적 그래프를 재구성하고 OWL 2 RL 추론을 실행한다."""
        g = self._clone_graph(self._static_asserted)

        layer1_items: list[tuple[URIRef, list[float]]] = []
        layer2_items: list[tuple[URIRef, list[float]]] = []

        for idx, prod in enumerate(products):
            yolo_class = str(_get_field(prod, "yolo_class", ""))
            # 비상품 클래스(vest: 형광조끼)는 로봇 파지·매장 재고 대상으로 유입되지 않도록 필터링
            if yolo_class.strip().lower() == "vest":
                continue

            track_id = str(_get_field(prod, "track_id", f"unk_{idx}"))
            inst_uri = STORE[f"inst_{track_id}"]
            g.add((inst_uri, RDF.type, STORE.ProductInstance))

            product_name = str(_get_field(prod, "product_name", yolo_class))
            cat_uri = (
                self._catalog_by_yolo.get(yolo_class.lower())
                or self._catalog_by_name.get(product_name.lower())
            )
            if cat_uri is not None:
                g.add((inst_uri, STORE.instanceOf, cat_uri))
                self._last_known_product_by_track[track_id] = (yolo_class, product_name)

            pose_raw = [float(v) for v in _get_field(prod, "pose_3d_base", [0.0, 0.0, 0.0, 0.0])]
            pose_json = json.dumps(pose_raw)
            depth_layer = int(_get_field(prod, "depth_layer", 1))
            is_backside = bool(_get_field(prod, "is_backside", False)) or (yolo_class.lower() == "product_back")

            g.add((inst_uri, STORE.hasTrackId, Literal(track_id, datatype=XSD.string)))
            g.add((inst_uri, STORE.hasName, Literal(product_name, datatype=XSD.string)))
            g.add((inst_uri, STORE.hasYoloClass, Literal(yolo_class, datatype=XSD.string)))
            g.add((inst_uri, STORE.hasStatus, Literal(str(_get_field(prod, "status", "Unjudgeable")), datatype=XSD.string)))
            g.add((inst_uri, STORE.hasDisposalReason, Literal(str(_get_field(prod, "disposal_reason", "")), datatype=XSD.string)))
            g.add((inst_uri, STORE.hasExpirationRaw, Literal(str(_get_field(prod, "expiration_raw", "")), datatype=XSD.string)))
            g.add((inst_uri, STORE.hasPose3D, Literal(pose_json, datatype=XSD.string)))
            g.add((inst_uri, STORE.depthLayer, Literal(depth_layer, datatype=XSD.integer)))
            g.add((inst_uri, STORE.isBackside, Literal(is_backside, datatype=XSD.boolean)))
            g.add((inst_uri, STORE.hasConfidence, Literal(float(_get_field(prod, "confidence", 0.0)), datatype=XSD.float)))
            g.add((inst_uri, STORE.hasGraspWidthMm, Literal(float(_get_field(prod, "grasp_width_mm", 0.0)), datatype=XSD.float)))

            anchor_uri, _ = self._infer_spatial_anchor(pose_raw)
            if anchor_uri is not None:
                g.add((inst_uri, STORE.placedOnSlot, anchor_uri))

            if len(pose_raw) >= 2:
                if depth_layer >= 2:
                    layer2_items.append((inst_uri, pose_raw))
                else:
                    layer1_items.append((inst_uri, pose_raw))

        for upper_uri, u_pose in layer2_items:
            for lower_uri, l_pose in layer1_items:
                if math.hypot(u_pose[0] - l_pose[0], u_pose[1] - l_pose[1]) <= 65.0:
                    g.add((upper_uri, STORE.blocks, lower_uri))

        for cont in containers or []:
            c_type = str(_get_field(cont, "container_type", "unknown")).lower()
            cont_uri = STORE[f"container_{c_type}"]
            g.add((cont_uri, RDF.type, STORE.Container))
            if c_type == "checkout":
                g.add((cont_uri, RDF.type, STORE.CheckoutContainer))
            elif c_type == "disposal":
                g.add((cont_uri, RDF.type, STORE.DisposalContainer))

            drop_pose = list(_get_field(cont, "drop_pose_base", [0.0, 0.0, 0.0]))
            g.add((cont_uri, STORE.hasContainerType, Literal(c_type, datatype=XSD.string)))
            g.add((cont_uri, STORE.isContainerValid, Literal(bool(_get_field(cont, "valid", False)), datatype=XSD.boolean)))
            g.add((cont_uri, STORE.hasDropPose3D, Literal(json.dumps([float(v) for v in drop_pose]), datatype=XSD.string)))
            g.add((cont_uri, STORE.hasConfidence, Literal(float(_get_field(cont, "confidence", 0.0)), datatype=XSD.float)))

        self._run_owlrl(g)
        self.graph = g

    def query_sellable_by_slots(self, slots: dict[str, Any] | None = None) -> list[dict[str, Any]]:
        """스캔 후 Normal 상태이면서 뒤집히지 않은 판매 가능 상품 인스턴스를 조회한다."""
        filter_clause = _build_slot_filters(slots or {})
        query = f"""
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT ?inst ?trackId ?name ?yoloClass ?brand ?color ?taste ?category ?robotCategory
               ?sizeMm ?sweetnessRank ?expirationRaw ?pose3d ?depthLayer ?graspWidth ?remoteAuth
        WHERE {{
            ?inst rdf:type store:ProductInstance ;
                  store:hasTrackId ?trackId ;
                  store:hasStatus ?status ;
                  store:isBackside ?backside ;
                  store:hasExpirationRaw ?expirationRaw ;
                  store:hasPose3D ?pose3d ;
                  store:depthLayer ?depthLayer ;
                  store:hasGraspWidthMm ?instGraspWidth ;
                  store:instanceOf ?cat .
            ?cat store:hasName ?name ;
                 store:hasYoloClass ?yoloClass ;
                 store:hasBrand ?brand ;
                 store:hasColor ?color ;
                 store:hasTaste ?taste ;
                 store:hasCategory ?category ;
                 store:hasRobotCategory ?robotCategory ;
                 store:hasSizeMm ?sizeMm ;
                 store:hasSweetnessRank ?sweetnessRank ;
                 store:hasGraspWidthMm ?catGraspWidth ;
                 store:requiresRemoteAuth ?remoteAuth .
            FILTER(STR(?status) = "Normal")
            FILTER(?backside = false)
            BIND(IF(?instGraspWidth > 0, ?instGraspWidth, ?catGraspWidth) AS ?graspWidth)
            {filter_clause}
        }}
        ORDER BY ?expirationRaw ?depthLayer ?trackId
        """
        results: list[dict[str, Any]] = []
        for row in self.graph.query(query):
            results.append({
                "track_id": str(row.trackId),
                "name": str(row.name),
                "yolo_class": str(row.yoloClass),
                "brand": str(row.brand),
                "color": str(row.color),
                "taste": str(row.taste),
                "category": str(row.category),
                "robot_category": str(row.robotCategory),
                "size_mm": float(row.sizeMm),
                "sweetness_rank": int(row.sweetnessRank),
                "expiration_raw": str(row.expirationRaw),
                "pose_3d_base": json.loads(str(row.pose3d)),
                "depth_layer": int(row.depthLayer),
                "grasp_width_mm": float(row.graspWidth),
                "requires_remote_auth": bool(row.remoteAuth.toPython()),
            })
        return results

    def get_world_state_snapshot(
        self,
        display_tiers: dict[str, list[list[float]]] | None = None,
    ) -> dict[str, Any]:
        """판단 로직(Task Formalizer / Action Planner)에 넘길 World Model Ask 스냅샷을 반환한다."""
        query = """
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT ?inst ?trackId ?name ?yoloClass ?status ?isBackside ?expirationRaw ?disposalReason ?pose3d ?depthLayer
        WHERE {
            ?inst rdf:type store:ProductInstance ;
                  store:hasTrackId ?trackId ;
                  store:hasName ?name ;
                  store:hasYoloClass ?yoloClass ;
                  store:hasStatus ?status ;
                  store:isBackside ?isBackside ;
                  store:hasExpirationRaw ?expirationRaw ;
                  store:hasDisposalReason ?disposalReason ;
                  store:hasPose3D ?pose3d ;
                  store:depthLayer ?depthLayer .
        }
        ORDER BY ?trackId
        """
        observed_products: list[dict[str, Any]] = []
        for row in self.graph.query(query):
            inst_uri = row.inst
            yolo_cls = str(row.yoloClass)
            name = str(row.name)
            entailed = [
                cls_name for cls_name in _OWL_USER_CLASSES
                if (inst_uri, RDF.type, STORE[cls_name]) in self.graph
            ]
            tid_str = str(row.trackId)
            observed_products.append({
                "track_id": tid_str,
                "name": name,
                "yolo_class": yolo_cls,
                "robot_category": self.get_robot_category(yolo_cls, name),
                "robot_product_key": self.get_robot_product_key(yolo_cls, name, track_id=tid_str),
                "status": str(row.status),
                "is_backside": bool(row.isBackside.toPython()),
                "expiration_raw": str(row.expirationRaw),
                "disposal_reason": str(row.disposalReason),
                "pose_3d_base": json.loads(str(row.pose3d)),
                "depth_layer": int(row.depthLayer),
                "entailed_types": entailed,
            })

        return {
            "catalog": self.query_catalog_by_slots({"include_remote_auth": True}),
            "observed_products": observed_products,
            "containers": self.get_containers(),
            "display_tiers": dict(display_tiers or self._display_tiers),
            "owlrl": {
                "enabled": self.enable_owlrl,
                "triples": len(self.graph),
                "last_reasoning_ms": round(self.last_reasoning_ms, 2),
                "blocking_pairs": self._query_blocking_pairs(),
            },
        }

    def query_alternatives(
        self,
        failed_item: dict[str, Any],
        exclude_track_ids: set[str] | list[str] | None = None,
    ) -> list[dict[str, Any]]:
        """주문 상품 불량/품절 시 SPARQL 온톨로지 그래프(맛·브랜드·카테고리) 기반으로 Normal 대체 상품을 찾는다."""
        excluded_ids = set(exclude_track_ids or [])
        seen: set[str] = set()
        alternatives: list[dict[str, Any]] = []

        def _collect(cands: list[dict[str, Any]]) -> None:
            for cand in cands:
                tid = cand["track_id"]
                if tid in excluded_ids or tid in seen:
                    continue
                seen.add(tid)
                alternatives.append(cand)

        # 0) 동일 품목(같은 상품명)의 다른 정상 재고 최우선 탐색 (1종당 최대 3개 재고 구조 반영)
        if failed_item.get("name"):
            _collect(self.query_sellable_by_slots({
                "name": failed_item["name"],
                "exclude": failed_item.get("exclude", []),
            }))

        # 1) failed_item에 name/track_id가 있으면 OWL 개념 그래프(ofTaste / ofBrand) 피어 우선 탐색
        peer_names: list[str] = []
        if failed_item.get("name") or failed_item.get("track_id"):
            for rel_type in ("same_taste", "same_brand"):
                for peer in self.query_by_relation(
                    rel_type,
                    name=str(failed_item.get("name", "")),
                    track_id=str(failed_item.get("track_id", "")),
                ):
                    if peer["name"] not in peer_names:
                        peer_names.append(peer["name"])
            for pname in peer_names:
                _collect(self.query_sellable_by_slots({
                    "name": pname,
                    "exclude": failed_item.get("exclude", []),
                }))

        # 2) 속성 조합 기반 SPARQL 단계별 탐색
        search_tiers: list[dict[str, Any]] = []
        if failed_item.get("category") and failed_item.get("taste"):
            search_tiers.append({
                "category": failed_item["category"],
                "taste": failed_item["taste"],
                "exclude": failed_item.get("exclude", []),
            })
        if failed_item.get("category"):
            search_tiers.append({
                "category": failed_item["category"],
                "exclude": failed_item.get("exclude", []),
            })
        if failed_item.get("taste"):
            search_tiers.append({
                "taste": failed_item["taste"],
                "exclude": failed_item.get("exclude", []),
            })
        for tier in search_tiers:
            _collect(self.query_sellable_by_slots(tier))

        # 3) 동일 품목·동일 속성 재고가 전무할 때만 Snack 상위 계열(포스틱 ↔ 아몬드 빼빼로)로 폴백
        if not alternatives and (
            failed_item.get("category") in ("봉지과자", "종이상자", "과자", "스낵")
            or failed_item.get("name") in ("포스틱", "아몬드 빼빼로", "빼빼로")
        ):
            _collect(self.query_sellable_by_slots({
                "category": "과자",
                "exclude": failed_item.get("exclude", []),
            }))

        return alternatives

    def get_containers(self) -> dict[str, dict[str, Any]]:
        """현재 동기화된 컨테이너(checkout, disposal) 정보를 반환한다."""
        query = """
        PREFIX store: <http://store.cobot.org/ontology#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT ?type ?valid ?dropPose ?conf
        WHERE {
            ?c rdf:type store:Container ;
               store:hasContainerType ?type ;
               store:isContainerValid ?valid ;
               store:hasDropPose3D ?dropPose ;
               store:hasConfidence ?conf .
        }
        """
        containers: dict[str, dict[str, Any]] = {}
        for row in self.graph.query(query):
            c_type = str(row.type)
            containers[c_type] = {
                "container_type": c_type,
                "valid": bool(row.valid.toPython()),
                "drop_pose_base": json.loads(str(row.dropPose)),
                "confidence": float(row.conf),
            }
        return containers

    def record_disposal(self, track_id: str, reason: str) -> None:
        """폐기 처리된 상품의 사유를 그래프와 이력에 기록한다."""
        inst_uri = STORE[f"inst_{track_id}"]
        self.graph.set((inst_uri, STORE.hasDisposalReason, Literal(reason, datatype=XSD.string)))
        self._disposal_history.append({"track_id": track_id, "reason": reason})

    def to_networkx_json(self) -> dict[str, Any]:
        """RDFLib 그래프의 인스턴스·카탈로그·컨테이너 관계를 NetworkX JSON으로 변환한다."""
        nx_graph = nx.DiGraph()

        def _short(term: Any) -> str:
            text = str(term)
            if "#" in text:
                return text.rsplit("#", 1)[-1]
            return text

        for subj in self.graph.subjects(RDF.type, STORE.CatalogProduct):
            sid = _short(subj)
            label = str(self.graph.value(subj, STORE.hasName) or sid)
            nx_graph.add_node(sid, kind="catalog", label=label)

        for subj in self.graph.subjects(RDF.type, STORE.ProductInstance):
            sid = _short(subj)
            name = str(self.graph.value(subj, STORE.hasName) or sid)
            status = str(self.graph.value(subj, STORE.hasStatus) or "")
            entailed = [
                cls_name for cls_name in _OWL_USER_CLASSES
                if (subj, RDF.type, STORE[cls_name]) in self.graph
            ]
            nx_graph.add_node(
                sid,
                kind="instance",
                label=f"{name} ({status})",
                status=status,
                entailed_types=entailed,
            )
            cat_uri = self.graph.value(subj, STORE.instanceOf)
            if cat_uri is not None:
                nx_graph.add_edge(sid, _short(cat_uri), relation="instanceOf")

        for subj in self.graph.subjects(RDF.type, STORE.Container):
            sid = _short(subj)
            c_type = str(self.graph.value(subj, STORE.hasContainerType) or sid)
            valid = bool((self.graph.value(subj, STORE.isContainerValid) or Literal(False)).toPython())
            nx_graph.add_node(sid, kind="container", label=c_type, valid=valid)

        graph_data = nx.node_link_data(nx_graph, edges="links")
        graph_data["disposal_history"] = list(self._disposal_history)
        graph_data["stats"] = {
            "catalog_count": sum(1 for _, d in nx_graph.nodes(data=True) if d.get("kind") == "catalog"),
            "instance_count": sum(1 for _, d in nx_graph.nodes(data=True) if d.get("kind") == "instance"),
            "container_count": sum(1 for _, d in nx_graph.nodes(data=True) if d.get("kind") == "container"),
            "owlrl_triples": len(self.graph),
        }
        return graph_data
