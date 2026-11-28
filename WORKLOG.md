# WORKLOG

## [2026-11-26 20:29] `store_ws` ROS 2 Jazzy Overlay 워크스페이스 초기 구축

- **목적 / 배경**: [24시_무인편의점_최종구상안.md](file:///home/ms-04/Downloads/24시_무인편의점_최종구상안.md) 5.2절·5.3절·6.2절 명세에 따라 기존 `~/cobot2_ws`를 Underlay로 두고 독립된 Overlay 워크스페이스(`/home/ms-04/store_ws`) 초기 뼈대를 구축함.
- **담당 역할**:
  - **Codex**: `codex exec -s workspace-write` 위임 시도 중 `bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted` 샌드박스 오류로 중단됨.
  - **Antigravity**: 규칙에 따라 즉시 흐름을 이어받아 전체 디렉터리·설정·ROS 2 패키지·`core/` 순수 로직·단위 테스트 작성 및 `colcon build` / `pytest` 검증까지 직접 완료.
- **변경 파일 및 핵심 내용**:
  - 루트 및 공통 설정: `.env.example`, `.gitignore`, `requirements.txt`, `data/snapshots/.gitkeep`, `data/logs/.gitkeep`, `scripts/setup_env.bash`, `scripts/start_all.bash`
  - `config/`: `products_catalog.yaml`, `workspace_zones.yaml`, `control_params.yaml`, `cyclonedds.xml`, `ontology/store_ontology.ttl`, `calibration/.gitkeep`, `weights/.gitkeep`
  - `docker/`: `Dockerfile`, `docker-compose.yaml`, `vision_ws/store_yolo_docker/` (`yolo_docker_node.py`)
  - `src/store_interfaces/`: `DetectedProduct.msg`, `ContainerPose.msg`, `NodeHealth.msg`, `ScanWorkspace.srv`, `VerifyRemoteKey.srv`, `ExecuteRobotTask.action`
  - `src/store_vision/`: `core/` (`coordinate_math.py`, `depth_layer.py`, `vision_vlm.py`, `mock_camera.py`), `nodes/` (`realsense_node.py`, `webcam_node.py`, `perception_node.py`), `tests/test_vision_core.py`
  - `src/store_llm_agent/`: `core/` (`voice_stt.py`, `mode_fsm.py`, `kg_manager.py`, `llm_parser.py`, `rule_validator.py`, `inbound_planner.py`), `nodes/` (`voice_node.py`, `orchestrator.py`), `tests/test_llm_core.py`
  - `src/store_robot_control/`: `core/` (`grasp_planner.py`, `reorient_motion.py`, `safety_guard.py`, `blackbox_log.py`, `teleop_math.py`), `nodes/` (`executor_node.py`, `mock_robot.py`, `hand_client.py`, `remote_node.py`), `tests/test_robot_core.py`
  - `src/store_web_ui/`, `src/store_bringup/`: `web_ui_node.py`, `launch/store_bot.launch.py`
- **테스트 / 리뷰 결과**:
  - `source /opt/ros/jazzy/setup.bash && source /home/ms-04/cobot2_ws/install/setup.bash && colcon build --symlink-install`: 7개 패키지 전체 빌드 성공 (`13.7s`).
  - `PYTHONPATH=src/store_vision:src/store_llm_agent:src/store_robot_control pytest -v src/store_vision/tests src/store_llm_agent/tests src/store_robot_control/tests`: `11 passed in 0.07s`.
  - `codex exec review --uncommitted`: Codex 샌드박스(`bwrap`) 권한 오류로 미실행.
- **남은 TODO**:
  - 사용자 담당 파트(비전 A vs 비전 B) 확정 후 Day 1 실작업(데이터 촬영·캘리브레이션·실제 로직 구현) 착수.

## [2026-11-26 21:12] 모바일 현장 테스트용 YOLO 웹 추론 서버 및 터널링 구축

- **목적 / 배경**: 후보 과자 선정을 위해 코랩에서 학습한 `snack_best.pt`를 편의점 현장에서 스마트폰으로 바로 촬영·검증할 수 있도록 모바일 웹 테스트 환경 구성.
- **담당 역할**:
  - **Antigravity**: `~/yolo_venv` 분리 환경 구축(Host `numpy 1.26.4` 충돌 방지), FastAPI 모바일 추론 웹 서버 작성 및 `cloudflared` HTTPS 터널링 연동.
- **변경 파일 및 핵심 내용**:
  - [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py): 모바일 브라우저용 카메라 촬영/앨범 업로드, 클라이언트 이미지 리사이징(1280px), CUDA 추론 및 바운딩박스 이미지·신뢰도 반환 UI 구현.
  - `~/yolo_venv`: `torch 2.15.0` (CUDA), `ultralytics 8.4.219`, `fastapi`, `uvicorn` 설치.
- **테스트 / 리뷰 결과**:
  - CUDA 인식(`NVIDIA GeForce RTX 4060 Laptop GPU`) 및 `/home/ms-04/Downloads/snack_best.pt` 클래스(`Saeukkang`, `Pocachip_Original`, `Yangpaling`) 로드 확인.
  - `curl -s http://localhost:8000/health` 응답 정상(`device: cuda:0`).
- **남은 TODO**:
  - 편의점 현장 테스트 후 잘 인식되는 봉지과자 1종 확정 및 박스과자·음료 데이터셋 탐색.

## [2026-11-26 21:57] 모바일 YOLO 웹 테스터 멀티모델(봉지과자 + 빼빼로) 통합

- **목적 / 배경**: 로컬 RTX 4060에서 학습 완료된 아몬드 빼빼로 모델(`pepero_train/weights/best.pt`)을 기존 봉지과자 테스트 웹 서버에 통합해, 하나의 폰 접속 URL에서 두 모델 및 동시 추론을 테스트할 수 있도록 개선.
- **담당 역할**:
  - **Antigravity**: [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py) 멀티모델(`snack`, `pepero`, `all` 동시 추론) 지원 및 모델별 바운딩박스 색상 구분 UI 반영, 서버 재기동 및 터널 연동 검증.
- **변경 파일 및 핵심 내용**:
  - [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py): `MODEL_REGISTRY`에 `snack`([snack_best.pt](file:///home/ms-04/Downloads/snack_best.pt)) + `pepero`([best.pt](file:///home/ms-04/store_ws/runs/pepero_train/weights/best.pt)) 등록, 상단 탭(`둘 다 검사` / `봉지과자` / `빼빼로`) 추가.
- **테스트 / 리뷰 결과**:
  - 로컬(`/health`) 및 Cloudflare 터널 URL(`/health`) 모두 `snack`(3종)과 `pepero`(1종) 모델 로드 및 `cuda:0` 정상 응답 확인.
- **남은 TODO**:
  - 편의점에서 폰으로 봉지과자·빼빼로 인식률 테스트 후 음료(bottle) 데이터셋 추가 및 최종 3종 선정.

## [2026-11-27 11:26] 빼빼로 + 포스틱 통합 YOLOv8n 모델 학습 및 모바일 테스터 반영

- **목적 / 배경**: `snack.v6i.yolov8.zip`에서 `postick` 클래스만 추출하여 기존 `pepero_almonds` 데이터셋과 병합한 뒤 단일 YOLOv8n 모델로 학습하고, 폰 웹 테스터에 반영.
- **담당 역할**:
  - **Antigravity**: `postick` 필터링·클래스 재매핑(`0: pepero_almonds`, `1: postick`), RTX 4060 GPU 50 에폭 학습 실행, [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py) 기본 모델로 등록 및 터널 검증.
- **변경 파일 및 핵심 내용**:
  - `~/datasets/pepero_postick`: 병합 데이터셋 (`train`: 빼빼로 715 + 포스틱 150, `valid`: 빼빼로 200 + 포스틱 15, `test`: 빼빼로 77 + 포스틱 6).
  - [runs/pepero_postick_train/weights/best.pt](file:///home/ms-04/store_ws/runs/pepero_postick_train/weights/best.pt): 50 에폭 통합 가중치 파일(6.5MB).
  - [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py): `combined` (`pepero_almonds` + `postick`) 모델 추가 및 기본 선택 탭으로 설정.
- **테스트 / 리뷰 결과**:
  - 검증 성능(`best.pt`): 전체 `mAP50 0.982` / `mAP50-95 0.912` (`pepero_almonds` `mAP50 1.000`, `postick` `mAP50 0.963`).
  - `https://document-isbn-throwing-rob.trycloudflare.com/health` 정상 응답(`combined`, `snack`, `pepero` 3개 모델 로드 확인).
- **남은 TODO**:
  - 음료(bottle) 데이터셋 추가 후 최종 3종 통합 학습 및 `config/weights/` 배치.

## [2026-11-27 11:53] Docker GPU YOLO ↔ `perception_node` 3D 좌표 변환 E2E 흐름 검증

- **목적 / 배경**: 학습된 빼빼로+포스틱 통합 모델(`products_yolov8n.pt`)을 Docker GPU 컨테이너에서 실행하고, Host의 `realsense_node`(mock_mode) 및 `perception_node`와 CycloneDDS(`ROS_DOMAIN_ID=40`)로 연동해 3D 좌표 변환까지 전체 비전 파이프라인 검증.
- **담당 역할**:
  - **Antigravity**: 비전 전담 노드(`yolo_docker_node.py`, `realsense_node.py`, `perception_node.py`) 구현, `products_catalog.yaml` 상품 추가, Docker GPU 컨테이너 빌드·기동 및 E2E 테스트 수행.
- **변경 파일 및 핵심 내용**:
  - [config/weights/products_yolov8n.pt](file:///home/ms-04/store_ws/config/weights/products_yolov8n.pt): `pepero_postick_train/weights/best.pt` 복사.
  - [config/products_catalog.yaml](file:///home/ms-04/store_ws/config/products_catalog.yaml): `pepero_almonds`, `postick` 항목 추가(파지 폭·힘은 실측 전 임시값).
  - [docker/vision_ws/store_yolo_docker/store_yolo_docker/yolo_docker_node.py](file:///home/ms-04/store_ws/docker/vision_ws/store_yolo_docker/store_yolo_docker/yolo_docker_node.py): CUDA YOLOv8 추론 + OpenCV `Yaw` 계산 → `/store/yolo_detections` 발행 구현.
  - [src/store_vision/store_vision/nodes/realsense_node.py](file:///home/ms-04/store_ws/src/store_vision/store_vision/nodes/realsense_node.py): 테스트 이미지 순환 및 가상 Depth(`16UC1`) 발행 `mock_mode` + D435i 실물 모드 구현.
  - [src/store_vision/store_vision/nodes/perception_node.py](file:///home/ms-04/store_ws/src/store_vision/store_vision/nodes/perception_node.py): `/store/yolo_detections` + `/camera/aligned_depth` 기반 `ScanWorkspace.srv` 3D 좌표 변환 구현(`T_gripper2camera.npy` 미존재 시 단위행렬 fallback).
- **테스트 / 리뷰 결과**:
  - `colcon build --symlink-install --packages-select store_vision` 및 `pytest -v src/store_vision/tests`: `4 passed in 0.05s`.
  - Docker `yolo-detection` 컨테이너 `cuda:0` 로드 확인 (`pepero_almonds`, `postick`).
  - `ros2 topic echo /store/yolo_detections --once`: Docker → Host JSON 검출 결과 수신 확인.
  - `ros2 service call /scan_workspace`: `success=True, error_code='OK'`, `DetectedProduct` (`pose_3d_base`, `grasp_width_mm`, `Z_MIN=15.0` 클램프 포함) 정상 응답 확인.
- **남은 TODO**:
  - 실물 D435i 연결 시 Eye-in-Hand 캘리브레이션(`T_gripper2camera.npy`) 생성 및 실물 상품 3종 파지 폭/힘 실측 반영.

## [2026-11-27 12:19] 팀 협의 규격(`_front`, 토픽/서비스명, `Z_surface`, `rz` 정규화) 반영 및 E2E 재검증

- **목적 / 배경**: LLM·로봇 파트와 확정한 인터페이스 규격(`pepero_almonds_front`/`postick_front`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`, `/store/scan_workspace`, 순수 `Z_surface` 반환, 단축 기준 `rz` `-90°~+90°`)을 비전 노드에 반영.
- **담당 역할**:
  - **Antigravity**: [config/products_catalog.yaml](file:///home/ms-04/store_ws/config/products_catalog.yaml) 더미 제거 및 `<영문명>_front` 규칙 적용, `realsense_node`·`yolo_docker_node`·`perception_node` 규격 정렬, Docker 재빌드 및 E2E 재검증.
- **변경 파일 및 핵심 내용**:
  - [config/products_catalog.yaml](file:///home/ms-04/store_ws/config/products_catalog.yaml): `item_a`~`item_f` 제거, `pepero_almonds_front` / `postick_front` 적용.
  - [realsense_node.py](file:///home/ms-04/store_ws/src/store_vision/store_vision/nodes/realsense_node.py): `/camera/aligned_depth_to_color/image_raw` + `/camera/color/camera_info` 발행.
  - [yolo_docker_node.py](file:///home/ms-04/store_ws/docker/vision_ws/store_yolo_docker/store_yolo_docker/yolo_docker_node.py): 단축 감싸는 각도 `[-90, 90]` 정규화.
  - [perception_node.py](file:///home/ms-04/store_ws/src/store_vision/store_vision/nodes/perception_node.py): `/store/scan_workspace` 서비스명, `camera_info` 동적 사용, 순수 `Z_surface` 반환, `_front` 카탈로그 호환 매칭.
- **테스트 / 리뷰 결과**:
  - `pytest -v src/store_vision/tests`: `4 passed in 0.05s`.
  - Docker `yolo-detection` 재빌드·기동 후 `/store/scan_workspace` 호출: `yolo_class='pepero_almonds_front'`, `pose_3d_base=[417.32, -3.17, 80.0, -71.79]`(`Z_surface=80.0`, `rz=-71.79`), `/camera/color/camera_info` 정상 수신 확인.
- **남은 TODO**:
  - 음료(`bottle`) + `product_back` 포함 최종 3종(또는 6종) 데이터셋을 `<영문명>_front` / `product_back` 클래스명으로 학습 및 실물 D435i `T_gripper2camera.npy` 캘리브레이션 진행.

## [2026-11-27 12:25] 재사용 가능한 YOLO 학습 스크립트(`scripts/train_yolo.py`) 생성

- **목적 / 배경**: 향후 음료(`bottle`) 및 `product_back` 클래스 추가 학습을 손쉽게 재실행하고 `config/weights/products_yolov8n.pt`로 바로 배포할 수 있도록 Python 학습 스크립트 생성.
- **담당 역할**:
  - **Antigravity**: [scripts/train_yolo.py](file:///home/ms-04/store_ws/scripts/train_yolo.py) 작성 및 `--help` 실행 검증.
- **변경 파일 및 핵심 내용**:
  - [scripts/train_yolo.py](file:///home/ms-04/store_ws/scripts/train_yolo.py): `--data`, `--epochs`, `--name`, `--deploy` 옵션 지원.
- **테스트 / 리뷰 결과**:
  - `~/yolo_venv/bin/python /home/ms-04/store_ws/scripts/train_yolo.py --help` 정상 출력 확인.
- **남은 TODO**:
  - 없음.

## [2026-11-27 19:13] 5개 상품 데이터셋 통합 병합 및 YOLOv8n 학습

- **목적 / 배경**: Roboflow에서 다운로드한 5개 데이터셋(`pepero_almond_front.v2i.yolov8`, `postick_front.v1i.yolov8`, `pocachip_onion_front.v1i.yolov8`, `product_back.v1i.yolov8`, `aloe_drink_front.v1i.yolov8`)에서 감자깡(`Gamjakkang`) 및 잡클래스를 제외하고, 클래스명을 5개(`pepero_almonds_front`, `postick_front`, `pocachip_onion_front`, `product_back`, `aloe`)로 통일하여 단일 YOLOv8n 모델로 학습.
- **담당 역할**:
  - **Antigravity**: [scripts/merge_datasets.py](file:///home/ms-04/store_ws/scripts/merge_datasets.py) 작성·실행, `/home/ms-04/datasets/store_5classes` 병합 생성 및 [scripts/train_yolo.py](file:///home/ms-04/store_ws/scripts/train_yolo.py)로 YOLOv8n 학습 실행.
- **변경 파일 및 핵심 내용**:
  - [scripts/merge_datasets.py](file:///home/ms-04/store_ws/scripts/merge_datasets.py): 5개 데이터셋의 라벨을 정규 5개 클래스 ID(`0~4`)로 리매핑하고, 불필요한 클래스 및 빈 라벨 이미지 제거, `pepero_almonds_front` 수량 밸런싱(Train 350 / Valid 60 / Test 30) 적용.
  - `/home/ms-04/datasets/store_5classes`: `train` 1,304장, `valid` 162장, `test` 81장 병합 완료.
  - `/home/ms-04/store_ws/runs/store_5classes_train/weights/best.pt`: 50 epochs 학습 가중치 생성.
- **테스트 / 리뷰 결과**:
  - YOLOv8n Validation (`best.pt`, 162 images / 227 instances): `Precision=0.972`, `Recall=0.966`, `mAP50=0.974`, `mAP50-95=0.843` (`pepero_almonds_front` mAP50=0.980, `postick_front` mAP50=0.977, `pocachip_onion_front` mAP50=0.911, `product_back` mAP50=1.000, `aloe` mAP50=1.000).
  - `codex exec review --uncommitted`: Codex Linux bubblewrap 샌드박스 환경 오류로 실행 불가(미검증).
- **남은 TODO**:
  - `config/products_catalog.yaml`에 `pocachip_onion_front`, `aloe` 항목 추가 및 필요 시 `runs/store_5classes_train/weights/best.pt`를 `config/weights/products_yolov8n.pt`로 배포.

## [2026-11-27 19:21] 모바일 실시간 YOLO 웹 테스터 업그레이드 (`scripts/mobile_yolo_test.py`)

- **목적 / 배경**: 방금 학습한 5종 통합 모델(`store_5classes`)을 휴대폰에서 사진 찍기뿐만 아니라 Ultralytics YOLO 앱처럼 **실시간 카메라 스트림**으로 바로 테스트할 수 있도록 개선.
- **담당 역할**:
  - **Antigravity**: [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py)에 5종 모델 등록, WebSocket(`/ws/predict`) 실시간 추론 + `<canvas>` 오버레이, 자체 서명 HTTPS 인증서 자동 생성(`.certs/`) 적용 후 데몬 재기동.
- **변경 파일 및 핵심 내용**:
  - [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py): `store_5classes` 기본 모델 추가, `getUserMedia` + WebSocket 기반 실시간 바운딩박스 오버레이 및 FPS/지연시간 표시, 기존 `/predict` 사진 모드 유지.
- **테스트 / 리뷰 결과**:
  - `https://0.0.0.0:8000` 기동 및 `curl -sk https://127.0.0.1:8000/health` 정상 응답 확인.
- **남은 TODO**:
  - 없음.

## [2026-11-27 19:41] 클래스명 변경 (`aloe` -> `bottle`)

- **목적 / 배경**: 사용자 요청에 따라 5번 클래스(4번 인덱스) 이름을 `aloe`에서 `bottle`로 통일.
- **담당 역할**:
  - **Antigravity**: [scripts/merge_datasets.py](file:///home/ms-04/store_ws/scripts/merge_datasets.py), `/home/ms-04/datasets/store_5classes/data.yaml`, `runs/store_5classes_train/weights/{best,last}.pt`의 `names[4]` 및 [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py) 색상 매핑을 `bottle`로 업데이트 후 서버 재기동.
- **변경 파일 및 핵심 내용**:
  - `best.pt`, `last.pt`: `names`가 `{0: 'pepero_almonds_front', 1: 'postick_front', 2: 'pocachip_onion_front', 3: 'product_back', 4: 'bottle'}`로 갱신됨.
- **테스트 / 리뷰 결과**:
  - `best.pt`/`last.pt`/`data.yaml` 클래스명 `bottle` 반영 및 모바일 서버 재기동 확인.
- **남은 TODO**:
  - 없음.

## [2026-11-27 19:54] 모바일 테스터 단일 모델 정리 및 클래스명 변경 (`bottle` -> `drink_front`)

- **목적 / 배경**: 모바일 테스터에서 구버전 모델을 제거해 `store_5classes`만 표시하고, 4번 인덱스 클래스명을 팀 규격에 맞게 `drink_front`로 변경.
- **담당 역할**:
  - **Antigravity**: [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py)에서 구버전 모델 및 선택 탭 제거, [scripts/merge_datasets.py](file:///home/ms-04/store_ws/scripts/merge_datasets.py), `data.yaml`, `best.pt`/`last.pt`의 `names[4]`를 `drink_front`로 변경 후 서버 재기동.
- **변경 파일 및 핵심 내용**:
  - `best.pt`, `last.pt`, `data.yaml`: `4: 'drink_front'` 반영.
- **테스트 / 리뷰 결과**:
  - 모바일 서버 기동 및 `/health` 확인 예정.
- **남은 TODO**:
  - 없음.


## [2026-11-27 20:54] 신규 제품 클래스명(`postick_front`, `pepero_almond_front`, `drink_front`, `product_back`, `vest`) 코드·설정 반영

- **목적 / 배경**: 최종 사용 클래스명(`postick_front`, `pepero_almond_front`, `drink_front`, `product_back`, `vest`, 후순위 `ciga`)에 맞춰 카탈로그·데이터셋 병합·학습·모바일 테스터·테스트 이미지 경로를 통일.
- **담당 역할**:
  - **Antigravity**: 카탈로그/스크립트/비전 노드 일괄 수정 및 문법·YAML·단위 테스트 검증, `WORKLOG.md` 기록.
  - **Codex**: `codex exec review --uncommitted` 실행 시 Linux `bubblewrap` 샌드박스 권한 오류로 Git diff 조회 불가(미검증).
- **변경 파일 및 핵심 내용**:
  - [config/products_catalog.yaml](file:///home/ms-04/store_ws/config/products_catalog.yaml): `pepero_almonds_front` → `pepero_almond_front` 수정, `drink_front` 항목 추가(후순위 `ciga` 주석 메모).
  - [scripts/merge_datasets.py](file:///home/ms-04/store_ws/scripts/merge_datasets.py): `TARGET_CLASSES` 및 `CLASS_MAP`을 신규 5종(`postick_front`, `pepero_almond_front`, `drink_front`, `product_back`, `vest`)으로 재정의하고 `pocachip_onion_front` 제거, `vest` 소스는 미존재 시 건너뛰도록 `optional: True` 처리.
  - [scripts/train_yolo.py](file:///home/ms-04/store_ws/scripts/train_yolo.py): `DEFAULT_DATA`를 `/home/ms-04/datasets/store_5classes/data.yaml`로 변경.
  - [scripts/mobile_yolo_test.py](file:///home/ms-04/store_ws/scripts/mobile_yolo_test.py): 모델 라벨 및 `CLASS_COLORS`를 신규 5종 + `ciga` 기준으로 갱신.
  - [src/store_vision/store_vision/nodes/realsense_node.py](file:///home/ms-04/store_ws/src/store_vision/store_vision/nodes/realsense_node.py): `mock_mode` 기본 테스트 이미지 경로를 `~/datasets/store_5classes/test/images`(`pepero_*.jpg`, `postick_*.jpg`, `aloe_*.jpg`)로 갱신.
- **테스트 / 리뷰 결과**:
  - `python3 -m py_compile` (4개 Python 파일) 및 `yaml.safe_load` (`products_catalog.yaml`: `['pepero_almond', 'postick', 'drink']`) 정상 통과.
  - `PYTHONPATH=src/store_vision pytest -v src/store_vision/tests`: `4 passed in 0.05s`.
  - `codex exec review --uncommitted`: Codex Linux `bubblewrap` 샌드박스 오류로 미검증.
- **남은 TODO**:
  - `vest` 데이터셋(`/home/ms-04/datasets/vest.v1i.yolov8`) 확보 후 `merge_datasets.py` → `train_yolo.py --deploy` 재실행 및 필요 시 `yolo_docker_node.py`의 `vest` 감지 → `/store/vision/vest_detected` 발행 연동.


## [2026-11-28 11:32] 실물 상품 카탈로그(`products_catalog.yaml`) 외형 치수 및 파지 폭 반영

- **목적 / 배경**: 실측 제품 외형 치수(빼빼로 `88x162`, 포스틱 `180x235`, 병음료 `63x208`)를 반영하고, `grasp_width_mm` 및 `drink` 항목의 `display_name`을 실제 사양에 맞게 업데이트.
- **담당 역할**:
  - **Antigravity**: [config/products_catalog.yaml](file:///home/ms-04/store_ws/config/products_catalog.yaml) 수정, YAML 파싱 및 `store_vision` 단위 테스트 검증, `WORKLOG.md` 기록.
  - **Codex**: `codex exec review --uncommitted` 실행 시 Linux `bubblewrap` 샌드박스 시작 실패로 변경분 미검증.
- **변경 파일 및 핵심 내용**:
  - [config/products_catalog.yaml](file:///home/ms-04/store_ws/config/products_catalog.yaml):
    - `pepero_almond`: `width_mm: 88.0`, `height_mm: 162.0` 추가, `grasp_width_mm` `70.0` → `88.0` 변경.
    - `postick`: `width_mm: 180.0`, `height_mm: 235.0` 추가, `grasp_width_mm: 65.0` 유지(그리퍼 최대 폭 `110.0mm` 초과 방지).
    - `drink`: `display_name` `"음료"` → `"병음료"` 변경, `width_mm: 63.0`, `height_mm: 208.0` 추가, `grasp_width_mm` `65.0` → `63.0` 변경.
- **테스트 / 리뷰 결과**:
  - `yaml.safe_load` 검증 통과: `{'pepero_almond': ('아몬드 빼빼로', 88.0, 162.0, 88.0), 'postick': ('포스틱', 180.0, 235.0, 65.0), 'drink': ('병음료', 63.0, 208.0, 63.0)}`.
  - `PYTHONPATH=src/store_vision pytest -v src/store_vision/tests`: `4 passed in 0.07s`.
  - `codex exec review --uncommitted`: `bubblewrap` 샌드박스 시작 오류로 미검증.
- **남은 TODO**:
  - 실물 로봇 파지 테스트 시 포스틱 두께 방향 `grasp_width_mm` 및 각 품목의 `grasp_force_n` / `grasp_depth_offset_mm` 미세 조정.
