# 24h Unmanned Store Cobot (`store_ws`)

24시 무인편의점 물류·매장 관리 협동로봇 프로젝트 (Doosan M0609 + OnRobot RG2 + RealSense D435i, ROS 2 Jazzy)

## Workspace Structure
- `config/`: 중앙 YAML 설정 (`workspace_zones.yaml`, `products_catalog.yaml`, `control_params.yaml`), OWL 2 RL 온톨로지 (`ontology/store_ontology.ttl`), Eye-in-Hand 캘리브레이션 (`calibration/T_gripper2camera.npy`)
- `src/store_interfaces/`: 공용 ROS 2 인터페이스 계약 (`.msg` 3종, `.srv` 2종, `ExecuteRobotTask.action`)
- `src/store_llm_agent/`: 음성(STT) · OWL 2 RL 온톨로지 · `STOREPLAN` 판단 에이전트 패키지
  - `core/`: ROS 2 비의존 순수 Python 파서·온톨로지 추론·Task Formalizer·Symbolic BFS 플래너 (`pytest` 단독 실행)
  - `nodes/`: `voice_node`, `orchestrator`
- `src/store_robot_control/`: 로봇 제어 패키지
  - `core/`: ROS 2 비의존 순수 Python 안전·파지·재정렬·블랙박스 모듈 (`pytest` 단독 실행)
  - `hardware/`: Doosan M0609 (`dsr_interface.py`) · OnRobot RG2 (`rg2_gripper.py`) 어댑터
  - `nodes/`: `mock_robot`, `executor_node`, `remote_node`, `teach_zones`, `robot_demo`
- `src/store_bringup/`: 통합 런치 (`mock_robot.launch.py`, `executor.launch.py`, `store_bot.launch.py`)

## Build & Test (Underlay: `ws_dsr` + Overlay: `store_ws`)
```bash
cd ~/store_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
colcon build --symlink-install
source ~/store_ws/install/local_setup.bash

# 순수 Python 계획·안전 테스트와 ROS 2 드라이런·원격 노드 단위 테스트
pytest -v src/store_robot_control/tests
```

## Quick Run
```bash
# 1) 가상 로봇 Action 서버 (LLM / UI 비점유 연동 테스트용)
ros2 launch store_bringup mock_robot.launch.py

# 2) 실물 로봇 실행기 (안전 기본값 dry_run:=true)
ros2 launch store_bringup executor.launch.py dry_run:=true

# 3) 실물 로봇 저속 실행 (자동 모드 robot_mode=1 전환 후)
ros2 launch store_bringup executor.launch.py dry_run:=false velocity:=20.0 acceleration:=20.0

# 4) 직접 교시 좌표 기록 (수동 모드 robot_mode=0에서 실행)
ros2 run store_robot_control teach_zones

# 5) 원격 조작 노드 (안전 기본값 dry_run:=true) 및 입력 클라이언트
REMOTE_ADMIN_KEY=my-secret ros2 launch store_bringup remote.launch.py dry_run:=true
ros2 run store_robot_control hand_client --mode keyboard
```

## 통합 런치와 시나리오 검증

```bash
# 모의 로봇과 전체 시나리오. 각각 별도 터미널에서 실행
ros2 launch store_bringup store_bot.launch.py
ros2 run store_robot_control robot_demo --suite all --log-dir data/logs

# 실행기 드라이런에서 비상 정지와 home 복구 확인
ros2 launch store_bringup store_bot.launch.py use_mock_robot:=false dry_run:=true step_delay_sec:=0.15
ros2 run store_robot_control robot_demo --suite estop

# 실물 저속 실행기와 원격 노드
REMOTE_ADMIN_KEY=my-secret ros2 launch store_bringup store_bot.launch.py \
  use_mock_robot:=false dry_run:=false enable_remote:=true velocity:=20.0 acceleration:=20.0 \
  check_grip_detection:=true apply_grasp_depth_offset:=true
```

- 실행기가 지원하는 작업은 `home`, `scan_pose`, `pick_place`, `move_to_buffer`, `restore_from_buffer`, `reorient`입니다.
- 소매 상품의 기본 재정렬 방식인 `gentle_flip`은 상품을 들어 올린 뒤 J5만 기본 40° 기울여 놓고, 원래 관절 자세를 복원한 다음 수직으로 후퇴합니다. 기울기와 상승 여유는 `control_params.yaml`의 `reorient`에서 설정합니다. J5 이동 속도와 가속도는 일반 동작 설정과 저속 설정의 작은 값을 사용합니다.
- `robot_demo`는 `safety`, `motion`, `estop`을 개별 실행하거나 `all`에서 이 순서로 실행합니다. 비상 정지 검증은 `CANCELED`와 상태 `ERROR`를 확인한 뒤 정지를 해제하고 `home`으로 복귀합니다. `--log-dir`을 지정하면 해당 실행의 블랙박스 기록을 표시합니다.
- 실물 시험 시 `--target-pose X Y Z RZ`와 `--dest-pose X Y Z RZ`로 교시한 테이블 좌표를 전달할 수 있습니다. 테이블 A에 종이상자 하나를 놓고 테이블 B와 첫 번째 버퍼를 비워 둡니다. 데모는 스캔 후 홈으로 돌아와 파지 시험을 시작합니다. 실물 데모는 A에서 B로 옮긴 시험 물품을 다음 버퍼 시험 전에 A로 돌려놓습니다. `--yes`는 실물 실행 확인을 생략합니다.
- `--config-dir`으로 런치와 같은 설정 디렉터리를 지정할 수 있으며 `--timeout-sec`의 기본값은 60초입니다. 워크스페이스 환경을 불러온 뒤 `./scripts/run_robot_demo.py`로도 실행할 수 있습니다.

## Team Interface Conventions
- **좌표 단위**: 모든 3D/4D 좌표는 **`mm` + `deg`** (Robot Base Frame)
- **버퍼 키**: `config/workspace_zones.yaml` → `zones.buffer_slots`
- **`ExecuteRobotTask.product_category`**:
  - 확정 매장 상품 3종(각 3개씩 총 9개 실물): `snack_bag` (봉지과자: 포스틱×3), `paper_box` (종이상자: 아몬드 빼빼로×3), `bottle` (병음료×3) *(레거시 `candy_chocolate` 호환 유지)*
  - 특수 작업용: `ciga` / `cigarette_mock` (모형 담배 — 원격 제어 전용, 일반 폐기 투입은 `pick_place`로 폐기통 좌표 전달)
  - `grasp_width_mm` / `grasp_force_n`은 `0.0` 전달 시 로봇 파트가 자동 계산
- **토픽 / 서비스 / 액션**:
  - Action: `/store/execute_robot_task` (`store_interfaces/action/ExecuteRobotTask`)
  - Topics: `/store/emergency_stop` (`std_msgs/Bool`), `/store/mode/remote_override` (`std_msgs/Bool`), `/store/node_health` (`store_interfaces/msg/NodeHealth`)
  - Service: `/store/verify_remote_key` (`store_interfaces/srv/VerifyRemoteKey`)
