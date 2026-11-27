# 24h Unmanned Store Cobot (`store_ws`)

24시 무인편의점 물류·매장 관리 협동로봇 프로젝트 (Doosan M0609 + OnRobot RG2 + RealSense D435i, ROS 2 Jazzy)

## Workspace Structure
- `config/`: 중앙 YAML 설정 (`workspace_zones.yaml`, `products_catalog.yaml`, `control_params.yaml`) 및 확정 Eye-in-Hand 캘리브레이션 (`calibration/T_gripper2camera.npy`)
- `src/store_interfaces/`: 공용 ROS 2 인터페이스 계약 (`.msg` 3종, `.srv` 2종, `ExecuteRobotTask.action`)
- `src/store_robot_control/`: 로봇 제어 패키지
  - `core/`: ROS 2 비의존 순수 Python 안전·파지·180° 뒤집기·블랙박스 모듈 (`pytest` 단독 실행)
  - `hardware/`: Doosan M0609 (`dsr_interface.py`) · OnRobot RG2 (`rg2_gripper.py`) 어댑터
  - `nodes/`: `mock_robot`, `executor_node`, `teach_zones`
- `src/store_bringup/`: 통합 런치 (`mock_robot.launch.py`, `executor.launch.py`, `store_bot.launch.py`)

## Build & Test (Underlay: `ws_dsr` + Overlay: `store_ws`)
```bash
cd ~/store_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
colcon build --symlink-install
source ~/store_ws/install/local_setup.bash

# 순수 Python 단위 테스트 (56건)
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

## Team Interface Conventions
- **좌표 단위**: 모든 3D/4D 좌표는 **`mm` + `deg`** (Robot Base Frame)
- **버퍼 키**: `config/workspace_zones.yaml` → `zones.buffer_slots`
- **`ExecuteRobotTask.product_category`**:
  - 상품 카테고리 3종: `paper_box` (종이상자), `snack_bag` (봉지과자), `candy_chocolate` (캔디·초콜릿류)
  - 특수 작업용: `cigarette_mock` (폐기 상품 투입은 `pick_place`로 폐기통 좌표 전달)
  - `grasp_width_mm` / `grasp_force_n`은 `0.0` 전달 시 로봇 파트가 자동 계산
- **토픽 / 서비스 / 액션**:
  - Action: `/store/execute_robot_task` (`store_interfaces/action/ExecuteRobotTask`)
  - Topics: `/store/emergency_stop` (`std_msgs/Bool`), `/store/mode/remote_override` (`std_msgs/Bool`), `/store/node_health` (`store_interfaces/msg/NodeHealth`)
  - Service: `/store/verify_remote_key` (`store_interfaces/srv/VerifyRemoteKey`)
