# Project Worklog

## 2026-11-26 20:09 — `store_ws` 초기 뼈대, `store_interfaces` 동결 및 로봇 파트(`store_robot_control` / `store_bringup`) 1차 구축

### Goal / Background
- 24시 무인편의점 협동로봇 프로젝트에서 로봇·원격 파트(김용수) 착수 작업으로, 팀 전체의 ROS 2 인터페이스 계약(`store_interfaces`)을 동결하고, LLM(`orchestrator`)·UI 팀원이 실물 로봇 점유 없이 즉시 연동 테스트할 수 있는 가상 로봇 Action 서버(`mock_robot`)와 순수 Python 로봇 핵심 안전/파지/뒤집기/블랙박스 모듈을 구축함.
- 최종 합의된 **`orchestrator` vs `executor_node` 연결 경계**를 설계서(`~/24_CON/24시_무인편의점_최종구상안.md`)와 코드에 일관되게 반영함 (1스텝씩 Action 호출, `product_category` 기반 로봇 측 RG2 파지 폭/힘 자동 계산, `scan_pose`/`home` 고정 좌표 로봇 측 소유, `REMOTE` Cancel 및 E-Stop 물리 안전 정지).

### Responsible Agent(s)
- **Antigravity**:
  - `~/24_CON/24시_무인편의점_최종구상안.md`에 로봇 담당 명시 및 6.3절(`orchestrator` vs `executor_node` 연결 경계) 반영
  - `~/store_ws/` Git 저장소 초기화, 확정본 `T_gripper2camera.npy` 백업 복사, `.gitignore`, `.env.example`, `requirements.txt`, `scripts/setup_env.bash`, 중앙 `config/*.yaml` 3종 작성
  - `src/store_interfaces/`(`ament_cmake`) 전체 인터페이스(`.msg` 3종, `.srv` 2종, `.action` 1종) 작성
  - `colcon build --symlink-install` 빌드 검증, 실제 다중 프로세스 ROS 2 E2E 스모크 테스트 실행, 코드 규격 보정(`error_code="OK"`, `retreat` 단계 추가, Codex 리뷰 4건 반영)
- **Codex (`codex exec -s workspace-write` & `codex exec review --uncommitted`)**:
  - `src/store_robot_control/` (`core/` 순수 Python 4모듈 + `nodes/mock_robot.py` + `tests/` 4개 테스트 파일) 및 `src/store_bringup/` (`mock_robot.launch.py`, `store_bot.launch.py`) 1차 구현
  - 미커밋 변경분 크로스 리뷰 수행 (P2 `_busy` 슬롯 영구 점유 방지, `blackbox.enabled: false` 미적용, `config_dir` 탐색 보강, P3 `plan_reorient()` 비유한 `NaN`/`-inf` 선검사 지적)

### Detailed Changes
- **Workspace & Configs**:
  - `/home/yongsoo/store_ws/.gitignore`, `.env.example`, `requirements.txt`, `scripts/setup_env.bash`
  - `/home/yongsoo/store_ws/config/calibration/T_gripper2camera.npy` (확정 캘리브레이션 행렬 복사: `t=[32.8, 78.0, -216.3] mm`)
  - `/home/yongsoo/store_ws/config/products_catalog.yaml`: 카테고리별 RG2 파지 폭/힘/여유폭/파지 깊이 오프셋 및 RG2 물리 한계(`0–110 mm`, `3–40 N`) 정의
  - `/home/yongsoo/store_ws/config/workspace_zones.yaml`: `Tool_Weight` / `GripperDA_v1`, `z_min_mm=15.0`, `safe_transit_z_mm=260.0`, `j6_min/max=±150.0°`, 작업 반경 및 `home`/`scan_pose`/`employee_handover_pose` 기준 좌표 정의
  - `/home/yongsoo/store_ws/config/control_params.yaml`: 속도/가속도, RG2 Modbus IP/Port, Heartbeat 주기, Mock 단계 지연, Blackbox 및 후순위 원격 파라미터 정의
- **ROS 2 Interfaces (`/home/yongsoo/store_ws/src/store_interfaces/`)**:
  - `CMakeLists.txt`, `package.xml`
  - `msg/DetectedProduct.msg`, `msg/ContainerPose.msg`, `msg/NodeHealth.msg`
  - `srv/ScanWorkspace.srv`, `srv/VerifyRemoteKey.srv`
  - `action/ExecuteRobotTask.action` (`grasp_width_mm`/`grasp_force_n`은 `0.0`일 때 로봇 `grasp_planner` 자동 계산)
- **Robot Control (`/home/yongsoo/store_ws/src/store_robot_control/`)**:
  - `store_robot_control/core/safety_guard.py`: 순수 Python 좌표 유효성·`Z_MIN_VIOLATION`·`OUT_OF_REACH`·`J6(±150°)`·안전 이동 높이·스레드 안전 E-Stop 관리
  - `store_robot_control/core/grasp_planner.py`: `product_category` 기반 RG2 파지 폭/힘·레지스터(`*10`) 계산, 알 수 없는 카테고리 `defaults` 폴백, 명시적 override 및 한계 클램핑
  - `store_robot_control/core/reorient_motion.py`: 180° 뒤집기 8단계 웨이포인트 생성 (`J6 ±150°` 내 회전 방향 자동 선택, 비유한 좌표 선검사)
  - `store_robot_control/core/blackbox_log.py`: JSONL 실행 로그 기록기 (`robot_blackbox.jsonl`)
  - `store_robot_control/nodes/mock_robot.py`: `/store/execute_robot_task` ActionServer (7개 `action_type`, `handle_accepted_callback` 기반 `_busy` 관리, `/store/emergency_stop` 구독, `/store/verify_remote_key` 원격 키 검증, `/store/node_health` Heartbeat 발행, Blackbox 기록)
  - `tests/test_safety_guard.py`, `test_grasp_planner.py`, `test_reorient_motion.py`, `test_blackbox_log.py`
- **Bringup (`/home/yongsoo/store_ws/src/store_bringup/`)**:
  - `launch/mock_robot.launch.py`, `launch/store_bot.launch.py` (`use_mock_robot:=true` 토글 지원)

### Verification & Review
- **Build**: `colcon build --symlink-install` (underlay: `/opt/ros/jazzy` + `~/ws_cobot_pjt/ws_dsr`) → `store_interfaces`, `store_robot_control`, `store_bringup` 3개 패키지 모두 오류 없이 빌드 완료.
- **Unit Tests**: `pytest -v src/store_robot_control/tests` → **28 passed** (기존 26건 + `plan_reorient` 비유한 입력 검증 2건 추가). `core/` 모듈의 ROS 2 비의존성 확인 완료.
- **ROS 2 Multi-Process Smoke Tests**: 실제 `mock_robot` 노드 실행 후 ActionClient로 `scan_pose`, `pick_place`(`product_category="snack_box"`), `reorient`, `Z_MIN_VIOLATION` 차단, 중간 `Cancel`(`REMOTE` 시뮬레이션), `/store/emergency_stop` 즉시 중단, `/store/node_health` `OK→ERROR` 전환, `robot_blackbox.jsonl` 기록 모두 통과.
- **Codex Review Resolution**:
  - `mock_robot.py`: `_busy` 획득을 `handle_accepted_callback`으로 이동(예외 시 즉시 해제), `blackbox.enabled: false` 시 로거 미생성, `find_config_dir()`에 `~/store_ws/config` 폴백 추가.
  - `reorient_motion.py`: `max(z_min_mm, target_z)` 보정 전 비유한(`NaN`/`±inf`) 좌표 검사 선행 및 단위 테스트 추가.
  - `safety_guard.py`: `max_radial_reach_mm` 조회 시 `safety` 및 `workspace_bounds_mm` 양쪽 모두 방어적으로 지원.

### Follow-ups / Caveats
- `config/workspace_zones.yaml`의 `scan_pose`, `home`, `employee_handover_pose`, `z_min_mm`, `zones` 좌표와 `config/products_catalog.yaml`의 품목별 파지 폭/힘은 실물 테이블 및 확정 상품 6–9종 기준으로 실측 업데이트 필요.
- 다음 단계로 실제 Doosan M0609(`DSR_ROBOT2`) + OnRobot RG2(`onrobot.py`)를 제어하는 `store_robot_control/nodes/executor_node.py` (`dry_run` 모드 포함) 구현 예정.
- 아직 첫 Git 커밋 전 상태이므로 팀 원격 저장소 연결 시 `git add` & 초기 커밋 권장.

## 2026-11-26 20:25 — 실물 로봇 실행기(`executor_node`), Doosan/RG2 드라이버 및 직접 교시 좌표 기록 도구(`teach_zones`) 구현

### Goal / Background
- `mock_robot` 검증 완료 후 실제 Doosan M0609 + OnRobot RG2를 `ExecuteRobotTask.action` 인터페이스로 안전하게 제어하기 위해 `executor_node.py`, 하드웨어 드라이버(`rg2_gripper.py`, `dsr_interface.py`), 직접 교시 좌표 기록 CLI(`teach_zones.py`), 런치 파일(`executor.launch.py`, `store_bot.launch.py`)을 구현함.
- Doosan `DSR_ROBOT2.py`의 모든 동기 호출이 `rclpy.spin_until_future_complete(g_node, future)`를 호출하는 구조를 고려해, 메인 `MultiThreadedExecutor`에 등록하지 않는 전용 보조 노드(`store_dsr_client`)로 데드락을 원천 차단하고, E-Stop/Cancel 시에는 별도 콜백 그룹에서 `/{robot_id}/dsr_controller2/motion/move_stop` (`MoveStop(stop_mode=1)`)를 비동기로 즉시 전송하도록 설계함.

### Responsible Agent(s)
- **Codex (`codex exec -s workspace-write` & `codex exec review --uncommitted`)**:
  - `hardware/rg2_gripper.py`, `hardware/dsr_interface.py`, `nodes/executor_node.py`, `nodes/teach_zones.py`, `launch/executor.launch.py`, `launch/store_bot.launch.py` 및 `test_rg2_gripper.py`, `test_executor_dry_run.py` 구현
  - 크로스 리뷰 수행 (급정지 실패 래치 누락, 직선 이동 `J6` 보호 미적용, `home(J6=0°)` 출발 `reorient` 과도 차단, `mock_robot` 동시 수락 `BUSY` 응답 누락 지적)
- **Antigravity**:
  - `DSR_ROBOT2.py` 및 `MoveStop.srv` 실제 반환 규격 검증 (`set_tool`/`set_tcp`/`movej`/`movel` 성공=`0`)
  - Codex 리뷰 핵심 지적 4건 반영 (`_stop_fault` 래치 + Heartbeat `ERROR`, `_move_linear` 실행 전 `J6 ±150°` 예측 검사, `reorient` 접근 후 예상 `J6` 기준 사전 판별, `mock_robot` 동시 수락 `_active_goal` 처리) 및 단위 테스트 3건 추가
  - `colcon build --symlink-install`, `pytest`, CycloneDDS `executor_node` `dry_run` 다중 프로세스 스모크 테스트 검증

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/rg2_gripper.py`: OnRobot RG2 Modbus TCP 드라이버 (`slave`/`device_id` 양쪽 호환, `dry_run` 및 `simulate_grip_fail` 지원)
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: Doosan M0609 전용 보조 노드 래퍼 (`set_tool`/`set_tcp`/`movej`/`movel`/`get_current_posx`/`get_current_posj`, `dry_run` 시뮬레이션 지원)
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: 7개 `action_type` 실물 실행기, `dry_run` 모드, `GRASP_FAIL` 시 안전 후퇴 후 개방, `_move_linear` `J6 ±150°` 보호, `MoveStop` 비동기 급정지 및 `_stop_fault` 래치, Blackbox 기록, `/store/node_health` Heartbeat
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/teach_zones.py`: 직접 교시 모드(`robot_mode=0`)에서 `home`, `scan_pose`, `z_min_mm`, `employee_handover_pose`, `buffer_slots`를 읽어 `.bak` 백업 후 `config/workspace_zones.yaml`에 저장하는 대화형 CLI (`--print-only` 지원)
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/mock_robot.py`: 동시 수락 목표가 `BUSY` 결과를 정상 반환하도록 `_active_goal` 처리 보강
- `/home/yongsoo/store_ws/src/store_bringup/launch/executor.launch.py`, `store_bot.launch.py`: 기본 안전값 `dry_run:=true`로 `executor_node` 기동 (`use_mock_robot:=false` 시 전환)
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_rg2_gripper.py`, `test_executor_dry_run.py`: RG2 드라이버 및 `executor_node` `dry_run` 전체 시나리오 단위 테스트 추가

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control store_bringup` 성공
- `pytest -v src/store_robot_control/tests` → **51 passed** (기존 28건 + 신규 23건)
- CycloneDDS 다중 프로세스 `executor_node` (`dry_run:=true`) 스모크 테스트: `home`, `scan_pose`, `pick_place`, `give_expired_box`, `Z_MIN_VIOLATION` 차단, `/store/node_health` Heartbeat, `robot_blackbox.jsonl` 기록 검증

### Follow-ups / Caveats
- 실물 로봇(`dry_run:=false`) 최초 실행 전 반드시 티치펜던트에서 **`J6` 소프트 리미트 `±150°`**를 2중 방어로 설정하고, `ros2 run store_robot_control teach_zones`로 실제 `home`, `scan_pose`, `z_min_mm`을 먼저 실측해 `config/workspace_zones.yaml`에 반영해야 함.
- 실물 첫 테스트는 낮은 속도(`velocity:=20.0 acceleration:=20.0`)로 `home` / `scan_pose`부터 단계적으로 검증 권장.

## 2026-11-26 20:54 — 자동 모드(`robot_mode=1`)에서 `set_tool` 실패(`-1`) 버그 수정

### Goal / Background
- 실물 로봇 자동 모드(`robot_mode=1`)에서 `executor_node` 기동 시 Doosan 제어기가 `set_tool("Tool_Weight")` 호출을 거부(`-1`)하여 노드가 종료되는 문제 해결.
- Doosan 제어기는 수동 모드(`robot_mode=0`)에서만 `set_tool`/`set_tcp`를 허용하며, A4 프로젝트(`doosan_adapter.py`)와 동일하게 현재 설정된 Tool/TCP가 이미 일치하면 생략하고 다를 때만 일시 수동 모드 전환 후 복원하도록 수정.

### Responsible Agent(s)
- **Antigravity**: 실물 로봇 `get_current_tool`/`get_current_tcp` 및 `DSR_ROBOT2.py` 규격 확인, `dsr_interface.py` 수정 및 단위 테스트 추가/검증.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: `_ensure_tool_and_tcp()` 추가 (`get_tool`/`get_tcp` 선조회 → 일치 시 스킵, 불일치 시 `set_robot_mode(0)` → 적용 → `finally`에서 기존 모드 복원).
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_executor_dry_run.py`: `test_ensure_tool_and_tcp_skips_when_matching_and_switches_mode_when_needed` 추가.

### Verification & Review
- `pytest -v src/store_robot_control/tests` → **52 passed**.

## 2026-11-26 20:59 — Doosan ZYZ 짐벌락(`|ry|≈180°`) 손목 각도 보정 및 테스트 fixture 분리

### Goal / Background
- 실물 M0609 `home`/`scan_pose` 동작 성공 확인 중, `ry≈180°` ZYZ 짐벌락 구간에서 `rx`와 `rz`가 함께 변하는(`116.9° → 140.9° → 143.9°`) 현상을 포착함.
- 기존 `move_linear_4d`와 `_predict_j6_after_linear`가 원시 `rz`만 비교하면 `pick_place` 시 불필요한 손목 회전이나 `J6_LIMIT_VIOLATION` 오탐이 발생할 수 있어, 하향 기준 자세를 `(rx=0, ry=180, rz)`로 정규화하고 `|ry|≈180°`에서 실제 Z축 회전 `(rz - rx)` 기준으로 J6 변화를 예측하도록 보정함.
- 또한 실물 `teach_zones` 저장(`z_min_mm=-13.78 mm`)으로 실제 `config/workspace_zones.yaml`이 바뀔 때 `pytest`가 영향받지 않도록 테스트용 고정 설정(`tests/conftest.py`)으로 분리함.

### Responsible Agent(s)
- **Antigravity**: 실물 로그 분석, `dsr_interface.py`/`executor_node.py` ZYZ 짐벌락 보정, `tests/conftest.py` 분리 및 회귀 테스트 추가, 재빌드 및 검증.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: `move_linear_4d()` 기본 하향 자세를 `(rx=0.0, ry=180.0)`으로 정규화하고 `dry_run` J6 누적 계산에 `(rz - rx)` 적용.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: `_predict_j6_after_linear()`에서 `|ry|≈180°`일 때 `(cur_rz - cur_rx)` 기준으로 `delta_rz` 계산.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/conftest.py`, `test_safety_guard.py`, `test_reorient_motion.py`, `test_executor_dry_run.py`: 고정 테스트 설정 분리 및 ZYZ 짐벌락 회귀 테스트(`test_zyz_gimbal_lock_at_ry_180_does_not_false_trigger_j6`) 추가.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공
- `pytest -v src/store_robot_control/tests` → **53 passed**.

## 2026-11-26 21:23 — 상품 카테고리 3종(`paper_box`, `snack_bag`, `candy_chocolate`) 정정 및 RG2 힘 제어 파지 기본값 반영

### Goal / Background
- 실제 프로젝트 상품 카테고리 3종(**종이상자 `paper_box`**, **봉지과자 `snack_bag`**, **캔디·초콜릿류 `candy_chocolate`**)과 특수 작업용 2종(`expired_box`, `cigarette_mock`)으로 `config/products_catalog.yaml`을 정확히 수정함.
- OnRobot RG2의 `grip_detected` 하드웨어 특성(목표 폭 도달 전에 물체와 접촉해 힘으로 멈춰야 감지됨)에 맞춰 접근/해제 폭(`open_width_mm`)과 닫힘 목표 폭(`grasp_width_mm`)을 분리하고, 추후 개별 상품 예외 튜닝을 위한 `products:` 2단계 조회(`products` → `categories` → `defaults`)를 `grasp_planner.py`에 추가함.

### Responsible Agent(s)
- **Antigravity**: `products_catalog.yaml`, `grasp_planner.py`, 단위 테스트 수정 및 재빌드/검증.

### Detailed Changes
- `/home/yongsoo/store_ws/config/products_catalog.yaml`: `paper_box`, `snack_bag`, `candy_chocolate` (+ `expired_box`, `cigarette_mock`) 및 `open_width_mm`/`grasp_width_mm`/`products` 구조 반영.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/grasp_planner.py`: `products` → `categories` → `defaults` 2단계 조회 및 `open_width_mm` 명시 지원.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_grasp_planner.py`, `test_executor_dry_run.py`: 3종 카테고리 및 개별 상품 override 테스트 추가/갱신.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공
- `pytest -v src/store_robot_control/tests` → **56 passed**.

## 2026-11-26 21:30 — `mock_robot` `/store/mode/remote_override` 발행 연동 및 `.gitignore` 백업 파일 제외

### Goal / Background
- LLM 파트 협의서(`TEAM_SYNC_AGREEMENT.md`)의 원격 모드 전환 토픽(`/store/mode/remote_override`, `std_msgs/Bool`) 연동을 `remote_node` 본체 구현 전에도 `mock_robot`으로 즉시 테스트할 수 있도록 보강함.
- `teach_zones` 실행 시 생성되는 `*.bak` 백업 파일이 Git 저장소에 커밋되지 않도록 `.gitignore`에 추가함.

### Responsible Agent(s)
- **Antigravity**: `mock_robot.py` 및 `.gitignore` 수정, 재빌드 및 ROS 2 서비스→토픽 스모크 테스트 검증.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/mock_robot.py`: `/store/mode/remote_override` (`std_msgs/Bool`) 퍼블리셔 추가 및 `/store/verify_remote_key` 인증 성공 시 `True`/`False` 발행.
- `/home/yongsoo/store_ws/.gitignore`: `*.bak` 패턴 추가.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공
- `pytest -v src/store_robot_control/tests` → **56 passed**
- ROS 2 스모크 테스트: `/store/verify_remote_key` 호출 시 `/store/mode/remote_override`에 `True` → `False` 정상 발행 확인.

## 2026-11-26 21:32 — `README.md` 추가 및 GitHub 신규 레포(`dduter/24h-store-cobot`) 초기 커밋·푸시

### Goal / Background
- 팀원(LLM·비전·UI)과의 코드 통합 및 공용 인터페이스(`store_interfaces`) 공유를 위해 신규 GitHub 저장소(`dduter/24h-store-cobot`)를 생성하고 로봇 파트 1차 완성본을 업로드함.

### Responsible Agent(s)
- **Antigravity**: `README.md` 작성, 스테이징 파일 점검(비밀값/백업 파일 제외 확인), 초기 커밋 및 `gh repo create --public --push` 실행.

### Detailed Changes
- `/home/yongsoo/store_ws/README.md`: 워크스페이스 구조, 빌드/테스트/실행 명령, 팀 인터페이스 규약 요약 작성.
- GitHub 원격 저장소 생성 및 `main` 브랜치 초기 푸시 (`https://github.com/dduter/24h-store-cobot`).

### Verification & Review
- `.env`, `*.bak`, `build/`, `install/`, `log/` 미포함 확인 후 푸시 완료.

## 2026-11-26 21:39 — `ws_dsr` 호환 `config_dir` 탐색 보강 및 `README.md` 업데이트

### Goal / Background
- `/home/yongsoo/ws_cobot_pjt/ws_dsr`로 이전한 환경과 GitHub(`dduter/store_ws`) 레포를 동기화하고, `ws_dsr` 내부 선택 빌드 및 `~/ws_cobot_pjt/ws_dsr/config` 자동 탐색을 지원함.

### Responsible Agent(s)
- **Antigravity**: `mock_robot.py` 및 `README.md` 업데이트, `dduter/store_ws` 커밋 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/mock_robot.py`: `find_config_dir()`에 `~/ws_cobot_pjt/ws_dsr/config` 탐색 추가.
- `/home/yongsoo/store_ws/README.md`: `ws_dsr` 내부 `--packages-select` 빌드 방법 추가.

## 2026-11-26 21:44 — `ws_dsr` 분리 원복 및 `~/store_ws` 단일 Overlay 관리 확정

### Goal / Background
- `~/ws_cobot_pjt/ws_dsr`와 `~/store_ws` 양쪽에 파일이 분산되는 혼선을 없애기 위해 `ws_dsr`에서 이전했던 `store_*` 패키지·설정·빌드 산출물을 모두 제거하고, `~/ws_cobot_pjt/ws_dsr`(Underlay) + `~/store_ws`(Overlay / GitHub 단일 관리) 분리 구조로 확정함.

### Responsible Agent(s)
- **Antigravity**: `ws_dsr` 내 추가 파일 일괄 제거, `store_ws`의 `find_config_dir()` 및 `README.md` 정리, 재빌드·`pytest`·패키지 분리 검증 및 GitHub 푸시.

### Detailed Changes
- `/home/yongsoo/ws_cobot_pjt/ws_dsr/`: 이전했던 `config/`, `data/`, `scripts/`, `src/store_*`, `build/store_*`, `install/store_*`, 문서 파일 제거(기존 `doosan-robot2`/`onrobot-ros2`/`rg2`/`rokey1` 원본 보존).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/mock_robot.py`, `README.md`: `store_ws` 단일 Overlay 구조로 정리.

## 2026-11-26 21:52 — 1일차 마무리: 레포명 `store_ws` 확정, `sod` 통합 및 실물·팀 싱크 검증 요약

### Goal / Background
- 1일차 로봇 파트 작업 마무리: GitHub 레포명 확정(`dduter/store_ws`), 터미널 `sod` alias 통합, 실물 로봇 검증 결과 및 팀원(LLM·비전 A/B) 싱크 정리 기록.

### Responsible Agent(s)
- **Antigravity**: GitHub 레포명 변경(`24h-store-cobot` → `store_ws`), `~/.bashrc` `sod` 통합 및 새 셸 라이브 검증, LLM·비전 A/B 싱크 회신문 작성.

### Detailed Changes
- GitHub 저장소 이름 변경: `https://github.com/dduter/store_ws`
- `/home/yongsoo/.bashrc`: `sod` alias가 `~/ws_cobot_pjt/ws_dsr/install/setup.bash`와 `~/store_ws/install/local_setup.bash`를 함께 로드하도록 갱신.

### Verification & Review
- **실물 로봇(M0609 + RG2) 검증 완료**:
  - Eye-in-Hand 캘리브레이션(`T_gripper2camera.npy`, 재투영 오차 `0.39 px`, `verify.py` 과일 모형 파지 확인)
  - `teach_zones`로 `home`, 임시 `scan_pose`, `z_min_mm(-13.78 mm)` 실측 저장
  - 실물 `executor_node`로 `home`, `scan_pose`, `Z_MIN_VIOLATION` 차단, `GRASP_FAIL` 안전 후퇴·개방(`final_pose z=259.99 mm, rz=0.61°`) 동작 검증 완료
- **소프트웨어 검증 완료**: `pytest` **56 / 56 통과**, 롤백 후 `sod` 단독 새 셸 라이브 스모크 테스트 통과.

### Follow-ups / Next Steps (내일 이어서 할 일)
1. LLM·비전 A/B 담당자 회신 확인 (`mm`+`deg`, `zones.buffer_slots`, 3종 카테고리, `store_interfaces` 6개 파일 일치, `Z_surface` 오프셋 담당).
2. 실제 상품 6–9종으로 실물 `pick_place` 완주 및 `reorient`(180° 뒤집기) 파지·회전 실측 튜닝.
3. 테이블 세팅 확정 후 `teach_zones`로 최종 `scan_pose`, `zones.buffer_slots` 2–4개, `employee_handover_pose` 실측 업데이트.

## 2026-11-27 09:27 — 비전 A/B 합의 반영 (`apply_grasp_depth_offset` 기본값 `true` 활성화)

### Goal / Background
- 비전 A/B 회신에서 **`Z` 파지 깊이 오프셋 담당**을 "비전 B는 순수 물체 윗면(`Z_surface`, mm) 전달 → 로봇이 카테고리별 `grasp_depth_offset_mm` 차감(이중 차감 방지)"으로 최종 합의함에 따라, `executor_node` 및 런치 기본값을 `apply_grasp_depth_offset: true`로 전환함.

### Responsible Agent(s)
- **Antigravity**: `control_params.yaml`, `executor_node.py`, `executor.launch.py` 기본값 전환, 대상(`target_pose`)만 차감하고 투하 위치(`dest_pose`)는 유지되는지 검증하는 단위 테스트 추가, 재빌드 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/config/control_params.yaml`: `motion.apply_grasp_depth_offset: true` 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: `apply_grasp_depth_offset` 파라미터 기본값을 `control_params.yaml`(`True`)에서 읽도록 변경.
- `/home/yongsoo/store_ws/src/store_bringup/launch/executor.launch.py`: `apply_grasp_depth_offset` 런치 인자(기본 `"true"`) 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_executor_dry_run.py`: `test_depth_offset_applied_only_to_target_not_dest` 추가.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control store_bringup` 성공
- `pytest -v src/store_robot_control/tests` → **57 passed**.

## 2026-11-27 09:48 — `teach_zones` 실행 시 `Tool_Weight`/`GripperDA_v1` 자동 보증 및 4D 좌표 출력 보강

### Goal / Background
- 로봇 재부팅 직후 티치펜던트 TCP가 기본 플랜지(`0 mm`)인 상태에서 `teach_zones --print-only`를 실행하면 RG2 길이(`+240 mm`)만큼 높은 플랜지 Z가 출력되어 `executor_node`(`GripperDA_v1`)와 Z 오차가 발생하는 문제를 해결함.

### Responsible Agent(s)
- **Antigravity**: 실물 Doosan 컨트롤러 로그(`NOT REACHABLE`) 및 `GetCurrentToolFlangePosx` 분석, `teach_zones.py` 수정 및 재빌드.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/teach_zones.py`: `_ensure_tool_tcp()` 추가(실행 시 `Tool_Weight`/`GripperDA_v1` 자동 확인·적용 및 활성 Tool/TCP 출력) + 복사용 `4D [x, y, z, rz]` 라인 추가 출력.

## 2026-11-27 11:26 — 2일차 핵심 제어 3종 일괄 구현 및 크로스모델 리뷰 반영 (`two_stage_flip`, 도달 오차 가드, `teleop_math`)

### Goal / Background
- 작업대 최종 배치 확정과 무관하게 선행 가능한 로봇 제어 핵심 3종을 일괄 구현하고, 실물 테스트에서 확인된 M0609 상단 도달 한계(`Z ≤ ~400 mm`)와 Doosan `movel`의 무음 `NOT REACHABLE` 반환 문제를 차단함.
- 앞↔뒤 180° 반전(`reorient`) 시 J6 케이블 한계(`±150°`)를 넘지 않으면서 임의의 `rz ∈ [-90°, +90°]`에서 안전하게 뒤집을 수 있는 2단계 90°+90° 반전 시퀀스를 도입함.
- 원격 텔레옵(`remote_node`) 구현의 순수 수학 기반(`DeadzoneFilter`, `EmaLowPassFilter`, `GripperGestureDebouncer`, `TeleopWatchdog`, `TeleopPipeline`)을 ROS 비의존 코어 모듈로 선구축함.

### Responsible Agent(s)
- **Codex (`codex exec -s workspace-write`)**: `two_stage_flip` 16단계 6D 경로 계획(`reorient_motion.py`), `move_linear_6d` / `predict_linear_j6` / `MotionUnreachableError`(`dsr_interface.py`), `executor_node.py` 6D 반전 실행 및 `OUT_OF_REACH` 매핑, 순수 Python 원격 조작 코어(`core/teleop_math.py`) 및 단위 테스트 초기 구현.
- **Codex (`codex exec review --uncommitted`)**: 크로스모델 코드 리뷰 수행 — P1 2건(상승 여유 부족 시 `lift_z` 클램핑 문제, `teach_zones` Tool/TCP 실패 미검증) 및 P2 2건(`move_linear_6d` 회전 오차 미검사, `teach_zones` 4D `rz` 하드코딩) 지적.
- **Antigravity**: 전체 diff 검증, `DSR_ROBOT2.py` (`get_solution_space`, `ikin`) 시그니처 대조, Codex 리뷰 지적 4건 전량 수정 및 회귀 테스트 추가, 재빌드·108개 단위 테스트·CycloneDDS 6종 라이브 스모크 테스트 실행, `WORKLOG.md` 기록 및 GitHub 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/config/workspace_zones.yaml`: M0609 실측 하향 도달 한계를 반영해 `safety.bounds.z_max`를 `650.0` → `400.0`으로 하향 조정(기존 실측 `z_min_mm`, `home_*`, `scan_pose_*` 값 보존).
- `/home/yongsoo/store_ws/config/control_params.yaml`: `motion.arrival_tolerance_mm: 15.0`, `motion.arrival_angle_tolerance_deg: 10.0` 추가.
- `/home/yongsoo/store_ws/config/products_catalog.yaml`: 기본 및 일반 상품 3종(`paper_box`, `snack_bag`, `candy_chocolate`)의 `reorient_mode`를 `"two_stage_flip"`으로 전환(`expired_box`, `cigarette_mock`은 `"none"` 유지).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_two_stage_flip()` 추가 및 `MotionWaypoint` 확장(`orientation_zyz`, `action`, `check_grip`). 상승 여유(`lift_z > z_stand`, `lift_z <= z_max`)를 확보할 수 없는 목표 높이는 클램핑하지 않고 `OUT_OF_REACH`로 거부하도록 보강.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/teleop_math.py` (신규): `DeadzoneFilter`, `EmaLowPassFilter`, `GripperGestureDebouncer`(0.5s), `TeleopWatchdog`(0.5s), `TeleopPipeline` 순수 Python 구현(ROS import 0건).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: `MotionUnreachableError`, `ensure_tool_and_tcp()`, `top_down_effective_rz()`, `zyz_orientation_error_deg()`(SO(3) 측지 회전 오차로 ZYZ 짐벌락 자동 해소), `move_linear_6d()`, `predict_linear_j6()`(`get_solution_space` + `ikin`) 구현 및 `movel` 이후 위치(15 mm)·회전(10°) 도달 검증 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: `two_stage_flip` 6D 실행(`_execute_two_stage_flip`, 1/2단계 `GRASP_FAIL` 후퇴 자세 보존), `reorient_mode == "none"` 시 `INVALID_ACTION` 거부, `MotionUnreachableError` → `OUT_OF_REACH` 매핑(E-Stop/Cancel 중단 우선 유지).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/teach_zones.py`: `ensure_tool_and_tcp()` 공용 헬퍼로 Tool/TCP 설정 실패 시 즉시 중단하도록 강화, 하향 자세의 실제 유효 `rz`(`rz - rx`) 출력·저장 및 비하향 자세 4D 복사/저장 방지 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/`: `test_teleop_math.py` 신규 추가 및 `conftest.py`, `test_reorient_motion.py`, `test_executor_dry_run.py`, `test_grasp_planner.py`, `test_safety_guard.py` 테스트 대폭 확충(총 108개).

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.14s`).
- `pytest -q src/store_robot_control/tests` → **108 passed in 7.39s** (순수 Python `core/` ROS 미임포트 검사 포함).
- CycloneDDS (`ROS_DOMAIN_ID=40`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) `executor.launch.py dry_run:=true` 6종 라이브 액션 스모크 테스트 통과:
  - `home` → `OK`
  - `reorient` (`rz=0°`, `rz=45°`) → `OK`
  - `reorient` (`z=390 mm`, 상승 여유 부족) → `OUT_OF_REACH`
  - `reorient` (`expired_box`) → `INVALID_ACTION`
  - `pick_place` (`z=410 mm > z_max`) → `OUT_OF_REACH`

### Follow-ups / Caveats
- 작업대 최종 배치 확정 후 `teach_zones`로 최종 `scan_pose`, `zones.buffer_slots`, `employee_handover_pose` 실측 업데이트 필요.
- 실물 M0609 + RG2에서 빈 공간/실제 상품으로 `two_stage_flip` 실행 시 `standing_offset_z_mm(20 mm)`, `reapproach_offset_mm(35 mm)`, `flip_lift_delta_mm(90 mm)` 실물 튜닝 권장.
- 원격 텔레옵 웹캠/MediaPipe ROS 노드(`remote_node`) 구현 시 `core/teleop_math.py`의 `TeleopPipeline`을 바로 연결하여 사용.

## 2026-11-27 11:36 — `DSR_ROBOT2` 서비스 미준비 시 영구 대기 버그 수정 및 `executor_node` 시작/작업 로그 추가

### Goal / Background
- 실물 `roboton` 실행 직후 `dsr_controller2` 활성화(11:33:25)보다 1초 먼저 `executor_node`를 실행(11:33:24)했을 때, 두산 벤더 라이브러리(`DSR_ROBOT2.get_tool()`/`get_tcp()`)가 `wait_for_service()` 없이 즉시 `call_async` 후 `spin_until_future_complete()`를 호출해 영구 대기(행)되는 현상을 발견·해결함.
- `executor_node` 실행 시 초기화 완료 여부 및 `dry_run`/속도/작업 시작·종료가 터미널에 명확히 보이도록 로그를 보강함.

### Responsible Agent(s)
- **Antigravity**: 실물 프로세스·ROS 그래프·`DSR_ROBOT2.py` 원인 분석, `wait_for_dsr_services()` 및 `executor_node` 로그 추가, 단위 테스트 추가·재빌드·푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: `wait_for_dsr_services()` 추가 및 `ensure_tool_and_tcp()` 선두에서 `_ros2_get_current_tool`, `_ros2_get_current_tcp`, `_ros2_movel` 서비스 준비 대기(최대 30초, 초과 시 명시적 예외 발생).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: 초기화 완료 시 `dry_run`/속도/파지체크/깊이오프셋 설정 로그와 각 액션 작업 시작·종료 로그 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_executor_dry_run.py`: `test_wait_for_dsr_services_waits_and_times_out` 추가.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.31s`).
- `pytest -q src/store_robot_control/tests` → **109 passed**.

## 2026-11-27 11:43 — 실물 M0609 역기구학 실측 기반 `two_stage_flip` 측면(`±Y`) 반전 수정

### Goal / Background
- 실물 M0609 + RG2(240 mm)에서 `reorient` 첫 테스트 시 `open_gripper → approach → descend → grasp → lift_stage1`까지 정상 동작 후, `tilt_90`의 전방(`+X`, `ZYZ=(0, 90, 0)`) 기울기에서 두산 컨트롤러가 `NOT REACHABLE`을 반환했고, 새로 추가한 SO(3) 회전 도달 가드가 `회전 오차 90.0deg > 10.0deg` → `OUT_OF_REACH`로 안전하게 차단함.
- 실물 `dsr_controller2`의 `/dsr01/dsr_controller2/motion/ikin`을 읽기 전용으로 조회한 결과, 전방(`+X`) 90° 기울기 시 RG2 길이 때문에 손목 플랜지가 `x=110 mm`로 접히며 `J3=162.2°`(M0609 한계 `±160°` 초과), 후방(`-X`) 기울기 시 `J5=167.1°`(한계 `±135°` 초과)에 걸리는 반면, 측면(`±Y`, `ZYZ=(±90, 90, ±90)`) 기울기는 `ry=180°→90°` 연속 경로 전체에서 모든 관절(`J1~J6`)이 안전 범위 내에 머무름을 확인함.

### Responsible Agent(s)
- **Antigravity**: 실물 M0609 `ikin` 워크스페이스·연속 경로 전수 조사, `plan_two_stage_flip()` 측면(`±Y`) 반전 알고리즘 수정 및 단위 테스트 갱신, 재빌드·110개 테스트 검증·푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_two_stage_flip()`에서 파지 후 상승(`lift_stage1`) 시 하향 기준 `(0, 180, 0)`으로 정렬하고, `y >= 0`이면 `+Y` (`ZYZ=(+90, 90, +90)`), `y < 0`이면 `-Y` (`ZYZ=(-90, 90, -90)`) 방향으로 손목이 로봇 중심 쪽으로 접히도록 90°+90° 2단계 반전 경로 변경.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_reorient_motion.py`, `test_executor_dry_run.py`: 측면 반전 및 음수 `y` 방향 대칭 테스트 추가·갱신.

### Verification & Review
- 실물 M0609 `ikin` 연속 경로(`ry=180°→90°` @ `(350, 20, 260)`) 확인: `J2=-1.2°→29.7°`, `J3=79.1°→99.2°`, `J5=102.1°→123.6°`, `J6=2.3°→-48.8°` (전 구간 단조·안전).
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.35s`).
- `pytest -q src/store_robot_control/tests` → **110 passed**.

## 2026-11-27 12:08 — `give_expired_box` 액션 및 직원 전달용 설정 제거 (폐기통 `pick_place` 단일화)

### Goal / Background
- 실제 매장 운영 시나리오를 "직원이 폐기 지시 → 로봇은 폐기 대상 상품을 폐기통에 투입(`pick_place`) → 이후 직원이 직접 폐기통 처리"로 확정함에 따라, 불필요한 `give_expired_box` 액션과 `employee_handover_pose` / `expired_box` 설정을 제거함(`store_interfaces` 메시지 정의는 호환성 유지를 위해 무변경).

### Responsible Agent(s)
- **Antigravity**: 코드·설정·테스트·`README.md`에서 `give_expired_box`/`employee_handover_pose`/`expired_box` 제거, 재빌드·109개 테스트 검증 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/mock_robot.py`, `executor_node.py`: `TRANSFER_ACTIONS` 및 `employee_handover_pose` 분기 제거(`give_expired_box` 요청 시 `INVALID_ACTION` 반환).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/teach_zones.py`: `4. 직원 전달 자세` 메뉴 제거 및 `4. 버퍼 슬롯`으로 정리.
- `/home/yongsoo/store_ws/config/workspace_zones.yaml`, `products_catalog.yaml`, `README.md`: `employee_handover_pose` 및 `expired_box` 제거(기존 실측 `z_min_mm`, `home_*`, `scan_pose_*` 보존).
- `/home/yongsoo/store_ws/src/store_robot_control/tests/`: `conftest.py`, `test_executor_dry_run.py`, `test_grasp_planner.py` 갱신 및 `give_expired_box` → `INVALID_ACTION` 회귀 테스트 추가.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.08s`).
- `pytest -q src/store_robot_control/tests` → **109 passed**.

## 2026-11-27 12:25 — 원격 텔레옵 스택(`remote_node`, `hand_client`, `remote.launch.py`) 및 `executor_node` 원격 오버라이드 안전 연동

### Goal / Background
- 로봇 파트 2대 핵심 담당 중 하나인 원격 조작 시스템(`remote_node` + `hand_client` + `remote.launch.py`)을 구축하고, 실물 `executor_node`에 `/store/mode/remote_override` 인터록을 추가해 원격 모드 진입 시 자동 작업이 즉시 안전 중단·차단되도록 완성함.

### Responsible Agent(s)
- **Codex (`codex exec -s workspace-write`)**: `executor_node.py` `/store/mode/remote_override` 구독·중단, `remote_node.py`(키 인증·데드존/EMA/0.5s 디바운스/0.5s 워치독·작업영역+반경 이분탐색 클램핑·RG2 제어), `hand_client.py`(웹캠 MediaPipe + 키보드 폴백), `remote.launch.py`, `control_params.yaml` 확장 및 초기 단위 테스트 구현.
- **Codex (`codex exec review --uncommitted`)**: 크로스모델 리뷰 수행 — P1 4건(자동↔원격 제어권 전환 동기화, `TRANSIENT_LOCAL` 재접속/크래시 복구, 이동 중 독립 워치독 급정지, 정지 직후 이동 경합 방지) 지적.
- **Antigravity**: 전체 diff 검토, Codex 리뷰 P1 4건 전량 수정(`TRANSIENT_LOCAL` QoS 통일, `NodeHealth` `busy/remote_override` 기반 제어권 핸드오버, 발행자 이탈 시 자동 복구, `_stop_group` 독립 10 Hz 워치독·후속 급정지 타이머) 및 회귀 테스트 추가, 빌드·135건 단위 테스트·늦은 접속 CycloneDDS 스모크 테스트 검증, `README.md`/`WORKLOG.md` 갱신 및 GitHub 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/remote_node.py` (신규): `/store/verify_remote_key` 인증(`REMOTE_ADMIN_KEY` `compare_digest`), `/store/mode/remote_override`(`TRANSIENT_LOCAL`) 발행, `/store/remote/hand_cmd` JSON 수신, `TeleopPipeline` 필터링, 작업공간·반경·J6 클램핑(항상 수직 하향 `ry=180°` 유지), `NodeHealth` 기반 자동 실행기 유휴 확인 핸드오버, 독립 10 Hz 워치독/급정지 감시 타이머.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/hand_client.py` (신규): MediaPipe Hands 웹캠 입력(`landmarks_to_hand_command`) + `mediapipe` 미설치 시 자동 키보드(`WASD/RF/G/Q`) 폴백 클라이언트.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`, `mock_robot.py`: `REMOTE_OVERRIDE_QOS`(`TRANSIENT_LOCAL` + `RELIABLE`) 적용, 원격 모드 시 진행 작업 `quick_stop()` + `CANCELED` 중단 및 새 목표 거부, 발행자 크래시 시 자동 잠금 해제, `NodeHealth`에 `busy`/`remote_override` 상태 반영.
- `/home/yongsoo/store_ws/src/store_bringup/launch/remote.launch.py` (신규), `src/store_robot_control/setup.py`, `config/control_params.yaml`, `README.md`: 런치(안전 기본값 `dry_run:=true`), 엔트리포인트(`remote_node`, `hand_client`), 텔레옵 제어 파라미터 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/test_remote_node.py` (신규), `test_executor_dry_run.py`: 원격 인증·오버라이드·경계 클램핑·디바운스·워치독·핸드오버·발행자 크래시 복구·키보드/랜드마크 테스트 추가(총 135건).

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control store_bringup` 성공 (`2.25s`).
- `pytest -q src/store_robot_control/tests` → **135 passed in 13.26s**.
- CycloneDDS (`ROS_DOMAIN_ID=40`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) 교차 프로세스 스모크 테스트 통과:
  - 잘못된 키 거부, 유효 키 인증 → 원격 ON 시 `executor_node` 목표 거부 → `/store/remote/hand_cmd` 발행 → 원격 OFF 시 재수락
  - `remote_node` 원격 ON 이후 늦게 실행된 `executor_node`가 `TRANSIENT_LOCAL` 래치 상태를 즉시 수신해 목표 거부(`LATE-JOINER SMOKE OK`, `HANDOVER SMOKE OK`).

### Follow-ups / Caveats
- 웹캠 손 추종 모드(`--mode webcam`)를 실사용하려면 `pip install mediapipe` 설치 필요(미설치 환경에서는 `--mode keyboard` 또는 `--mode auto`로 즉시 키보드 테스트 가능).
- 실물 M0609 + RG2에서 `remote_node`를 `dry_run:=false`로 실행할 때는 `executor_node`와 동일 로봇에 대한 Doosan 서비스 동시 접근을 피하거나, 핸드오버 검증 후 저속(`velocity:=20.0`)으로 테스트 권장.

## 2026-11-27 12:56 — `gentle_flip` 저속 소각도 `J5` 반전, 단독 시나리오 검증 도구(`robot_demo`), `store_bot.launch.py` 통합 정비

### Goal / Background
- 실물 테스트에서 90° 눕히기(`two_stage_flip`)가 팔 본체(`J1/J4`)를 크게 휘두르며 테이블 가까이 내려와 위험했던 문제를 해소하기 위해, 손목 `J5`만 40° 저속으로 기울이고 지지면 안전 여유 높이까지 내려놓은 뒤 복원하는 `gentle_flip`을 기본 `reorient` 모드로 전환함.
- 비전/LLM 통합 없이 로봇 파트 단독으로 전 시나리오(안전 차단·동작·비상정지·블랙박스 로그)를 자동 검증·시연할 수 있는 `robot_demo` 및 `scripts/run_robot_demo.py`를 구축함.
- `store_bot.launch.py`에서 `executor_node`와 `remote_node` 통합 실행(`enable_remote`) 및 실행 파라미터 전달을 일원화하고 `README.md`를 최신화함.

### Responsible Agent(s)
- **Codex (`codex exec -s workspace-write`)**: `plan_gentle_flip`, `executor_node._execute_gentle_flip`, `robot_demo.py`, `scripts/run_robot_demo.py`, `store_bot.launch.py`, `executor.launch.py`, `mock_robot.py` 및 초기 단위 테스트 구현.
- **Codex (`codex exec review --uncommitted`)**: 크로스모델 리뷰 수행 — P1 3건(중복 액션 서버 판정 보강, `gentle_flip` 공중 낙하 방지 지지면 하강, `J5` 회전 중 TCP 원호 작업 범위 검증) 및 P2 1건(`robot_demo` 타임아웃 시 실행 중 목표 취소) 지적.
- **Antigravity**: 리뷰 지적 4건 전량 수정(`plan_gentle_flip` 원호 궤적 검증 + `place_flip`/`lift_after_flip` 추가, `dsr_interface.predict_forward_pose` `fkin` 검증, `robot_demo` 그래프/퍼블리셔 중복 서버 차단 및 타임아웃 자동 취소), `remote_node.py` 실시간 이동/그리퍼 로그 추가, 회귀 테스트 확충, 빌드·156건 단위 테스트·`store_bot.launch.py` + `robot_demo` CycloneDDS 10종 라이브 검증, `WORKLOG.md` 기록 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/config/control_params.yaml`, `products_catalog.yaml`: `reorient` 섹션(`gentle_tilt_deg: 40.0`, `flip_lift_delta_mm: 90.0`, `release_clearance_mm: 55.0`) 추가 및 일반 상품 3종 기본 `reorient_mode`를 `"gentle_flip"`으로 전환.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_gentle_flip()` 구현(`J5`를 0° 방향으로만 40° 회전, 240mm RG2 원호 궤적 작업범위 검증, `place_flip`→`release_flip`→`lift_after_flip`→`restore_j5` 10스텝 생성).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: `predict_forward_pose()`(`fkin`) 추가.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: `_execute_gentle_flip()` 구현(저속 `min(vel, slow_vel)`, 실기 `fkin` 궤적 검증, 지지면 여유 높이 하강 후 개방 및 복원), `step_delay_sec` dry_run 지원.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/robot_demo.py`, `/home/yongsoo/store_ws/scripts/run_robot_demo.py` (신규): `all`/`safety`/`motion`/`estop` 스위트 자동 검증, 중복 액션 서버 차단, 타임아웃 시 자동 목표 취소, 블랙박스 요약 출력.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/remote_node.py`: 실시간 원격 이동 좌표 및 그리퍼 개폐 정보 로그 추가.
- `/home/yongsoo/store_ws/src/store_bringup/launch/store_bot.launch.py`, `executor.launch.py`, `README.md`, `src/store_robot_control/setup.py`: 통합 런치 옵션(`enable_remote`, `velocity`, `acceleration`, `check_grip_detection`, `apply_grasp_depth_offset`, `step_delay_sec`), `robot_demo` 엔트리포인트, 최신 가이드 반영.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/`: `test_robot_demo.py` 신규 추가 및 `test_reorient_motion.py`, `test_executor_dry_run.py`, `test_grasp_planner.py` 갱신(총 156건).

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control store_bringup` 성공 (`2.26s`).
- `pytest -q src/store_robot_control/tests` → **156 passed in 13.78s**.
- `store_bot.launch.py use_mock_robot:=false dry_run:=true enable_remote:=true` + `ros2 run store_robot_control robot_demo --suite all --yes` (CycloneDDS) → **10 / 10 시나리오 전부 통과** (`z_min_violation`, `out_of_reach`, `invalid_action`, `home`, `scan_pose`, `pick_place`, `move_to_buffer`, `restore_from_buffer`, `reorient`, `estop`).

### Follow-ups / Caveats
- 실물 로봇(M0609 + RG2)에서 `robot_demo --suite safety` 및 `--suite estop` 실행으로 실물 비상정지 즉시 중단·복구 확인 후 4번(`hand_client` 웹캠/키보드 보완) 진행 예정.
- 최종 상품·작업대 확정 시 P0 체크리스트(`grasp_depth_offset_mm` 3종, `gentle_tilt_deg`/`release_clearance_mm`, `scan_pose`/`buffer_slots`/폐기통 좌표) 실측 튜닝 필요.

## 2026-11-27 14:13 — 방식 B `edge_pivot_flip` (가장자리 바닥 피벗 180° 반전) 기본 모드 적용 및 리뷰 반영

### Goal / Background
- `gentle_flip`(40° 기울기)만으로는 평평한 바닥에서 실제 180° 뒤집기가 어렵고, 공중 낙하 방식은 봉지과자 튕김 및 `J5=0°` 손목 특이점 위험이 있음을 확인함에 따라, 물체 가장자리를 집어 반대쪽 끝을 바닥 축(피벗) 삼아 `J5`를 60° 기울이며 앞으로 끌어 넘기는 무낙하·무특이점 방식(`edge_pivot_flip`)을 기본 `reorient_mode`로 구현함.

### Responsible Agent(s)
- **Codex (`codex exec -s workspace-write`)**: `plan_edge_pivot_flip`, `executor_node._execute_edge_pivot_flip`, `mock_robot.py` 지원, `control_params.yaml`/`products_catalog.yaml` 기본값 전환, `robot_demo.py` 순서 보정 및 단위 테스트 초기 구현.
- **Codex (`codex exec review --uncommitted`)**: 크로스모델 리뷰 수행 — P1 1건(드래그 후 `restore_j5` 시 정기구학 궤적 미검증) 및 P2 1건(작업 영역 외곽 `x≈500mm`에서 드래그 공간 부족 시 기존 `gentle_flip` 대비 거부 회귀) 지적.
- **Antigravity**: 리뷰 2건 전량 수정(`_check_j5_arc_fkin` 공용 헬퍼로 `pivot_tilt_j5`·`restore_j5` 양쪽 `fkin` 검증, 초기 계획 시 `OUT_OF_REACH` 발생하면 이동 전 `gentle_flip`으로 자동 폴백), 회귀 테스트 추가, 빌드·174건 단위 테스트·CycloneDDS `robot_demo` 10종 검증, `WORKLOG.md` 기록 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/config/control_params.yaml`, `products_catalog.yaml`: `reorient` 파라미터(`edge_offset_mm: 35.0`, `pivot_lift_mm: 70.0`, `pivot_tilt_deg: 60.0`, `drag_forward_mm: 55.0`, `release_z_offset_mm: 25.0`) 추가 및 일반 상품 3종(`paper_box`, `snack_bag`, `candy_chocolate`) 기본 `reorient_mode`를 `"edge_pivot_flip"`으로 전환.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_edge_pivot_flip()` 추가(`|J5| >= 20°`·동부호 유지로 `±15°` 손목 특이점 완전 차단, 240mm 원호 + 전방 드래그 작업범위 검증, 10스텝 생성).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`, `mock_robot.py`: `_execute_edge_pivot_flip()` 및 `_check_j5_arc_fkin()` 구현(기울기·복원 양쪽 `fkin` 검증, 공간 부족 시 이동 전 `gentle_flip` 자동 폴백).
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/robot_demo.py`, `tests/`: `scan_pose` 순서 조정 및 `edge_pivot_flip`·특이점·폴백·`restore_j5` 정기구학 회귀 테스트 추가(총 174건).

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.07s`).
- `pytest -q src/store_robot_control/tests` → **174 passed in 2.08s**.
- `executor.launch.py dry_run:=true` + `ros2 run store_robot_control robot_demo --suite all --yes` (CycloneDDS) → **10 / 10 시나리오 전부 통과**.
