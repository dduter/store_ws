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
