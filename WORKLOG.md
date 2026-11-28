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

## 2026-11-27 14:26 — 실물 M0609 `J5`/`fkin` 실측 반영 (`361 mm`, `50°` 자동 특이점 클램핑) 및 LLM 싱크 `zones.display_tiers` 추가

### Goal / Background
- 실물 M0609에서 `edge_pivot_flip` (`[330, 0, 50]`) 테스트 시 `pivot_lift`(`[312.57, 0, 102.10]`) 직후 `OUT_OF_REACH`로 안전 중단된 원인을 실물 `ikin`/`fkin`으로 조회한 결과:
  1. `pivot_lift`에서 실물 `J5 = +77.76°`로 낮아져 `60°` 감산 시 `+17.76°`(< 특이점 하한 `20°`)가 되어 특이점 가드가 정상 차단함.
  2. M0609 `J5` 손목 중심→RG2 TCP 실제 회전 반경이 `240 mm`가 아니라 `361 mm`(손목 링크 `121 mm` + RG2 `240 mm`)임을 `fkin`으로 실측 확인함.
- LLM 파트 싱크 요청에 따라 `config/workspace_zones.yaml`에 3단 진열대 키 구조(`zones.display_tiers.tier_1/2/3`, 각 3슬롯, 총 9슬롯)를 사전 추가함.

### Responsible Agent(s)
- **Antigravity**: 실물 M0609 `ikin`/`fkin` 정밀 계측, `tool_length_mm=361.0` 및 `pivot_tilt_deg=50.0`·실시간 특이점 여유 클램핑(`min(tilt, abs(j5) - 20.0)`) 적용, `zones.display_tiers` 추가, 재빌드·175건 테스트 검증 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/config/workspace_zones.yaml`: `zones.display_tiers` (`tier_1`, `tier_2`, `tier_3` 각 3슬롯, `[x, y, z, rz]` 임시 좌표 — 기존 실측 `z_min_mm`, `home_*`, `scan_pose_*`, `buffer_slots` 보존).
- `/home/yongsoo/store_ws/config/control_params.yaml`: `reorient.pivot_tilt_deg`를 `60.0` → `50.0`으로 조정.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: 실측 `J5→TCP` 반경 `tool_length_mm=361.0` 및 기본 `pivot_tilt_deg=50.0` 적용.
- `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`, `mock_robot.py`: `pivot_tilt_j5` 직전 실물 `J5` 기준으로 `effective_tilt = min(tilt, abs(j5) - 20.0)` 자동 클램핑(`≥35°` 유지, `|J5|≥20°` 특이점 보호).
- `/home/yongsoo/store_ws/src/store_robot_control/tests/`: `test_edge_pivot_caps_live_j5_at_77_deg_safely` 추가 및 기대값 갱신(총 175건).

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.31s`).
- `pytest -q src/store_robot_control/tests` → **175 passed**.

## 2026-11-27 14:33 — 실물 `drag_over` `J5` 특이점 실측 보정 (`pivot_tilt_deg: 35.0°`)

### Goal / Background
- 실물 M0609에서 `edge_pivot_flip`(`50°`) 테스트 시 `pivot_tilt_j5`까지는 정상 실행됐으나 `drag_over`(`Z: 231→57 mm`) 하강 중 `J2+J3`가 `+30°` 증가하면서 `J5`가 `+27.8°→+1.1°` 손목 특이점에 진입해 `ikin`이 `J6=+178.8°`로 뒤집히는 현상을 `predict_linear_j6`가 사전에 정확히 차단함(`J6_LIMIT_VIOLATION`).
- 실물 `ikin` 전수 조회 결과 `pivot_tilt_deg=35.0°`에서는 `Z=65 mm` 바닥 근처까지 내려와도 `J5=+25.7°`, `J4=+1.3°`, `J6=-2.0°`로 전 구간 특이점 없이 안전함을 확인하고 기본값을 `35.0°`로 확정함.

### Responsible Agent(s)
- **Antigravity**: 실물 M0609 `ikin` 드래그 하강 전수 계측, `pivot_tilt_deg` `50.0°→35.0°` 보정, 재빌드·175건 테스트 검증 및 푸시.

### Detailed Changes
- `/home/yongsoo/store_ws/config/control_params.yaml`, `reorient_motion.py`, `executor_node.py`, `mock_robot.py`: `pivot_tilt_deg` 기본값 `35.0°` 및 `effective_tilt` 하한 `20.0°` 적용.
- `/home/yongsoo/store_ws/src/store_robot_control/tests/`: `test_reorient_motion.py`, `test_executor_dry_run.py` 기대값 갱신.

### Verification & Review
- `colcon build --symlink-install --packages-select store_robot_control` 성공 (`1.32s`).
- `pytest -q src/store_robot_control/tests` → **175 passed**.


## 2026-11-27 15:02 — 실물 상품 2종 (`pepero_almond`, `postick`) 카탈로그 등록

- **Goal / Background**:
  - 실물 상품 단독 테스트를 위해 사용자 실측 치수(`빼빼로 아몬드맛 88×162 mm`, `포스틱 가로 150 mm 추정 × 세로 225~235 mm`)를 `products_catalog.yaml`의 `products:` 오버라이드 섹션에 선반영.
  - 앞서 검토한 `80° movej` 특이점 우회 변경은 실물 상품 기반 튜닝 전까지 안정 확인된 `35°`(`20f7c10`) 상태를 유지하기로 결정해 미커밋 변경을 원복.
- **Responsible Agent(s)**:
  - **Antigravity**: 실물 M0609 `ikin`/`fkin` 기구학 시뮬레이션(80° 2단계 `movej` vs 20° 소각도), 35° 원복, 상품 2종 파라미터 계산·등록 및 단위 테스트 보강.
  - **Codex**: 80° 2단계 `movej` 프로토타입 구현(184/184 테스트 통과)을 수행했으나 사용자 요청으로 35° 기준 원복.
- **Detailed Changes**:
  - `/home/yongsoo/store_ws/config/products_catalog.yaml`: `products:` 하위에 `pepero_almond` (`open_width_mm: 108.0`, `grasp_width_mm: 68.0`, `grasp_force_n: 20.0`, `grasp_depth_offset_mm: 15.0`) 및 `postick` (`open_width_mm: 110.0`, `grasp_width_mm: 20.0`, `grasp_force_n: 12.0`, `grasp_depth_offset_mm: 25.0`) 추가.
  - `/home/yongsoo/store_ws/src/store_robot_control/tests/test_grasp_planner.py`: `test_measured_product_overrides` 테스트 추가.
- **Verification & Review**:
  - `pytest -q src/store_robot_control/tests/test_grasp_planner.py` → **11 passed** (0.06s).
- **Follow-ups / Caveats**:
  - 포스틱 가로 폭이 150 mm(15 cm)라면 RG2 최대 벌림 폭(110 mm)을 초과하므로 눕힌 상태 가로 집기는 불가 — 세워 잡기(두께 방향) 또는 좁은 모서리/중앙 집기 검증 필요.
  - 두 상품의 바닥 기준 높이/두께(Depth, mm) 실측 후 `grasp_depth_offset_mm` 최종 보정 예정.


## 2026-11-27 15:15 — 빼빼로 1차 실물 파지 피드백 보정 (파지력·폭 완화 및 z_min_mm 조정)

- **Goal / Background**:
  - 실물 `pepero_almond` 1차 `pick_place` 테스트(`[275.7, -11.5, -12.0, 0.0] → [480.8, -76.9, -12.0, 0.0]`)가 깔끔하게 성공했으나, `20 N / 68 mm` 설정으로 종이상자가 구겨졌고 파지 시 Z축을 더 내려 안정적으로 감싸 잡을 필요가 있다는 실측 피드백을 반영.
- **Responsible Agent(s)**:
  - **Antigravity**: 실물 피드백 기반 파라미터 조정, 단위 테스트 갱신 및 검증.
- **Detailed Changes**:
  - `/home/yongsoo/store_ws/config/products_catalog.yaml`: `pepero_almond`의 `grasp_force_n`을 `20.0 → 10.0 N`으로 절반 완화하고, `grasp_width_mm`을 `68.0 → 78.0 mm`(실측 88 mm 대비 10 mm 여유)로 상향해 상자 구겨짐을 이중 방지.
  - `/home/yongsoo/store_ws/config/workspace_zones.yaml`: `safety.z_min_mm`을 `-13.785 → -18.5 mm`로 조정(실측 바닥 ~`-23.8 mm` 대비 안전 여유 약 `5.3 mm` 유지, `Z = -16~-17 mm` 하강 파지 허용).
  - `/home/yongsoo/store_ws/src/store_robot_control/tests/test_grasp_planner.py`: `pepero_almond` 기대값 갱신.
- **Verification & Review**:
  - `pytest -q src/store_robot_control/tests/test_grasp_planner.py src/store_robot_control/tests/test_safety_guard.py` 통과 확인.
- **Follow-ups / Caveats**:
  - `executor_node` 재기동 후 `Z = -16.0`~`-17.0 mm`로 빼빼로 2차 `pick_place` 및 `reorient` 실물 테스트 진행 예정.

- **보수 조정 (15:18)**: 사용자 안전 요청에 따라 `workspace_zones.yaml`의 `z_min_mm`을 `-18.5 → -16.5 mm`(바닥 대비 여유 약 `7.3 mm`)로 `+2.0 mm` 상향.


## 2026-11-27 15:29 — 빼빼로 2차 실물 파지 보정 (`5 N / 82 mm / z_min -18.0 mm`) 및 90° 세워 뒤집기 기구학 검증

- **Goal / Background**:
  - `pepero_almond` 2차 `pick_place`(`Z=-14.5 mm`, `10 N / 78 mm`) 성공 후, 힘을 더 빼고 Z를 조금 더 내리려는 피드백을 반영.
  - 동시에 `35° edge_pivot_flip`이 납작한 빼빼로 상자를 전혀 뒤집지 못한 원인을 분석하고, 실물 M0609 `ikin`/`fkin`으로 공중 저속 `movej` 측면 90° 세우기(`sol=2`, `J1=-46.6°, J5=+107~120°`, `Z=200→10 mm` 수직 하강)의 무특이점 도달성을 확인.
- **Responsible Agent(s)**:
  - **Antigravity**: 파라미터 2차 보정, 실물 M0609 측면 90° `ikin`/`fkin` 검증, 단위 테스트 확인.
- **Detailed Changes**:
  - `/home/yongsoo/store_ws/config/products_catalog.yaml`: `pepero_almond` `grasp_force_n` `10.0 → 5.0 N`, `grasp_width_mm` `78.0 → 82.0 mm`.
  - `/home/yongsoo/store_ws/config/workspace_zones.yaml`: `safety.z_min_mm` `-16.5 → -18.0 mm`.
  - `/home/yongsoo/store_ws/src/store_robot_control/tests/test_grasp_planner.py`: 기대값 갱신.
- **Verification & Review**:
  - `pytest -q src/store_robot_control/tests/test_grasp_planner.py src/store_robot_control/tests/test_safety_guard.py` 통과 확인.
- **Follow-ups / Caveats**:
  - 사용자와 새 뒤집기 전략(공중 저속 `movej` 90° 2단계 세워 뒤집기 vs 벽면/지그 보조 반전) 합의 후 구현 예정.

- **3차 파지력 보정 (15:42)**: `products_catalog.yaml`의 `pepero_almond`를 RG2 하드웨어 최소 힘 **`3.0 N`** + 폭 **`84.0 mm`**(88 mm 상자 대비 4 mm)로 추가 완화 (`test_grasp_planner.py` 11 passed).


## 2026-11-27 16:18 — 모드 A 세로 뒤집기 (`back_tilt_vertical_flip`) 구현 및 리뷰 보완

- **Goal / Background**:
  - 납작한 빼빼로 상자(`88 × 162 × 25 mm`)를 재파지 없이 1회 파지로 세로(길이방향) 180° 뒤집기 위해, 사용자 아이디어(손목 `J5`를 뒤로 눕혀 상단 끝을 잡고 들어 올린 뒤 반대쪽까지 조향해 전복)를 실물 M0609 기구학으로 검증하고 `back_tilt_vertical_flip` 모드로 구현.
  - 실물 M0609 `/dsr01` `ikin`/`fkin` 사전 검증 결과, `뒤로 -35° 파지 → 공중 +40° 경유(Z=220 mm) → 앞쪽 +75° sol=3 movej(총 110° 회전)` 조합이 4개 작업 위치 모두에서 `J3 ≤ 148.2°`, `J5 ∈ [+124.5°, -59.8°]`, `J4/J6 변동 < 2°`, 중간 `Z` 꺼짐 없이 통과함을 확인.
- **Responsible Agent(s)**:
  - **Antigravity**: 실물 M0609 `ikin`/`fkin` 전체 궤적·관절 여유 검증, 인터랙티브 시뮬레이션 작성, diff 감사 중 실기 `retreat` ZYZ 래핑 오탐 및 `ref_joints` 원거리 시작 엣지케이스 발견, Codex 리뷰 3건과 함께 총 5건 일괄 수정 및 최종 빌드/테스트 검증.
  - **Codex**: `back_tilt_vertical_flip` 초기 구현(`reorient_motion.py`, `dsr_interface.py`, `executor_node.py`, YAML 설정, 단위 테스트) 및 `codex exec review --uncommitted` 교차 코드 리뷰 수행.
- **Detailed Changes**:
  - `/home/yongsoo/store_ws/config/control_params.yaml`: `reorient:` 하위에 `back_tilt_deg: 35.0`, `vertical_mid_tilt_deg: 40.0`, `vertical_fwd_tilt_deg: 75.0`, `vertical_edge_offset_mm: 45.0`, `vertical_lift_delta_mm: 150.0`, `vertical_mid_z_delta_mm: 210.0`, `vertical_release_z_delta_mm: 95.0` 추가.
  - `/home/yongsoo/store_ws/config/products_catalog.yaml`: `products.pepero_almond`에 `reorient_mode: "back_tilt_vertical_flip"` 적용(기존 파지값 `108/84/3.0/15.0` 유지).
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: 순수 Python `plan_back_tilt_vertical_flip()` 추가 (9개 웨이포인트 계획 및 각도 합 `≥ 95°`·작업범위·J6 사전 검증).
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: `solve_continuous_ik()` 추가 (팔꿈치 분기 유지, 관절 변화량·한계 필터, `j6_limits` 사전 필터링, `fkin` 위치·자세 오차 교차 검증).
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: `_check_joint_path_fkin()` 및 `_execute_back_tilt_vertical_flip()` 추가. 실행 전 모든 IK(`ref`, `app`, `grp`, `mid`, `end`, `retreat`)와 모든 관절 경로 FK를 사전 검증하며, `approach_back` 수직 상승 후 실제 관절 → `j_app` 경로 FK도 실행 직전 검증. `retreat`도 관절 공간 `move_joint(j_retreat)`로 통합해 비하향 `ry≈-140°`에서의 ZYZ J6 오탐 원천 차단, 드라이런 시 시뮬레이션 TCP 자세 갱신.
  - `/home/yongsoo/store_ws/src/store_robot_control/tests/test_reorient_motion.py`, `test_executor_dry_run.py`, `test_grasp_planner.py`: `back_tilt_vertical_flip` 계획·실행·사전 차단·`approach_back` FK 차단·`j6_limits` 필터·파지 실패 시 관절 복귀 단위 테스트 추가.
- **Verification & Review**:
  - `codex exec review --uncommitted` 주요 지적 3건(P1 접근 경로 FK 검증 누락, P2 `solve_continuous_ik` J6 사전 필터, P2 드라이런 TCP 자세 갱신) + Antigravity 자체 감사 2건(`ref_joints` 원거리 시작 J1/J6 여유, `retreat` ZYZ 래핑 오탐 방지) 모두 수정 완료.
  - `colcon build --symlink-install --packages-select store_robot_control` 성공 및 `pytest -q src/store_robot_control/tests/` → **199 passed** (3.99s), `git diff --check` 통과.
- **Follow-ups / Caveats**:
  - 실물 M0609에서 `pepero_almond`로 `back_tilt_vertical_flip` 테스트 후 실제 뒤집힘 각도(`vertical_fwd_tilt_deg` / `vertical_release_z_delta_mm`) 및 앞쪽 공간 여유 확인 예정.


## 2026-11-27 16:51 — `back_tilt_vertical_flip`을 사용자 실측 `J4` 상하(Y–Z 롤) 방향으로 전환 및 2차 리뷰 보완

- **Goal / Background**:
  - 실물 테스트 시 `J5` 기반 틸트(X–Z 평면)가 사용자 시선에서 좌우로 눕는 문제를 확인. 사용자가 직접 교시로 찍은 자세(`[283.7, -90.1, -19.0], ZYZ=[89.5, -152.3, 89.0], J4=-31.2°, J5=+60.3°`)를 기준으로 **사용자 시선 상하(Y–Z 평면, `J4` 롤)** 로 전환하고, 기존 픽앤플레이스 타겟 좌표(`[275.7, -11.5]`) 제자리에서 반전하도록 수정.
  - 실물 M0609 `/dsr01` `ikin`/`fkin`으로 4개 위치(`Target A [275.7,-11.5,-15.5]`, `Dest B [480.8,-76.9,-15.5]` 포함) 전체 검증 결과: `J5 ∈ [+62.8°, +119.0°]`(특이점 0%), `J3 ≤ 128.8°`, `J6 ∈ [-72.8°, +38.2°]`, 제자리 X/Y 유지 확인.
- **Responsible Agent(s)**:
  - **Antigravity**: 사용자 실측 자세 분석, 실물 M0609 `J4` Y–Z 롤 4점 기구학 검증, `reorient_motion.py`/`executor_node.py`/`control_params.yaml` 전환, Codex 2차 리뷰 지적 2건 반영 및 전체 테스트 검증.
  - **Codex**: `codex exec review --uncommitted` 2차 교차 리뷰 수행 (P1: `rz` 회전 시 끝단 오프셋 불일치 및 범위 가드, P2: `mock_robot.py` `back_tilt_vertical_flip` 미지원 지적).
- **Detailed Changes**:
  - `/home/yongsoo/store_ws/config/control_params.yaml`: `back_tilt_deg: 30.0`, `vertical_mid_tilt_deg: 0.0`, `vertical_fwd_tilt_deg: 85.0` (총 `115°`), `vertical_edge_offset_mm: 65.0`, `vertical_lift_delta_mm: 165.0`, `vertical_mid_z_delta_mm: 215.0`, `vertical_release_z_delta_mm: 90.0`.
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_back_tilt_vertical_flip()`을 `-Y` 상단 끝 파지 + 제자리 `[x, y]` 반전으로 전환, `|rz| <= 30°` 가드 및 `rz` 회전 반영 오프셋 적용.
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`: Y–Z 롤 ZYZ 자세(`[90, -(180-back), 90+rz]` → `[0, 180, rz]` → `[90, 180-fwd, 90+rz]`) 및 `roll_deltas` 연속 IK 적용.
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/mock_robot.py`: `back_tilt_vertical_flip` 허용 및 웨이포인트 계획 연결.
  - `/home/yongsoo/store_ws/src/store_robot_control/tests/test_reorient_motion.py`, `test_executor_dry_run.py`: `J4` Y–Z 롤 기대값, `rz` 회전/거부 테스트, `MockRobot` 회귀 테스트 추가.
- **Verification & Review**:
  - `colcon build --symlink-install --packages-select store_robot_control` 성공 및 `pytest -q src/store_robot_control/tests/` 전체 통과.
- **Follow-ups / Caveats**:
  - 실물 M0609에서 자동 모드(`robot_mode=1`) 전환 후 `pepero_almond`로 `[275.7, -11.5, -15.5, 0.0]` 실기 테스트 예정.


## 2026-11-27 17:12 — `3.0 N` 중력 힌지 + `+Y` 지렛대 넘기기(`vertical_sweep_y_mm`) 적용

- **Goal / Background**:
  - 실물 테스트 영상(`IMG_4448.MOV`) 분석 결과, `3.0 N / 84 mm`의 가벼운 파지 덕분에 들어 올릴 때 중력만으로 빼빼로가 구겨짐 없이 `~70°(\)`까지 자연스럽게 늘어졌으나, `Y` 고정 상태로 `Z=75 mm` 하강 시 하단 모서리가 `+Y` 바닥에 먼저 걸려 90°를 못 넘었던 문제를 해결.
  - 사용자가 선호한 **중력 힌지 회전(`3.0 N`)** 을 유지하면서 `mid_upright`(`Z=130 mm` 바닥 살포시 착지) → `flip_over`(`+Y`로 `+145 mm` 넘기며 `Z=75 mm`, `+65°` 롤)로 바닥 모서리를 지렛대 축으로 삼아 `90°→135°(/)`를 강제 통과하도록 개선.
- **Responsible Agent(s)**:
  - **Antigravity**: 실물 영상 프레임 추출·분석, 중력 힌지 + `+Y` 넘기기 비교 시뮬레이션 작성, `control_params.yaml`·`reorient_motion.py`·`executor_node.py`·`mock_robot.py`·테스트 반영 및 빌드/검증.
- **Detailed Changes**:
  - `/home/yongsoo/store_ws/config/control_params.yaml`: `vertical_sweep_y_mm: 145.0` 추가, `vertical_mid_z_delta_mm: 215.0 → 145.0` (`Z=130 mm` 착지), `vertical_fwd_tilt_deg: 85.0 → 65.0` (완만한 `+65°` 롤).
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_back_tilt_vertical_flip()`에 `sweep_y_mm` 추가(`rz` 회전 반영, 작업영역 경계·음수 검증) 및 `mid_z` 하한 `130.0 mm` 조정.
  - `/home/yongsoo/store_ws/src/store_robot_control/store_robot_control/nodes/executor_node.py`, `mock_robot.py`: `vertical_sweep_y_mm` 파라미터 전달 연동.
  - `/home/yongsoo/store_ws/src/store_robot_control/tests/test_reorient_motion.py`, `test_executor_dry_run.py`: `sweep_y_mm` 기대값, `rz=25°` 회전 넘기기 좌표, `y_max` 경계/음수 거부 테스트 추가.
- **Verification & Review**:
  - `colcon build --symlink-install --packages-select store_robot_control` 성공 및 전체 `pytest` 통과 (`git diff --check` clean).
- **Follow-ups / Caveats**:
  - 실물 M0609에서 비상정지 해제 → `roboton` 기동 → `pepero_almond` 실물 테스트 후, 바닥 미끄러짐 여부와 `+Y` 넘김 거리(`vertical_sweep_y_mm`) 미세 조정 예정.

- **방향 A 부호 수정 & 놓는 높이 상향 (17:30)**: 방향 A(`-Y` 파지 → `+Y` 넘기기)와 일치하도록 `fwd_orientation`을 `/` 방향(`ry = -(180 - 45) = -135°`)으로 수정하고, 바닥 간섭 방지를 위해 `vertical_release_z_delta_mm`을 `90 → 115 mm` (`Z=100 mm`)로 상향.

- **1차 실물 180° 반전 성공(`IMG_4450.MOV`) 후 대칭·이동거리 미세 튜닝 (17:36)**: `pepero_almond.grasp_width_mm` `84.0 → 85.5 mm` (3.0 N 최소 힘 유지하며 저마찰 힌지 강화), `vertical_edge_offset_mm` `65 → 55 mm` (양옆 대칭 파지), `vertical_sweep_y_mm` `145 → 105 mm`, `vertical_fwd_tilt_deg` `45 → 35°`.

- **Y축 직선 `movel` + Z축 순응제어 시험 적용 (17:43)**: `dsr_interface.py`에 `enable_z_compliance(stiffness_z=500.0)` / `release_compliance()` 추가, `executor_node.py`의 `mid_upright` 및 `flip_over`를 관절 호(`movej`)에서 Y축 직선 `movel` + `try...finally` Z축 순응제어로 전환해 X축 우회 제거.

- **성공 확정 `IMG_4450.MOV` 조합 복원 (17:46)**: `movel` 직선 하강 및 축소 이동(`55/105/35°/85.5mm`) 시 90° 직립 상자를 쳐서 원복되는 현상 확인 → 180° 반전이 실물 검증된 `movej` 곡선 + `84.0 mm / 3.0 N` + `edge=65, sweep=145, fwd=45°` 조합으로 즉시 복원.

- **제자리 착지 및 `+Y` 침범 방지 오프셋(`vertical_mid_y_offset_mm: -135.0`) 추가 (17:49)**: `mid_upright`를 `-Y`로 `-135 mm`(`Y=-146.5`) 당긴 뒤 동일하게 `+145 mm` 넘겨 최대 `Y=-1.5 mm`로 노트북 작업공간(`+Y`) 침범을 없애고 원래 타겟 위치(`-11.5 mm`) 제자리에 180° 착지하도록 개선.

- **파지점 `-Y` 추가 이동 (17:51)**: `control_params.yaml`의 `vertical_edge_offset_mm`을 `65.0 → 74.0 mm` (`Y=-85.5 mm`, 상자 끝 81 mm 대비 7 mm 안쪽)로 이동해 중력 회전 지렛대 극대화.

- **파지점 `65.0 mm` 복원 (17:53)**: `74.0 mm` 끝 모서리 접합부는 강성이 높아 힌지 회전이 막힘을 확인 → 부드럽게 회전했던 `65.0 mm`로 복원 (`vertical_mid_y_offset_mm: -135.0` 제자리 착지는 유지).

- **제자리 당김 반동 제거 (`vertical_mid_y_offset_mm: -135.0 → -60.0`) (17:57)**: `-135 mm` 적용 시 `lift_back(-76.5) → mid(-146.5)` 구간에서 `-Y` 역방향 반동 및 `J1` 변화로 원복 현상 발생 → `lift_back`과 거의 일치하는 `-60.0 mm` (`Y_mid=-71.5`, `Y_end=+73.5`)로 조정해 반동 제거 및 `+Y` 침범 60 mm 축소.

- **`vertical_fwd_tilt_deg: 45 → 75°` 상향으로 90° 통과 여유 극대화 (18:00)**: `mid_y_offset=-60 mm`에서 `45°`는 90° 경계에 아슬아슬하게 걸림 → 실물 M0609 `ikin`/`fkin` 검증(`J3=97.5°, J5=+112.6°, minZ=87.3 mm`) 후 `75°` (`sweep_y=150 mm`, `release_z_delta=120 mm`)로 상향 및 `roll_deltas` J1/J6 여유 확대.

- **순응제어 바닥 접촉 감지(`touchdown_compliant`) + `75° movej` 하이브리드 구현 (18:04)**: `dsr_interface.py`에 `get_tool_force_z()` 및 `descend_until_z_contact()`(`400 N/m` Z축 순응제어 + `8 mm` 스텝 `movel` + 공중 영점 대비 `ΔFz >= 4.0 N` 접촉 시 즉시 정지, `finally` 자동 해제) 추가, `executor_node.py`의 `mid_upright`(`Z≈155 mm`) 이후 `touchdown_compliant` → `flip_over`(`75° movej`) 하이브리드 시퀀스 구성.

- **`IMG_4450.MOV` 성공 확정 버전 완전 복원 (18:09)**: 55 g 경량 종이상자 + 3 N 힌지 조합에서는 내장 토크 센서 반력 감지(`touchdown_compliant`)가 실효성이 낮음을 실물 검증 후 제거. `IMG_4450.MOV`에서 180° 반전이 검증된 순수 10단계 `movej` 버전(`edge=65, mid_y_offset=0.0, sweep_y=145, back=30°, fwd=45°, lift_delta=165, mid_z_delta=145, release_z_delta=115, 84mm/3N`)으로 완전 복원 (`dsr_interface.py` 순응/힘 헬퍼는 향후 중량 상품용으로 보존).

- **제자리 2구간 반전 (`pre_swing_air` + `flip_pass`) 시험 적용 (18:16)**: `vertical_mid_y_offset_mm < -50.0`일 때 허공 `+Y` 예열(`pre_swing_air`, `Y=-11.5, Z=160`) → `-Y` 이동(`mid_upright`, `Y=-146.5, Z=135`) → 1구간 바닥 짚기(`flip_pass`, `Y=-85, Z=118, -18°`) → 2구간 살짝 누르며 넘기기(`flip_over`, `Y=+5, Z=102, -55°`) 시퀀스 추가 (`mid_y_offset=0.0` 시 기존 10단계 성공 버전 그대로 유지).

- **옵션 A 제자리 복귀 반전 (`vertical_recenter_carry: true`) 적용 (18:27)**: 성공 확정 `IMG_4450.MOV` 반전 직후 `lay_flat`(Z=20 mm 완전히 눕히기) → `carry_back`(Z=45 mm 살짝 들어 `-Y` 95 mm 복귀) → `place_back`(Z=10 mm 내려놓기) → `release_flip` 시퀀스 추가 및 실물 M0609 `ikin`/`fkin` 사전 검증 완료.

---

## 2026-11-27 18:55 — 옵션 A(`vertical_recenter_carry`) 실기 실패로 `IMG_4450.MOV` 10단계 성공 버전 즉시 원복

- **목표 / 배경**: 실기 테스트 결과 옵션 A(`lay_flat → carry_back → place_back`)가 `3.0 N` 약한 파지 및 기울어진 그리퍼 자세에서 제자리 복귀가 전혀 되지 않아, 사용자 요청에 따라 100% 검증된 `IMG_4450.MOV` 10단계 `back_tilt_vertical_flip` 버전으로 즉시 원복.
- **담당 에이전트**: Antigravity 직접 수정 및 문법 검증.
- **세부 변경 내역**:
  - `config/control_params.yaml`: `vertical_recenter_carry: true` 항목 제거 (`back_tilt_deg: 30.0`, `vertical_mid_tilt_deg: 0.0`, `vertical_fwd_tilt_deg: 45.0`, `vertical_edge_offset_mm: 65.0`, `vertical_mid_y_offset_mm: 0.0`, `vertical_sweep_y_mm: 145.0`, `vertical_lift_delta_mm: 165.0`, `vertical_mid_z_delta_mm: 145.0`, `vertical_release_z_delta_mm: 115.0` 유지).
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`: `self._vertical_recenter_carry` 및 13단계 `lay_flat / carry_back / place_back` 분기 제거, 순수 10단계 `movej` 성공 시퀀스(`open_gripper → approach → descend → grip → lift → mid_upright → flip_over → release_flip → lift_after_flip → retreat`)로 복원.
- **검증**: `python3 -m py_compile` + YAML 로드 확인.
- **후속 조치**: 필요 시 뒤집은 직후 수직 재파지(`pick_place` 조합, 옵션 B) 또는 고정 Flip Zone 설계 검토.

---

## 2026-11-27 19:03 — 뒤집기 완료 후 수직 재파지 원위치 재정렬(`vertical_recenter_after_flip`, 옵션 B) 추가

- **목표 / 배경**: 100% 성공한 `IMG_4450.MOV` 10단계 뒤집기 동작은 그대로 유지하면서, 뒤집혀 `+Y` 쪽에 착지한 상품을 수직 탑다운(`pick_place` 방식)으로 다시 집어 원래 `target_pose` 제자리로 복귀·정렬시키는 기능 추가.
- **담당 에이전트**: Antigravity 구현 (`IMG_4450.MOV` 실제 프레임 `v2_2_52.jpg`/`v2_3_68.jpg` 착지 오프셋 `ΔY ≈ +140 mm` 실측 반영).
- **세부 변경 내역**:
  - `config/control_params.yaml`: `vertical_recenter_after_flip: true`, `vertical_recenter_y_offset_mm: 140.0` 추가.
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`:
    - `1~9단계`는 기존 `IMG_4450.MOV` 성공 시퀀스 100% 유지(`open_gripper → ... → release_flip → lift_after_flip`).
    - `10~17단계` 추가: 착지 예상 위치(`target + offset_y` along rotated `+Y`, `rz=0` 기준 `[275.7, +128.5, -15.5]`)로 수직 접근 → 하강 → `84 mm / 3.0 N` 수직 파지(`_verify_grasp` 포함, 평행 핑거 닫힘으로 약간의 틀어짐 자동 정렬) → `Z≈130 mm` 상승 → 원위치 수평 이동 → 하강 → 놓기 → `retreat`.
    - 모든 착지/원위치 웨이포인트를 모션 시작 전에 `_checked_pose`로 사전 검증.
- **검증**: `python3 -m py_compile` + YAML 로드 확인.
- **후속 조치**: 실기 테스트 후 착지 오차에 따라 `vertical_recenter_y_offset_mm` 미세 조정 (`±10 mm` 단위).

---

## 2026-11-27 19:14 — `vertical_recenter_after_flip` 제거 및 `IMG_4450.MOV` 10단계 뒤집기 전용 버전 확정

- **목표 / 배경**: 고정 오프셋(`+140 mm`) 수직 재파지는 뒤집기 착지 위치 변동으로 미파지가 발생하며, 실제 Vision+LLM 파이프라인에서는 `reorient → scan_pose(비전 재스캔) → pick_place` 폐루프로 재정렬하는 것이 정석이므로 사용자 요청에 따라 10단계 뒤집기 전용 버전으로 최종 복원.
- **담당 에이전트**: Antigravity 직접 정리 및 문법 확인.
- **세부 변경 내역**:
  - `config/control_params.yaml`: `vertical_recenter_after_flip`, `vertical_recenter_y_offset_mm` 제거.
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`: 17단계 고정 재파지 분기 제거, 순수 10단계 `IMG_4450.MOV` 성공 시퀀스로 복원.
- **검증**: `python3 -m py_compile` 통과.

---

## 2026-11-27 19:28 — 상품별 `vertical_edge_offset_mm` 지원, 단위 테스트 212개 동기화, Codex 2회 리뷰 반영 및 커밋/푸시

- **목표 / 배경**: 실기 100% 성공한 빼빼로 아몬드맛 `back_tilt_vertical_flip` 10단계 버전을 안전하게 백업하고, 추후 다른 길이의 종이상자 상품이 추가되어도 상품별 편심 거리(`vertical_edge_offset_mm`)를 설정할 수 있게 확장하며, 전체 단위 테스트 및 크로스모델 코드 리뷰를 통과시켜 `main`에 커밋·푸시함.
- **담당 에이전트**:
  - **Codex (`codex exec -s workspace-write`)**: `GraspPlan`/`GraspPlanner`의 `vertical_edge_offset_mm` 옵션 추가 및 검증, `executor_node.py`/`mock_robot.py`/`products_catalog.yaml` 연결, 미사용 실험용 순응제어 메서드(`dsr_interface.py`) 정리, 단위 테스트 211개 동기화.
  - **Codex (`codex exec review --uncommitted` 2회)**: 1차 리뷰 `[P1]`(회전 상품 `rz`에 대한 ZYZ 기울기 축 불일치) · `[P2]`(`MockRobot`의 `apply_grasp_depth_offset` 누락) 및 2차 리뷰 `[P1]`(`approach_back` FK 검사 직후 중단 상태 재확인 누락) 발견.
  - **Antigravity**: Codex 리뷰 지적 3건 모두 수학적 검증(`zyz(90 - rz, -(180 - θ), 90)` ↔ `[0, 180, rz]` 연속성 오차 `1.7e-16`) 후 직접 수정, 회전 테스트 및 `MockRobot` 깊이 보정 테스트 추가, `colcon build` + `pytest` 212/212 최종 검증 및 Git 커밋·푸시.
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `pepero_almond`에 `vertical_edge_offset_mm: 65.0` 명시.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`: `GraspPlan.vertical_edge_offset_mm: float | None = None` 필드 및 유한 양수 검증 추가.
  - `src/store_robot_control/store_robot_control/core/reorient_motion.py`: Doosan 하향 ZYZ `[0, 180, rz]` 공구 Y축(`+sin(rz), +cos(rz)`) 기준으로 `-Y` 끝단 파지·`+Y` 넘기기 오프셋 부호 정렬.
  - `src/store_robot_control/store_robot_control/hardware/dsr_interface.py`: 미사용 실험용 순응제어 함수 4종 제거 (`solve_continuous_ik` 유지).
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`: 상품별 `vertical_edge_offset_mm` 오버라이드 적용, ZYZ 기울기 자세를 `[90 - rz, -(180 - tilt), 90]`으로 수정(`rz=0`일 때 `[90, -150/-135, 90]` 완전 동일), `approach_back()` FK 검사 직후 `_check_interruption(handle)` 재확인 추가.
  - `src/store_robot_control/store_robot_control/nodes/mock_robot.py`: `apply_grasp_depth_offset` 파라미터·보정 로직을 `ExecutorNode`와 동일하게 적용 및 상품별 `vertical_edge_offset_mm` 반영.
  - `src/store_robot_control/tests/test_grasp_planner.py`, `test_reorient_motion.py`, `test_executor_dry_run.py`: `IMG_4450.MOV` 실기 검증 기준(10단계, `ry=-135°`, `release_z_delta=115 mm`), 상품별 오프셋, `rz=25°` ZYZ 축 회전, `MockRobot` 깊이 보정 일관성 테스트 반영.
- **검증 결과**: `colcon build --symlink-install --packages-select store_robot_control` 성공, `pytest -q src/store_robot_control/tests` **212 passed**, Codex 리뷰 지적 전건 해결.
- **후속 조치**: 나머지 실물 품목 2종(`포스틱`, `알로에농장`) 파지/재정렬 실측 및 비전·LLM 팀원 신규 싱크 항목 협의.

---

## 2026-11-27 19:52 — 확정 YOLO 클래스명 별칭 호환(`aliases`) 추가 및 포스틱 뒤집기 준비

- **목표 / 배경**: 팀 확정 YOLO 클래스명(`pepero_almond_front`, `postick_front`, `drink_front`, `ciga`)을 로봇 카탈로그에서 중복 없이 자동 호환하도록 `aliases` 매핑을 추가하고, `postick`의 실기 파지·뒤집기 테스트 초기값을 준비함.
- **담당 에이전트**: Antigravity 구현 및 단위 테스트 검증.
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `drink` 항목(`in_place_yaw_180`, `95/45 mm`, `30 N`) 추가, `postick` 초기 테스트값(`back_tilt_vertical_flip`, `vertical_edge_offset_mm: 75.0`, `110/15 mm`, `12 N`) 설정, `aliases` 섹션(`pepero_almond_front → pepero_almond`, `postick_front → postick`, `drink_front → drink`, `ciga → cigarette_mock`) 추가.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`: `self.aliases` 해석 로직 추가.
  - `src/store_robot_control/tests/test_grasp_planner.py`: YOLO 별칭 호환 및 `postick`/`drink` 검증 테스트 갱신.
- **검증**: `pytest -q src/store_robot_control/tests/test_grasp_planner.py` 통과.

---

## 2026-11-27 20:03 — 포스틱(`postick_front`) 왼쪽 하단 빈 실링 꼭짓점 뒤집기(`vertical_corner_x_offset_mm`) 추가

- **목표 / 배경**: 포스틱(세로 `230 mm` 앞뒤 `±X`, 가로 `150 mm` 좌우 `±Y`, 정면 세로 배치)은 몸통을 집으면 질소 압력/과자 파손 우려가 있으므로, 내용물이 없는 왼쪽 하단 빈 실링 꼭짓점(`+X 90 mm, -Y 55 mm`)을 얇게 핀치(`open 50 mm → grasp 0 mm / 18 N`)해 오른쪽(`+Y`)으로 책장 넘기듯 뒤집는 설정 추가.
- **담당 에이전트**: Antigravity 구현 (`pepero_almond`는 `corner_x_offset_mm=0.0`으로 100% 기존 동작 유지).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_edge_offset_mm: 55.0`, `vertical_corner_x_offset_mm: 90.0`, `open_width_mm: 50.0`, `grasp_width_mm: 0.0`, `grasp_force_n: 18.0` 적용.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`: `GraspPlan.vertical_corner_x_offset_mm: float = 0.0` 및 유한 실수 검증 추가.
  - `src/store_robot_control/store_robot_control/core/reorient_motion.py`: `plan_back_tilt_vertical_flip()`에 `corner_x_offset_mm: float = 0.0` 추가(공구 X축 기준으로 전체 뒤집기 평면 `cx, cy` 이동, `0.0`일 때 기존과 완전 동일).
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`, `mock_robot.py`: `grasp.vertical_corner_x_offset_mm` 전달.
- **검증**: `py_compile` + `plan_back_tilt_vertical_flip` 실기 타겟 `[275.7, -11.5, -15.5, 0.0]` 계획 좌표 확인.

---

## 2026-11-27 20:12 — 포스틱 1차 실기 피드백 반영(바깥 꼭짓점 보정, 반대 J4 `\` 기울기, J6 90° 회전, 좁은 집게)

- **목표 / 배경**: 포스틱 1차 테스트에서 몸통 안쪽을 누르며 봉지가 터질 뻔한 문제를 해결하기 위해, 파지점을 바깥쪽 왼쪽 하단 귀퉁이(`+X +15 mm`, `-Y -30 mm`)로 이동하고 `Z=+10 mm`로 높이며, J4 반대 방향(`\` 기울기) + J6 90° 회전 + 좁은 개방(`25 mm`)을 적용함.
- **담당 에이전트**: Antigravity 구현 (`pepero_almond` 기본값 `vertical_tilt_sign=-1, vertical_j6_offset_deg=0.0` 완전 유지).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`을 `vertical_corner_x_offset_mm: 105.0`, `vertical_edge_offset_mm: 85.0`, `vertical_tilt_sign: 1`, `vertical_j6_offset_deg: 90.0`, `open_width_mm: 25.0`, `grasp_width_mm: 0.0`, `grasp_force_n: 18.0`, `grasp_depth_offset_mm: 15.0`으로 업데이트.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`: `vertical_tilt_sign` (`-1|1`) 및 `vertical_j6_offset_deg` (`[-120, 120]`) 필드·검증 추가.
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`: `sign * (180 - tilt)` 및 `90 + j6_off` ZYZ 자세 계산 적용 (`mid_orientation`과 연속성 유지).
- **검증**: `py_compile` + 계획 좌표/자세 출력 확인.

---

## 2026-11-27 20:16 — 포스틱 2차 실기 피드백 반영(`-X` 10 mm 보정, 상승 높이 `+40 mm`, 넘길 때 기존 J4 부호 `/` 유지)

- **목표 / 배경**: 포스틱 2차 테스트 피드백에 맞춰 파지 X를 `-10 mm` 안쪽으로 당기고(`380.7 → 370.7 mm`), 들어 올리는 높이를 `+40 mm` 높이며(`lift 215 mm / mid 195 mm`), 잡을 때는 `\`(`+1`, `ry=+150°`)·넘길 때는 기존 `/` 부호(`-1`, `ry=-135°`)로 분리 설정.
- **담당 에이전트**: Antigravity 구현 (`pepero_almond` 100% 무영향).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_corner_x_offset_mm: 95.0`, `vertical_fwd_tilt_sign: -1`, `vertical_lift_delta_mm: 205.0`, `vertical_mid_z_delta_mm: 185.0` 적용.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`, `nodes/executor_node.py`, `nodes/mock_robot.py`: 상품별 `vertical_fwd_tilt_sign`, `vertical_lift_delta_mm`, `vertical_mid_z_delta_mm` 오버라이드 지원 추가.
- **검증**: `py_compile` + 계획 좌표/자세 검증.

---

## 2026-11-27 20:22 — 포스틱 ㄷ자 옆면 슬라이드 진입(`vertical_slide_in_y_mm`) 및 `-X` 15 mm 추가 보정

- **목표 / 배경**: 포스틱 봉지 끝부분이 바닥에서 약 3.5 cm(`Z ≈ +11 mm`) 떠 있어 수직 하강 시 끝단을 눌러 밀어내는 현상을 해결하기 위해, 봉지 바깥쪽(`-Y 45 mm`) 빈 공간에서 먼저 하강한 뒤 `+Y` 수평 슬라이드로 끝단을 감싸 잡는 ㄷ자 진입(`vertical_slide_in_y_mm: 45.0`)과 `-X` 추가 보정(`95 → 80 mm`)을 적용함.
- **담당 에이전트**: Antigravity 구현 (`pepero_almond`는 `vertical_slide_in_y_mm=0.0`으로 기존 10단계 완전 유지).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`을 `vertical_corner_x_offset_mm: 80.0` (`X=355.7 mm`), `vertical_slide_in_y_mm: 45.0` (`Y: -141.5 → -96.5 mm`), `open_width_mm: 35.0`으로 업데이트.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`: `vertical_slide_in_y_mm: float = 0.0` 추가 및 검증.
  - `src/store_robot_control/store_robot_control/nodes/executor_node.py`: `slide_in > 0`일 때 11단계 ㄷ자 진입(`approach_back[Y_outside, Z_lift] → descend_outside[Y_outside, Z_grasp] → slide_in_grasp[Y_grasp, Z_grasp] → grasp → lift_back[Y_grasp, Z_lift] → ...`) 분기 추가.
- **검증**: `py_compile` + ㄷ자 진입 좌표 확인.

---

## 2026-11-27 20:24 — 포스틱 파지 시 J4 기울기 강화(`vertical_back_tilt_deg: 55.0`)

- **목표 / 배경**: ㄷ자 옆면 슬라이드 진입 시 기존 `30°` 기울기로는 수직에 가까워 떠 있는 봉지 끝단을 옆에서 감싸기 어려우므로, `postick`의 파지 기울기를 `55°`(`ry = +125°`)로 크게 눕혀 수평에 가깝게 진입하도록 보강함.
- **담당 에이전트**: Antigravity 구현 (`pepero_almond`는 `30.0°` 그대로 유지).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick.vertical_back_tilt_deg: 55.0` 추가.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`, `reorient_motion.py`, `nodes/executor_node.py`, `nodes/mock_robot.py`: 상품별 `vertical_back_tilt_deg` 오버라이드 지원, `back_tilt_deg` 허용 상한 `55 → 75°` 및 IK `J4` 허용 변화량 `65 → 85°` 확장.
- **검증**: `py_compile` + 계획 검증 완료.

---

## 2026-11-27 20:25 — 포스틱 파지 J4 기울기 `40.0°`로 하향 조정 (입고상자 간섭 방지)

- **목표 / 배경**: `55°` 기울기에서 `-Y` 쪽 노란색 입고상자와 로봇 손목/팔 간섭 우려가 있어 사용자 요청에 따라 `40.0°`(`ry = +140°`)로 안전 하향 조정.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick.vertical_back_tilt_deg: 55.0 → 40.0`.

---

## 2026-11-27 20:32 — 포스틱 ㄷ자 진입 시 봉지 밀림 방지를 위해 `open_width_mm` `35 → 50 mm` 확대

- **목표 / 배경**: `40°` 기울어진 상태에서 `open_width_mm=35 mm`는 상하 집게 틈이 좁아 슬라이드 진입 시 봉지 끝단이 밀리므로 `50.0 mm`로 1.5 cm 더 넓혀 넉넉하게 감싸 잡도록 조정.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick.open_width_mm: 35.0 → 50.0`.

---

## 2026-11-27 20:39 — 포스틱 J6 오프셋 `90° → 0°` 변경 (집게 날을 봉지 모서리 선과 평행 정렬)

- **목표 / 배경**: 기울기 `40°`는 유지하면서 두 집게가 봉지 왼쪽 금색 실링 모서리선(`±X`)과 평행하게 맞물려 잡도록 `vertical_j6_offset_deg`를 `90.0° → 0.0°`로 전환 (`toolX = [-1, 0, 0]`으로 두 손가락 높이가 동일해져 아래쪽 손가락 테이블 충돌도 원천 해소).
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick.vertical_j6_offset_deg: 90.0 → 0.0`.

---

## 2026-11-27 20:43 — 포스틱 앞쪽(`+X`) 평행 ㄷ자 진입(`vertical_entry_axis: "x"`, 정사영 평행) 적용

- **목표 / 배경**: 위에서 본 정사영 기준으로 그리퍼가 포스틱 왼쪽 금색 실링 끝단(`±X`)과 수직이 아닌 **평행(수평)**으로 만나도록, 진입 축을 `Y–Z`에서 `X–Z`(`vertical_entry_axis: "x"`, `[0, -140, 0]`)로 전환. 앞쪽 `+X` 빈 공간에서 하강 후 `-X`로 `40 mm` 수평 슬라이드해 위/아래 두 손가락이 끝단을 감싸 잡음(노란 입고상자 간섭 0%, `J4 ≈ -6°`, 실기 M0609 IK 전 단계 통과 검증 완료).
- **담당 에이전트**: Antigravity 구현 및 실기 IK 검증 (`pepero_almond`는 기본값 `vertical_entry_axis="y"`로 100% 무영향).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_entry_axis: "x"`, `vertical_slide_in_y_mm: 40.0`, `vertical_lift_delta_mm: 125.0` (`Z_app=160 mm`, `J5 ≈ 128°`), `vertical_mid_z_delta_mm: 185.0` (`Z_mid=215 mm`) 적용.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`, `nodes/executor_node.py`: `vertical_entry_axis` (`"x"|"y"`) 지원 추가.
- **검증**: `py_compile` + 실기 M0609 `ikin` 7단계 연속 관절해 검증.

---

## 2026-11-27 20:52 — 포스틱 `-X` 가로 뒤집기(진입 `[0, -140, 0]` → 반대 각도 `[0, +150, 0]` 넘기기) 적용

- **목표 / 배경**: 포스틱은 `±Y`(230 mm) 세로 방향보다 `±X`(150 mm) 가로 방향이 짧으므로, 앞쪽 `+X`에서 `[0, -140, 0]`으로 진입해 파지한 뒤 같은 `Y=-96.5` 선상에서 `-X`로 `140 mm`(`355.7 → 275.7 → 215.7 mm`) 이동하며 반대 각도 `[0, +150, 0]`으로 넘기는 가로 뒤집기 적용 (실기 M0609 IK/FK 단조 하강·`X > 212 mm`·노란 상자 비간섭 검증 완료).
- **담당 에이전트**: Antigravity 구현 및 실기 IK/FK 검증 (`pepero_almond` 100% 무영향).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_fwd_tilt_deg: 30.0`, `vertical_sweep_mm: 60.0`, `vertical_fwd_tilt_sign: 1` 적용.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`, `reorient_motion.py`, `nodes/executor_node.py`, `nodes/mock_robot.py`: `entry_axis="x"`일 때 `-X` 방향 가로 스윕(`355.7 → 275.7 → 215.7`) 및 반대 X-Z 기울기(`[0, +150, 0]`) 지원 추가.
- **검증**: `py_compile` + 실기 M0609 IK/FK 검증 통과.

---

## 2026-11-27 20:59 — 포스틱 영상(`IMG_4452.MOV`) 분석 반영: `+X` 전방 눕히기 반전(`255.7 → 430.7 mm`) 적용

- **목표 / 배경**: `IMG_4452.MOV` 프레임 확인 결과, 앞쪽 `+X` ㄷ자 진입 파지(`v3_3.jpg`) 및 공중 들기(`v3_4.jpg`)는 완벽 성공했으며, `v3_4.jpg`에서 포스틱 앞면("포스틱")이 사용자 쪽(`+X`)을 향하고 있음을 확인. 기존 `-X` 넘기기는 로봇 베이스 회피로 `J1`이 `-72.7°` 크게 비틀어져 부자연스러웠으므로, 높이(`Z=255 mm`) 든 뒤 앞쪽 빈 공간(`+X: 255.7 → 430.7 mm`, `Z: 255 → 95 mm`)으로 직선처럼 부드럽게 눕혀 앞면이 바닥으로 가고 뒷면이 위로 오게 반전하도록 개선 (실기 M0609 IK/FK 검증: `ΔJ1=12.8°`, `J4≈-6°`, `J6≈-11°`, 비틀림 0).
- **담당 에이전트**: Antigravity 영상 분석, 실기 IK/FK 검증 및 구현 (`pepero_almond` 100% 무영향).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_sweep_mm: 175.0`, `vertical_fwd_tilt_deg: 40.0`, `vertical_fwd_tilt_sign: -1`, `vertical_mid_z_delta_mm: 225.0`, `vertical_release_z_delta_mm: 65.0` 적용.
  - `src/store_robot_control/store_robot_control/core/grasp_planner.py`, `reorient_motion.py`, `nodes/executor_node.py`, `nodes/mock_robot.py`: `vertical_release_z_delta_mm` 오버라이드 및 `entry_axis="x"`의 `+X` 전방 눕히기 경로 추가.
- **검증**: `py_compile` + 실기 M0609 IK/FK 검증 통과.

---

## 2026-11-27 21:02 — 포스틱 높이 상향(`Z=315 mm`) + J4 `-Y` 방향 회전 뒤집기(`Y: +80 → -140 mm, ry=-120°`) 적용

- **목표 / 배경**: 포스틱 대각선 길이(`~275 mm`) 대비 `Z=255 mm` 높이가 부족해 뒤집히지 않았으므로, 사용자 제안대로 `Z=315 mm`까지 충분히 높이 들어 올린(`Y=+80 mm`) 뒤 `J4`를 `-Y` 방향(`0° → -51.6°`, `ry=-120°`, `Y: +80 → -140 mm`)으로 회전시켜 뒤집도록 변경 (실기 M0609 IK/FK 검증: 전 구간 `X≈345~356 mm`, `Z: 315→150 mm` 단조 하강, 손목은 `+Y` 빈 공간 쪽이라 노란 상자 비간섭).
- **담당 에이전트**: Antigravity 구현 및 실기 IK/FK 검증 (`pepero_almond` 100% 무영향).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_mid_z_delta_mm: 285.0` (`Z=315 mm`), `vertical_release_z_delta_mm: 120.0` (`Z=150 mm`), `vertical_fwd_tilt_deg: 60.0` (`ry=-120°`), `vertical_sweep_mm: 220.0` (`Y: +80 → -140 mm`) 적용.
  - `src/store_robot_control/store_robot_control/core/reorient_motion.py`, `nodes/executor_node.py`: `entry_axis="x"`일 때 `mid(Y=+80, Z=315) → end(Y=-140, Z=150, ry=-120°)` J4 `-Y` 회전 반전 경로 적용.
- **검증**: `py_compile` + 실기 M0609 IK/FK 검증 완료.

---

## 2026-11-27 21:06 — 포스틱 넘기기 J4 정반대 부호(`+50.7°`, `ry=+120°`) + `+Y` 방향(`-40 → +140 mm`) 적용

- **목표 / 배경**: 사용자 피드백에 따라 넘기기 각도를 기존 `-120°`(`J4=-51.6°`)의 정반대인 `+120°`(`J4=+50.7°`, `\` 모양)로 반전하고, 노란 입고상자 충돌을 방지하면서 왼쪽 끝단을 오른쪽 빈 공간(`Y: -40 → +140 mm`)으로 넘기도록 수정 (실기 M0609 IK/FK 검증: `ΔJ1=2.7°` 수직 상승 후 `Y: -40 → +140, Z: 315 → 150 mm` 단조 하강).
- **세부 변경 내역**: `config/products_catalog.yaml` (`vertical_fwd_tilt_sign: 1`, `vertical_sweep_mm: 180.0`), `reorient_motion.py` (`mid_shift_y = -28.5`, `+Y` 스윕).

---

## 2026-11-27 21:09 — 포스틱 45° 대각선 진입(`vertical_entry_yaw_deg: -45.0`) 적용

- **목표 / 배경**: 직전 `+Y` 반대 J4 넘기기(`mid Y=-40, Z=315 → end Y=+140, Z=150, ry=+120°`)를 유지하면서, 그리퍼와 봉지 끝단이 45° 대각선으로 만나도록 `vertical_entry_yaw_deg: -45.0`(`[-45, -140, 0]`, 바깥 대기 `[384.0, -124.8]` → 대각선 40 mm 슬라이드 `[355.7, -96.5]`) 적용 (실기 M0609 IK 전 단계 통과 검증 완료).
- **세부 변경 내역**: `config/products_catalog.yaml`, `grasp_planner.py`, `executor_node.py`에 `vertical_entry_yaw_deg` 추가 및 대각선 슬라이드 진입 벡터 반영.

---

## 2026-11-27 21:13 — 포스틱 180° 뒤집기 첫 실기 성공! 안정성 향상을 위해 45° 대각선 진입 깊이 `+15 mm` 심화

- **목표 / 배경**: `45°` 대각선 ㄷ자 진입 + `Z=315 mm` 높이 들기 + 반대 J4(`+50.7°`, `ry=+120°`) `+Y` 넘기기로 **포스틱 실기 180° 뒤집기 첫 성공** 달성! 파지 안정성을 높이기 위해 바깥 시작 위치(`[384.6, -125.4]`)는 그대로 유지하면서 파지점을 과자 안쪽으로 45° 대각선 방향 `15 mm` 더 깊게 들어가도록 보정.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick`: `vertical_corner_x_offset_mm: 80 → 70`, `vertical_edge_offset_mm: 85 → 75`, `vertical_slide_in_y_mm: 40 → 55`.

---

## 2026-11-27 21:16 — 포스틱 진입 기울기 `55°`로 바닥 쪽 밀착 + 집게 개방 `50 → 30 mm` 축소

- **목표 / 배경**: 사용자 요청에 따라 진입 각도를 바닥 쪽으로 `15°` 더 눕히고(`40° → 55°`, `ry=-125°`), 집게 개방을 `50 → 30 mm`로 줄여 아래쪽 손가락 처짐을 오히려 `1.7 mm` 더 줄임으로써 바닥 충돌 위험 없이 더 수평에 가깝게 진입하도록 개선. 실기 M0609 `J5` 상한(`130°`) 여유 확보를 위해 `entry_axis="x"`의 진입 대기 높이 하한을 `90 mm`(`J5=129.3°`)로 조정(`pepero_almond` 100% 무영향).
- **세부 변경 내역**: `config/products_catalog.yaml` (`vertical_back_tilt_deg: 55.0`, `vertical_lift_delta_mm: 60.0`, `open_width_mm: 30.0`), `reorient_motion.py` (`min_lift_z = 90.0 if entry_axis == "x" else 160.0`).

---

## 2026-11-27 21:17 — 포스틱 `55°` 진입 시 집게 개방폭 `30 → 50 mm` 복원

- **목표 / 배경**: `55°` 기울기에서 `30 mm` 개방은 수직 틈이 `~1.7 cm`로 좁아 봉지 끝단을 놓치므로 `50.0 mm`(수직 틈 `~2.9 cm`, `Z=30 mm` 기준 바닥과 약 `2.3 cm` 안전 여유)로 복원.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick.open_width_mm: 30.0 → 50.0`.

---

## 2026-11-27 21:26 — 2일차 마무리 및 내일(3일차) 업무 인계 요약

- **오늘 완료된 핵심 성과**:
  1. `pepero_almond` 실물 `pick_place` (`84 mm / 3 N`) + **180° 뒤집기(`back_tilt_vertical_flip`, 10단계)** 실기 반복 검증 완료 및 `main` 커밋·푸시(`e04cef3`, `pytest` 212/212 통과, Codex 2회 리뷰 반영).
  2. 팀 확정 YOLO 클래스명(`pepero_almond_front`, `postick_front`, `drink_front`, `ciga`) 별칭 호환(`aliases`) 추가.
  3. **`postick_front` 180° 뒤집기 첫 실기 성공** (`45°` 대각선 ㄷ자 진입 + `Z=315 mm` 높이 들기 + 반대 `J4 +50.7°` `+Y` 넘기기 — 현재 작업 트리에 미커밋 실험본 보관 중).
- **현재 Git 상태**: `e04cef3` 이후 `postick` 뒤집기 실험 변경사항(`config/products_catalog.yaml`, `grasp_planner.py`, `reorient_motion.py`, `executor_node.py`, `mock_robot.py`, `test_grasp_planner.py`, `WORKLOG.md`) 로컬 유지 중(내일 최적화 완료 후 테스트·Codex 리뷰·커밋 예정).
- **내일(3일차) 우선순위 업무**:
  1. **포스틱 뒤집기 안정화**: `40° + open 80 mm + Z=35 mm`(수직 포획 6.1 cm)로 오목/볼록 귀퉁이 편차 테스트 → 연속 5회 성공률 확인. (필요 시 금색 실링 중앙 파지 대안 비교)
  2. **포스틱 `pick_place` 실기 확인**: 뒤집기 외에 일반 입고/진열 이동용 파지 폭·힘 확인.
  3. **음료(`drink_front`, 알로에농장 또는 확정 품목) 실측 및 테스트**: 지름·높이·무게 실측 → `pick_place`(`30~40 N`) + 제자리 `J6` 180° 회전(`in_place_yaw_180`) 실기 검증.
  4. **포스틱·음료 최종본 단위 테스트 동기화 + Codex 코드 리뷰 + `main` 커밋/푸시**.
  5. **비전·LLM 팀원 싱크 회신 반영 및 합동 연동 준비**:
     - `product_back` 품목 식별 규칙(`product_name` 전달) 확정
     - 고정 Flip Zone(`zones.flip_zone`) 도입 여부 확정 및 `teach_zones`로 최종 현장 좌표(진열대 9슬롯, 버퍼 3슬롯, Flip Zone, `scan_pose`) 실측
     - 비전 `pose_3d_base` + `apply_grasp_depth_offset:=true` 실기 합동 `pick_place` 1회 검증(XY ±5 mm 이내 확인)
  6. **원격 조작(`hand_client`) 마무리**: 15 Hz 중립 프레임 키보드 희석 버그 수정, OpenCV 미리보기 창 추가, 실기 원격 조작 테스트.

### 내일 추가 확인 및 질문 예정 (21:30 추가)
- **LLM 회신 2건 확인**:
  1. https://doosan-rokey.slack.com/archives/C0CG89UDUES/p1795777566073149?thread_ts=1795774887.970189&cid=C0CG89UDUES
  2. https://doosan-rokey.slack.com/archives/C0CG89UDUES/p1795779374103809
- **팀원 질문 예정 (`product_back` 뒤집기 방식 결정)**:
  - 뒷면 YOLO 클래스가 `product_back` 하나로 통합되어 있는데, 실제 로봇 제어는 상품별로 파지·뒤집기 방식이 완전히 다름(빼빼로: `84 mm / 3 N` 끝단 중력 힌지, 포스틱: `45°` 대각선 ㄷ자 진입 + `Z=315` 반대 `J4` 넘기기, 음료: 강한 힘 + 제자리 `J6` 180° 회전).
  - 상품이 뒤집어져(`product_back`) 있을 때 **어떤 방법으로 뒤집을지(어느 품목인지)를 비전(VLM/3D bbox 크기 추정)에서 판별해 `product_name`으로 넘겨줄지, 아니면 LLM/오케스트레이터에서 어떻게 결정할지** 합의 필요.

---

## 2026-11-28 10:21 — LLM 파트 회신 접수 (연동 규약 반영 완료, `flip_zone` 도입 확정, 3대 확정 요청)

- **LLM 파트 반영 완료 내역** (`pytest` 250건 + `MockRobot` ROS 2 E2E 통과, LLM 브랜치 푸시):
  - `ExecuteRobotTask.product_category`에 구체 품목 키(`pepero_almond`, `postick`, `drink`, `ciga`) 전달 (`grasp_width_mm=0.0`, `grasp_force_n=0.0` 유지)
  - `yolo_class == "vest"` 스캔·파지 필터링 적용
  - `reorient → scan_pose + ScanWorkspace → pick_place` 폐루프 적용
  - `product_back`일 때 비전/VLM `product_name` 또는 앞면 스캔 이력(`track_id`) 기반으로 원래 품목 키 유지 전달 (완전 미상 시 현재 `""` → `defaults` 전달)
  - **고정 `flip_zone` 도입 확정**: `config/workspace_zones.yaml`에 `zones.flip_zone` 입력 시 자동으로 `pick_place(→ flip_zone, rz=0°) → reorient(flip_zone) → scan_pose → pick_place` 실행 (`scan_pose` 화각 내 위치 필수)
- **로봇 파트 회신/확정 필요 3건**:
  1. `drink`(병음료) RG2 실측 파지값 (`open/grasp/force/depth/reorient_mode`)
  2. `product_back` 완전 미상 시 안전 폴백 키·파라미터 (기존 `defaults` `20 mm / 18 N / edge_pivot_flip`은 빼빼로·포스틱 파손 위험 → 보수적 `product_back` 폴백 + 비전 3D 크기 기반 `product_name` 채우기 권장)
  3. `workspace_zones.yaml` 최종 실측 좌표 (`zones.flip_zone` 포함, `display_tiers`, `buffer_slots`, `display_area`, `inbound_box`, `scan_pose`)

---

## 2026-11-28 10:32 — 포스틱 오목/볼록 편차 흡수용 넓은 포획(`40° + open 80 mm + Z=35 mm`) 적용

- **목표 / 배경**: 전일 마지막 버전(`55° + open 50 mm`, 수직 포획 `2.8 cm`)이 귀퉁이 오목/볼록 편차로 실패함에 따라, 수직 포획 범위를 `6.1 cm`(2.1배)로 넓히는 `40° + open 80 mm + Z=35 mm` 설정을 적용해 오목/볼록 모두 안정적으로 잡히는지 실기 테스트.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick`: `vertical_back_tilt_deg: 55.0 → 40.0`, `vertical_lift_delta_mm: 60.0 → 125.0`, `open_width_mm: 50.0 → 80.0`.

---

## 2026-11-28 10:44 — 포스틱 수직 45° 인형뽑기 파지(`back_tilt_deg: 0.0`, `J6 -45°`) + 기존 J4 `+Y` 넘기기 적용

- **목표 / 배경**: 기울어진 옆면 슬라이드 진입의 오목/볼록 높이 편차를 근본 해소하기 위해, 사용자 스케치대로 **수직 하강(`[0, 180, -45]`) + 두 손가락이 왼쪽 하단 꼭짓점 양옆 빈 공간(`+X`/`-Y` 바깥)으로 내려가 `70 → 0 mm` 대각선으로 감싸 쥐는 인형뽑기 방식**으로 전환하고, 이후 검증된 `Z=315 mm` 높이 들기 + 반대 `J4(+49.9°, ry=+120°)` `+Y` 넘기기로 연결함 (실기 M0609 IK/FK 5단계 전 구간 검증 완료: 파지 중 `J4=0.0°`, 노란 상자 비간섭, 바닥 충돌 0).
- **세부 변경 내역**:
  - `config/products_catalog.yaml`: `postick`에 `vertical_back_tilt_deg: 0.0`, `vertical_entry_yaw_deg: -45.0`, `vertical_corner_x_offset_mm: 62.0`, `vertical_edge_offset_mm: 101.0`, `vertical_slide_in_y_mm: 0.0`, `open_width_mm: 70.0`, `vertical_lift_delta_mm: 172.0`, `vertical_mid_z_delta_mm: 327.0`, `vertical_release_z_delta_mm: 162.0` 적용.
  - `grasp_planner.py`, `reorient_motion.py`, `executor_node.py`: `back_tilt_deg == 0.0` 수직 인형뽑기 진입(`[0, 180, rz + entry_yaw]`) 지원 추가 (`pepero_almond` 100% 무영향).
- **검증**: `py_compile` + 실기 M0609 IK/FK 검증 통과.

---

## 2026-11-28 10:57 — 포스틱 수직 45° 인형뽑기 파지점을 꼭짓점 바깥쪽(`+X 10 mm, -Y 10 mm`)으로 이동

- **목표 / 배경**: 사용자 요청에 따라 높이(`Z=-12.0 mm`)는 유지하고, 파지 중심을 꼭짓점 쪽(`+X, -Y` 대각선 `~14 mm`)으로 더 이동해 얇은 빈 귀퉁이 끝을 잡도록 미세 조정.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick`: `vertical_corner_x_offset_mm: 62.0 → 72.0`, `vertical_edge_offset_mm: 101.0 → 111.0`.

---

## 2026-11-28 11:00 — 포스틱 수직 45° 인형뽑기 넘기기 `J4` 부호 반전(`ry=-120°`) 및 닫힘 시 바닥 충돌 방지(`Z=-5.0`)

- **목표 / 배경**: RG2 4절 링크 구조상 `70 → 0 mm` 닫힐 때 손가락 끝이 약 `12 mm` 아래로 뻗어 `Z=-12 mm`에서 바닥에 닿는 현상을 해결하기 위해 파지 높이를 `Z=-5.0 mm`로 `+7 mm` 상향하고, 넘기기 `J4` 부호를 반대 방향(`vertical_fwd_tilt_sign: 1 → -1`, `ry=-120°`, `J4=-36.6°`, 실기 M0609 IK/FK 검증 완료)으로 반전.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick.vertical_fwd_tilt_sign: 1 → -1`.

---

## 2026-11-28 11:07 — 포스틱 수직 45° 파지점 바깥쪽(`+X 10, -Y 10 mm`) 이동 및 들기 높이 하향(`Z_mid=290, Z_end=140`)

- **목표 / 배경**: `Z=+15.0 mm` 안전 파지 확인 후, 파지점을 꼭짓점 바깥쪽으로 `1.4 cm` 더 빼고 들어 올리는 높이를 약 `5 cm` 낮춤.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick`: `vertical_corner_x_offset_mm: 72→82`, `vertical_edge_offset_mm: 111→121`, `vertical_lift_delta_mm: 172→145`, `vertical_mid_z_delta_mm: 327→275`, `vertical_release_z_delta_mm: 162→125`.

---

## 2026-11-28 11:10 — 포스틱 45° 인형뽑기 두 손가락 수평 유지 + 팔 각도 `30°` 눕혀 진입(`[-45, -150, -90]`) 적용

- **목표 / 배경**: 수직(0°) 인형뽑기 대신 두 손가락의 45° 대각선 수평 배치(`toolX=[-0.707, -0.707, 0.0]`, 두 손가락 높이 동일)는 그대로 유지하면서 로봇팔 각도만 `30°` 바닥 쪽으로 눕혀 진입(`[-45, -150, -90]`, 실기 M0609 IK 전 단계 검증 완료: `J4=8.1°, J5=94.3°, J6=-75.4°`).
- **세부 변경 내역**: `config/products_catalog.yaml` (`vertical_back_tilt_deg: 0.0 → 30.0`), `executor_node.py` (`back_gamma = -90.0 + j6_off`).

---

## 2026-11-28 11:15 — 포스틱 30° 기울기 파지점 안쪽(`-X 7, +Y 7 mm`) 미세 보정 (`Z=-3.0` 테스트)

- **목표 / 배경**: 30° 기울기 45° 인형뽑기 파지 시 약간 안쪽(`10 mm` 대각선)으로 당겨 `Z=15.0 - 18.0 = -3.0 mm`로 테스트.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick`: `vertical_corner_x_offset_mm: 82→75`, `vertical_edge_offset_mm: 121→114`.

---

## 2026-11-28 11:22 — 포스틱 45° 수평 두 손가락 유지 + 진입 팔 각도 `30° → 50°` 상향 (`Z_app=125 mm` IK 검증)

- **목표 / 배경**: 사용자 요청에 따라 `Z=15.0 mm` 바닥 안전 높이를 유지하면서 팔 각도만 `30° → 50°`(`[-45, -130, -90]`)로 바닥 쪽에 더 붙여 진입하도록 수정. 실기 M0609 `ikin` 검증 결과 `Z_app=160 mm`는 `J5>130°` 불가, `Z_app=125 mm`(`lift_delta=110.0`)에서 전 단계 안전 통과(`J4=11.7°, J5=112.7~125.8°, J6=-76.6°`).
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick`: `vertical_back_tilt_deg: 30.0 → 50.0`, `vertical_lift_delta_mm: 145.0 → 110.0`.

---

## 2026-11-28 11:30 — 포스틱(`postick_front`) 180° 뒤집기 확정본 테스트 226개 통과, Codex 리뷰 반영 및 `main` 커밋·푸시

- **목표 / 배경**: 실기 M0609 + RG2에서 안정적으로 성공 확정된 포스틱 180° 뒤집기(`45°` 대각선 수평 두 손가락 + `50°` 팔 기울기 진입 `[-45, -130, -90]`, `open 70 / grasp 0 / 18 N`, `Z_app=125`, `Z_mid=290`, `J4=-36.6° / ry=-120°` `+Y` 넘기기)와 확정 YOLO 클래스 별칭(`aliases`)을 단위 테스트·크로스모델 리뷰까지 완료해 `origin/main`에 안전 백업함.
- **담당 에이전트**:
  - **Codex (`codex exec -s workspace-write` & `codex exec review --uncommitted`)**: `test_grasp_planner.py`, `test_reorient_motion.py`, `test_executor_dry_run.py` 226개 테스트 동기화·확충 및 코드 리뷰 수행(P1 0건, `[P2]` `MockRobot` 슬라이드 진입 경계 검증 일치 · `[P3]` 수직 파지 `j6_off` 반영 지적).
  - **Antigravity**: Codex 리뷰 `[P2]`/`[P3]` 수정, `colcon build` + CycloneDDS 전체 `pytest` 226/226 최종 검증 및 `origin/main` 커밋·푸시.
- **시행착오 및 해결 내역 (포스틱 최적화 최종 요약)**:
  - 몸통 핀치 시 질소 압력으로 파손 우려 → 빈 왼쪽 하단 꼭짓점으로 전환.
  - 기울인 채 `J6=90°` 옆면 ㄷ자 진입은 아래쪽 손가락 처짐(테이블 충돌)과 오목/볼록 귀퉁이 높이 편차(`2~3 cm`)에 민감 → 두 손가락을 45° 대각선 수평(`toolX Z=0`)으로 꼭짓점 양옆 빈 공간에 내리는 인형뽑기 방식으로 전환.
  - RG2 4절 링크가 `70 → 0 mm`로 완전히 닫힐 때 손가락 끝이 아래로 `~20~25 mm` 뻗어 `Z=-12/-5/-3 mm`에서 바닥에 닿음 → `Z=+15.0 mm` 안전 높이 확보 후 팔 각도만 `50°` 눕혀(`[-45, -130, -90]`, `Z_app=125 mm` IK 검증) + `J4=-36.6°(ry=-120°)` `+Y` 넘기기로 안정적 180° 반전 확정.
- **검증 결과**: `colcon build --symlink-install --packages-select store_robot_control` 성공, `pytest -q src/store_robot_control/tests` **226 passed**, `git diff --check` 통과.
- **후속 조치**: 포스틱 일반 이동(`pick_place`) 확인 → 음료(`drink_front`) 실측·테스트 → `product_back` 안전 폴백 및 `zones.flip_zone` 실측 반영.

---

## 2026-11-28 11:36 — 포스틱 수직 45° 꼭짓점 `pick_place` 오프셋(`pick_offset_x/y`, `pick_yaw_offset_deg`, `pick_transit_z_mm`) 추가

- **목표 / 배경**: 포스틱 중앙 파지 시 터짐·미끄러짐 리스크를 없애고 입고상자 벽 간섭을 피하기 위해, `pick_place`/`move_to_buffer`/`restore_from_buffer` 시 상품 정중앙(`target_pose`/`dest_pose`)에 자동으로 꼭짓점 오프셋(`+X 75, -Y 114 mm`, `J6 -45°`)과 높은 이동 높이(`transit_z=320 mm`)를 양쪽 동일 적용하는 옵션 A 구현 (`pepero_almond` 등 타 상품 100% 무영향).
- **세부 변경 내역**: `config/products_catalog.yaml`, `grasp_planner.py`, `executor_node.py`, `mock_robot.py`.
- **검증**: `py_compile` + `pick_place` 오프셋 좌표 검증 완료.

---

## 2026-11-28 11:38 — 포스틱 `pick_place` 이동 높이 `320 → 260 mm` 보정 (Dest B 외곽 도달 한계 해소)

- **목표 / 배경**: Dest B 꼭짓점 오프셋 위치(`[555.8, -190.9]`, 반경 `588 mm`)에서 `Z=320/290 mm`는 M0609 상단 도달 한계를 초과해 `lift` 후 `OUT_OF_REACH`로 중단됨을 실기 `ikin`으로 확인. `Z=260.0 mm`(바닥 위 `28.4 cm`, `sol=2: J2=39.7°, J3=22.5°, J5=117.8°`)에서는 안전 도달 가능하므로 `pick_transit_z_mm`을 `260.0`으로 하향 조정.
- **세부 변경 내역**: `config/products_catalog.yaml`의 `postick.pick_transit_z_mm: 320.0 → 260.0`.

---

## 2026-11-28 11:42 — 포스틱 `pick_place` 투하 시 봉지 압박 방지용 `place_z_offset_mm: 95.0` 추가

- **목표 / 배경**: 꼭짓점으로 매달려 이동한 포스틱을 `dest_z=15 mm`까지 끝까지 내려놓으면 아래에 깔린 봉지를 눌러 터질 위험이 있으므로, 투하 높이만 `+95 mm`(약 `Z=110 mm`, 바닥 위 ~13 cm)에서 부드럽게 놓도록 `place_z_offset_mm` 추가 (`pepero_almond` 등 타 상품 `0.0` 무영향).
- **세부 변경 내역**: `config/products_catalog.yaml`, `grasp_planner.py`, `executor_node.py`, `mock_robot.py`.


---

## 2026-11-28 11:52 — 포스틱(`postick_front`) `pick_place` 실기 성공본 테스트 232개 통과, Codex 리뷰 반영 및 `main` 커밋·푸시

- **목표 / 배경**: 실기 M0609 + RG2에서 정상 동작이 확인된 포스틱 수직 45° 꼭짓점 `pick_place`(`pick_offset_x=75.0`, `pick_offset_y=-114.0`, `pick_yaw_offset_deg=-45.0`, `pick_transit_z_mm=260.0`, `place_z_offset_mm=95.0`)를 단위 테스트·크로스모델 코드 리뷰까지 완료해 `origin/main`에 안전 백업함.
- **담당 에이전트**:
  - **Codex (`codex exec -s workspace-write` & `codex exec review --uncommitted`)**: `test_grasp_planner.py`, `test_executor_dry_run.py` 단위 테스트 5건 추가(231 passed) 및 코드 리뷰 수행(`[P1]` 보정 목적지 J6 한계 파지 전 사전 검증 누락, `[P2]` `place_z_offset_mm`이 `safe_transit_z`에 합산되어 검증 이동 높이 `260 mm`를 초과할 수 있는 문제 지적).
  - **Antigravity**: `executor_node.py`에 파지 전 출발·목적지 J6 한계 사전 검사(`[P1]`)와 `executor_node.py`/`mock_robot.py`에 투하 높이·이동 높이 분리 계산 및 초과 시 파지 전 `OUT_OF_REACH` 거부(`[P2]`) 반영, 회귀 테스트 추가, `colcon build` + `pytest` 232/232 검증 및 Git 커밋·푸시.
- **시행착오 및 해결 내역 (포스틱 `pick_place` 최적화)**:
  - 중앙 파지 대신 왼쪽 하단 빈 꼭짓점 수직 45° 오프셋(`+75, -114, -45°`)으로 터짐·미끄러짐 및 입고상자 벽 간섭 해소.
  - 초기 `pick_transit_z_mm: 320.0`은 Dest B 외곽(`[555.8, -190.9]`, 반경 `588 mm`)에서 M0609 상단 도달 한계(`OUT_OF_REACH`)에 걸려 집고 들기만 하고 중단 → 실기 `ikin` 검증 후 `260.0 mm`로 보정.
  - 투하 시 `dest_z=15 mm`까지 내려오면 매달린 봉지를 바닥에 눌러 터질 위험 → `place_z_offset_mm: 95.0`(`Z≈110 mm`에서 부드럽게 놓기) 추가로 실기 정상 완료 확인.
- **검증 결과**: `colcon build --symlink-install --packages-select store_robot_control` 성공, `pytest -q src/store_robot_control/tests` **232 passed**, `git diff --check` 통과, Codex 리뷰 `[P1]`/`[P2]` 전건 해결.
- **후속 조치**: 비전 확정 치수(`빼빼로 88×162`, `포스틱 180×235`, `병음료 63×208`) 기준 검토 및 `병음료(drink_front)` 파지·뒤집기 실기 설계/검증 진행.
