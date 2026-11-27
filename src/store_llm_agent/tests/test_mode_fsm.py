"""ModeFSM 단위 테스트."""

from store_llm_agent.core.mode_fsm import MODE_CUSTOMER, MODE_EMPLOYEE, MODE_REMOTE, ModeFSM


def test_vest_detection_debounce_and_timeout() -> None:
    fsm = ModeFSM(required_vest_frames=5, vest_conf_threshold=0.7, employee_timeout_sec=5.0)
    assert fsm.mode == MODE_CUSTOMER

    # 4프레임까지는 CUSTOMER 유지
    for i in range(4):
        assert fsm.update_vest_detection(True, 0.85, now=float(i)) == MODE_CUSTOMER

    # 중간에 낮은 신뢰도 프레임이 들어오면 카운터 리셋
    assert fsm.update_vest_detection(True, 0.5, now=4.0) == MODE_CUSTOMER
    assert fsm.update_vest_detection(True, 0.9, now=5.0) == MODE_CUSTOMER

    # 연속 5프레임 달성 시 즉시 EMPLOYEE 전환
    for i in range(4):
        fsm.update_vest_detection(True, 0.9, now=6.0 + i)
    assert fsm.mode == MODE_EMPLOYEE

    # 작업 중에는 미감지 10초가 지나도 EMPLOYEE 유지
    fsm.set_task_active(True, now=10.0)
    assert fsm.update_vest_detection(False, 0.0, now=20.0) == MODE_EMPLOYEE

    # 작업 종료 후 5초 미만은 유지, 5초 이상 경과 시 CUSTOMER 복귀
    fsm.set_task_active(False, now=21.0)
    assert fsm.update_vest_detection(False, 0.0, now=25.9) == MODE_EMPLOYEE
    assert fsm.update_vest_detection(False, 0.0, now=26.0) == MODE_CUSTOMER


def test_permissions_by_mode() -> None:
    fsm = ModeFSM(required_vest_frames=2)

    # CUSTOMER 모드 권한 체크
    assert fsm.check_permission("order")[0] is True
    assert fsm.check_permission("query_stock")[0] is True
    assert fsm.check_permission("inspect")[0] is False
    assert fsm.check_permission("dispose")[0] is False
    assert fsm.check_permission("inbound")[0] is False

    # EMPLOYEE 모드 전환 후 권한 체크
    fsm.update_vest_detection(True, 0.9, now=1.0)
    fsm.update_vest_detection(True, 0.9, now=2.0)
    assert fsm.mode == MODE_EMPLOYEE
    for intent in ("order", "query_stock", "inspect", "dispose", "inbound"):
        assert fsm.check_permission(intent)[0] is True

    # REMOTE 모드에서는 자동 명령 전부 차단
    assert fsm.enter_remote() == MODE_REMOTE
    assert fsm.check_permission("order")[0] is False
    assert fsm.exit_remote() == MODE_CUSTOMER

    # 음성 'remote' intent는 관리자 키 인증 안내와 함께 차단
    allowed_remote, msg_remote = fsm.check_permission("remote")
    assert allowed_remote is False
    assert "verify_remote_key" in msg_remote
