from types import SimpleNamespace

import pytest

from store_robot_control.hardware.rg2_gripper import RG2Gripper


def test_dry_run_open_grasp_and_grip_failure():
    gripper = RG2Gripper("unused", 502, dry_run=True)
    assert gripper.connect()
    assert gripper.get_width_mm() == 110.0
    gripper.open_mm(84.13, 20.04)
    assert gripper.get_width_mm() == 84.1
    assert gripper.get_status_bits() == {"busy": False, "grip_detected": False}
    gripper.move_mm(42.56, 17.26)
    assert gripper.get_width_mm() == 42.6
    assert gripper._force_n == 17.3
    assert gripper.is_grip_detected()
    gripper.simulate_grip_fail = True
    gripper.move_mm(40, 15)
    assert not gripper.is_grip_detected()
    assert gripper.wait_until_idle()
    gripper.close()


@pytest.mark.parametrize("unit_key", ["slave", "device_id"])
def test_modbus_register_conversion(unit_key):
    writes, reads = [], []

    def write(address, values, *, slave=None, device_id=None):
        writes.append((address, values, slave, device_id))
        return SimpleNamespace(isError=lambda: False)

    def read(address, count, *, slave=None, device_id=None):
        reads.append((address, count, slave, device_id))
        return SimpleNamespace(isError=lambda: False, registers=[426 if address == 267 else 2])

    if unit_key == "slave":
        def write_registers(address, values, *, slave):
            return write(address, values, slave=slave)

        def read_holding_registers(address, count, *, slave):
            return read(address, count, slave=slave)
    else:
        def write_registers(address, values, *, device_id):
            return write(address, values, device_id=device_id)

        def read_holding_registers(address, count, *, device_id):
            return read(address, count, device_id=device_id)

    gripper = RG2Gripper("unused", 502, wait_after_move_sec=0)
    gripper._client = SimpleNamespace(
        write_registers=write_registers, read_holding_registers=read_holding_registers,
    )
    gripper.move_mm(42.56, 17.26)
    assert gripper.get_width_mm() == 42.6
    assert gripper.get_status_bits() == {"busy": False, "grip_detected": True}
    unit = (65, None) if unit_key == "slave" else (None, 65)
    assert writes == [(0, [173, 426, 16], *unit)]
    assert reads == [(268, 1, *unit), (267, 1, *unit), (268, 1, *unit)]
