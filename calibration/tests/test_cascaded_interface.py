import math
import struct

import pytest
from cflib.crazyflie.commander import Commander, TYPE_MANUAL

from calibration.cascaded_pid import CascadedPIDConfig, CascadedPIDController, CascadedReference
from calibration.state import AltitudeState
from velocity_control_framework.backends import CrazyflieCommandAdapter


class PacketSink:
    def __init__(self): self.packet = None
    def send_packet(self, packet): self.packet = packet


class AdapterCF:
    def __init__(self, commander): self.commander = commander


def test_controller_adapter_and_installed_cflib_exact_manual_packet_contract():
    state = AltitudeState(0, 0, .02, .03, .04, 1)
    command = CascadedPIDController(CascadedPIDConfig()).update(
        state, CascadedReference(0), .02).command
    sink = PacketSink()
    CrazyflieCommandAdapter(AdapterCF(Commander(sink))).send(command)
    packet_type, roll, packed_pitch, yaw, thrust, rate = struct.unpack(
        "<BfffHB", sink.packet.data)
    assert packet_type == TYPE_MANUAL
    assert roll == pytest.approx(math.degrees(command.roll_rate))
    assert packed_pitch == pytest.approx(-math.degrees(command.pitch_rate))
    assert yaw == pytest.approx(math.degrees(command.yaw_rate))
    assert thrust == int(10001 + command.thrust*49999)
    assert rate == 1
    # For positive legacy pitch state, the API command is positive and cflib's
    # packet field is negative: the net firmware-side correction is restoring.
    assert command.pitch_rate > 0 and packed_pitch < 0
