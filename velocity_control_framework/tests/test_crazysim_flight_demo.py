from __future__ import annotations

import struct

import numpy as np
import pytest
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crtp.crtpstack import CRTPPort

from simulation.scripts.demo_crazysim_flight import (
    CRAZYSIM_URI,
    MAX_INITIAL_SPEED_M_S,
    stop_commands,
    validate_crazysim_uri,
    validate_stationary_start,
)


class State:
    def __init__(self, velocity=(0.0, 0.0, 0.0), roll=0.0, pitch=0.0) -> None:
        self.velocity = np.array(velocity, dtype=float)
        self.roll = roll
        self.pitch = pitch


def test_demo_uri_is_restricted_to_single_crazysim_endpoint() -> None:
    validate_crazysim_uri(CRAZYSIM_URI)
    with pytest.raises(ValueError):
        validate_crazysim_uri("radio://0/80/2M")
    with pytest.raises(ValueError):
        validate_crazysim_uri("udp://127.0.0.1:19851")


def test_stationary_start_validation() -> None:
    validate_stationary_start(State())
    with pytest.raises(RuntimeError, match="not stationary"):
        validate_stationary_start(State(velocity=(MAX_INITIAL_SPEED_M_S + 0.01, 0, 0)))
    with pytest.raises(RuntimeError, match="not level"):
        validate_stationary_start(State(roll=0.16))


class PacketSink:
    def __init__(self) -> None:
        self.packets = []

    def send_packet(self, packet) -> None:
        self.packets.append(packet)


def test_cflib_v7_high_level_packets_match_bundled_firmware_commands() -> None:
    sink = PacketSink()
    commander = HighLevelCommander(sink)
    commander.takeoff(0.35, 3.0, yaw=None)
    commander.land(0.03, 3.0, yaw=None)
    commander.stop()

    assert [packet.port for packet in sink.packets] == [CRTPPort.SETPOINT_HL] * 3
    takeoff = struct.unpack("<BBff?f", bytes(sink.packets[0].data))
    landing = struct.unpack("<BBff?f", bytes(sink.packets[1].data))
    assert takeoff == pytest.approx((7, 0, 0.35, 0.0, True, 3.0))
    assert landing == pytest.approx((8, 0, 0.03, 0.0, True, 3.0))
    assert bytes(sink.packets[2].data) == bytes((3, 0))


class StopHighLevel:
    def __init__(self, events) -> None:
        self.events = events

    def stop(self) -> None:
        self.events.append("high_level_stop")


class StopLowLevel:
    def __init__(self, events) -> None:
        self.events = events

    def send_stop_setpoint(self) -> None:
        self.events.append("generic_stop")


def test_cleanup_sends_both_stop_paths_in_order() -> None:
    events = []
    cf = type("CF", (), {})()
    cf.high_level_commander = StopHighLevel(events)
    cf.commander = StopLowLevel(events)
    assert stop_commands(cf) == []
    assert events == ["high_level_stop", "generic_stop"]


def test_cleanup_attempts_generic_stop_if_high_level_stop_fails() -> None:
    events = []
    cf = type("CF", (), {})()
    cf.high_level_commander = StopHighLevel(events)
    cf.high_level_commander.stop = lambda: (_ for _ in ()).throw(OSError("fail"))
    cf.commander = StopLowLevel(events)
    errors = stop_commands(cf)
    assert errors and errors[0].startswith("high_level_stop:")
    assert events == ["generic_stop"]
