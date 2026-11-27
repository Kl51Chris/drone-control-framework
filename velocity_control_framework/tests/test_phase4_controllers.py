from pathlib import Path

import numpy as np
import pytest

from velocity_control_framework.controllers.outer_loop_deepc import (
    OuterLoopDeePCConfig, OuterLoopDeePCController,
)
from velocity_control_framework.controllers.identified_hover_model import (
    IdentifiedHoverModelConfig, identified_discrete_model,
)
from velocity_control_framework.controllers.near_hover_model import NearHoverModelConfig, discrete_model
from velocity_control_framework.interfaces import DroneState, Reference
from experiments.measurement_noise import MeasurementNoise


def test_identified_fir_augmentation_matches_direct_rollout():
    coeffs=np.arange(24,dtype=float).reshape(4,6)*.01
    bias=np.array([.001,-.002,.005,.01])
    candidate=IdentifiedHoverModelConfig("fixture","fixture",tuple(map(tuple,coeffs)),tuple(bias))
    config=NearHoverModelConfig()
    ad,bd=discrete_model(config)
    a,b=identified_discrete_model(config,candidate)
    rng=np.random.default_rng(42)
    x=np.r_[rng.normal(size=29),1.]
    u=rng.normal(size=4)
    # Filter coefficients use current input followed by five issued packets.
    effective=np.sum(coeffs.T*np.vstack((u,x[9:29].reshape(5,4))),axis=0)+bias
    expected=ad@x[:9]+bd@(effective/[1,-1,-1,config.thrust_gain_m_s2])
    y=a@x+b@u
    np.testing.assert_allclose(y[:9],expected)
    np.testing.assert_allclose(y[9:29],np.r_[u,x[9:25]])
    assert y[-1]==1


@pytest.mark.parametrize("past", [6, 12, 18, 25])
def test_deepc_uses_sent_commands_and_preserves_input_box(past):
    pytest.importorskip("osqp")
    if not Path("log/phase3-v1/manifest.json").exists():
        pytest.skip("local phase3-v1 training artifact is not mounted")
    c=OuterLoopDeePCController(OuterLoopDeePCConfig(past=past))
    ref=Reference.hover(.03,0.,0.)
    for i in range(past+2):
        command=c.update(DroneState.zero(timestamp=1.+i*.02),ref,.02)
        expected="mpc_history_bridge" if i<past else "deepc"
        assert c.diagnostics["active_controller"]==expected
        assert max(abs(command.roll_rate),abs(command.pitch_rate),abs(command.yaw_rate))<=.2
        assert .38<=command.thrust<=.56
        c.record_issued_command(command)
    assert c.prediction["state"].shape==(26,9)
    assert c.diagnostics["g_norm"]>0.
    assert np.all(np.isfinite(c.prediction["slack"]))


def test_measurement_noise_replays_by_index_and_zero_preserves_state():
    state=DroneState.zero(timestamp=3.)
    zero,log=MeasurementNoise("iid-small",0.,12).apply(state,7)
    assert zero is state and log["raw"]==log["used"]
    left=MeasurementNoise("iid-small",1.,12)
    right=MeasurementNoise("iid-small",1.,12)
    left.apply(state,6)
    a,la=left.apply(state,7)
    b,lb=right.apply(state,7)
    assert la==lb and la["delta"]!=[0.]*12
    np.testing.assert_array_equal(a.position,b.position)
    np.testing.assert_array_equal(state.position,[0.,0.,0.])
    assert a.timestamp==3.


def test_mixed_fir_preserves_each_axis_history_across_rollout():
    rng = np.random.default_rng(43)
    lengths = (6, 6, 25, 12)
    coefficients = tuple(tuple(rng.normal(size=length)) for length in lengths)
    bias = (.001, -.002, .005, .01)
    model = NearHoverModelConfig()
    candidate = IdentifiedHoverModelConfig("fixture", "fixture", coefficients, bias)
    a, b = identified_discrete_model(model, candidate)
    ad, bd = discrete_model(model)
    x = rng.normal(size=9)
    history = rng.normal(size=(24, 4))
    state = np.r_[x, [history[lag, axis] for lag, axis in candidate.history_slots], 1.]
    assert state.size == 55
    for u in rng.normal(size=(40, 4)):
        packets = np.vstack((u, history))
        effective = [w @ packets[:len(w), axis] + bias[axis] for axis, w in enumerate(coefficients)]
        x = ad @ x + bd @ (effective / np.array([1., -1., -1., model.thrust_gain_m_s2]))
        state = a @ state + b @ u
        history = packets[:-1]
        np.testing.assert_allclose(state[:9], x, rtol=1e-12, atol=1e-12)
        np.testing.assert_array_equal(state[9:-1], [history[lag, axis] for lag, axis in candidate.history_slots])
