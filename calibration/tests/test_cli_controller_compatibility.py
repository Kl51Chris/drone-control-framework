import pytest

from calibration.cascaded_pid import CascadedPIDConfig, HorizontalLoopConfig, load_cascaded_config
from calibration.cli import _validate_cascaded_mode_config
from calibration.experiment import Mode


@pytest.mark.parametrize("path",[
    "calibration/stage5b_offline_config.json",
    "calibration/stage5c_altitude_pd_pole_candidate_config.json",
])
def test_pitch_response_accepts_stage5b_and_stage5c_cascaded_configs(path):
    _validate_cascaded_mode_config(Mode.PITCH_RESPONSE_ID,load_cascaded_config(path))
    _validate_cascaded_mode_config(Mode.ROLL_RESPONSE_ID,load_cascaded_config(path))


def test_pitch_response_rejects_stage5d_even_when_controller_is_valid():
    config=load_cascaded_config("calibration/stage5d_xy_first_flight_config.json")
    with pytest.raises(SystemExit,match="Stage 5B or Stage 5C"):
        _validate_cascaded_mode_config(Mode.PITCH_RESPONSE_ID,config)


def test_pitch_response_preserves_xy_disabled_requirement():
    config=CascadedPIDConfig(stage="5C",horizontal=HorizontalLoopConfig(
        enabled=True,position_kp=.1,velocity_kp=.1,max_acceleration_m_s2=.2,
        max_tilt_reference_rad=.05))
    with pytest.raises(ValueError,match="Stage 5C does not enable horizontal"):
        _validate_cascaded_mode_config(Mode.PITCH_RESPONSE_ID,config)


def test_xy_mode_still_requires_mapping_hardware_validation():
    config=load_cascaded_config("calibration/stage5d_xy_first_flight_config.json")
    assert config.horizontal_mapping_hardware_validated is False
    with pytest.raises(SystemExit,match="horizontal_mapping_hardware_validated is false"):
        _validate_cascaded_mode_config(Mode.CASCADED_XY,config)


def test_other_cascaded_modes_keep_exact_stage_matching():
    stage5c=load_cascaded_config("calibration/stage5c_altitude_pd_pole_candidate_config.json")
    with pytest.raises(SystemExit,match="cascaded-5b requires a Stage 5B"):
        _validate_cascaded_mode_config(Mode.CASCADED_5B,stage5c)
