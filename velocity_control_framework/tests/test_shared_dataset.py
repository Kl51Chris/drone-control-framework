import numpy as np

from experiments.excitation import HoverExcitation
from tools.shared_dataset import hankel, matrices


def test_hankel_does_not_cross_runs_and_shifts_output_one_tick():
    episodes = [{"u": np.full((5, 4), i), "y": np.broadcast_to(
        (i * 100 + np.arange(5))[:, None], (5, 9)
    )} for i in (1, 2)]
    h = hankel(episodes, "u", 3)
    assert h.shape == (12, 6)
    assert np.all(h == h[:1])
    x, y = matrices(episodes, past=1, future=2, u_scale=np.ones(4), y_scale=np.ones(9))
    np.testing.assert_array_equal(x[-1], [101, 102, 201, 202])
    np.testing.assert_array_equal(y[-1], [103, 104, 203, 204])


def test_excitation_is_seeded_bounded_and_axis_specific():
    first = HoverExcitation(123, "yaw", 8)
    second = HoverExcitation(123, "yaw", 8)
    np.testing.assert_array_equal(first.values, second.values)
    assert np.max(np.abs(first.values[:, 2])) <= .045
    assert not np.any(first.values[:, [0, 1, 3]])
    assert not np.any(first.sample(0.0))
