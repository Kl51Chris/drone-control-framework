import unittest

import numpy as np

from velocity_control_framework.interfaces import (
    DroneState,
    Reference,
    VelocityCommand,
)


class TestInterfaceImports(unittest.TestCase):
    def test_classes_can_be_imported(self) -> None:
        self.assertIsNotNone(DroneState)
        self.assertIsNotNone(Reference)
        self.assertIsNotNone(VelocityCommand)


class TestDroneState(unittest.TestCase):
    def test_zero_state(self) -> None:
        state = DroneState.zero(timestamp=1.5)

        np.testing.assert_array_equal(
            state.position,
            np.zeros(3, dtype=np.float64),
        )
        np.testing.assert_array_equal(
            state.velocity,
            np.zeros(3, dtype=np.float64),
        )

        self.assertEqual(state.yaw, 0.0)
        self.assertEqual(state.yaw_rate, 0.0)
        self.assertEqual(state.timestamp, 1.5)


class TestReference(unittest.TestCase):
    def test_hover_reference(self) -> None:
        reference = Reference.hover(
            x=1.0,
            y=-2.0,
            z=0.5,
            yaw=0.25,
        )

        np.testing.assert_array_equal(
            reference.position,
            np.array([1.0, -2.0, 0.5]),
        )
        np.testing.assert_array_equal(
            reference.velocity,
            np.zeros(3),
        )

        self.assertEqual(reference.yaw, 0.25)
        self.assertEqual(reference.yaw_rate, 0.0)


class TestVelocityCommand(unittest.TestCase):
    def test_zero_command(self) -> None:
        command = VelocityCommand.zero()

        np.testing.assert_array_equal(
            command.velocity,
            np.zeros(3),
        )

        self.assertEqual(command.vx, 0.0)
        self.assertEqual(command.vy, 0.0)
        self.assertEqual(command.vz, 0.0)
        self.assertEqual(command.yaw_rate, 0.0)

    def test_command_clipping(self) -> None:
        command = VelocityCommand(
            velocity=np.array([3.0, 4.0, 2.0]),
            yaw_rate=3.0,
        )

        limited = command.clipped(
            max_horizontal_speed=2.0,
            max_vertical_speed=0.5,
            max_yaw_rate=1.0,
        )

        self.assertAlmostEqual(
            np.linalg.norm(limited.velocity[:2]),
            2.0,
        )
        self.assertAlmostEqual(limited.vx, 1.2)
        self.assertAlmostEqual(limited.vy, 1.6)
        self.assertEqual(limited.vz, 0.5)
        self.assertEqual(limited.yaw_rate, 1.0)

    def test_invalid_vector_shape_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            VelocityCommand(
                velocity=np.array([1.0, 2.0]),
                yaw_rate=0.0,
            )


if __name__ == "__main__":
    unittest.main()
