import unittest

from modtracking import BoundingBox, FrameSize, GimbalController, PursuitController, TrackingObservation


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.frame = FrameSize(width=1280, height=720)

    def test_gimbal_yaws_right_and_pitches_up_for_upper_right_target(self) -> None:
        observation = TrackingObservation(
            frame_index=1,
            bbox=BoundingBox(x=900, y=100, width=100, height=100),
        )

        command = GimbalController().update(observation, self.frame)

        self.assertGreater(command.yaw_rate, 0)
        self.assertGreater(command.pitch_rate, 0)

    def test_gimbal_holds_when_target_is_lost(self) -> None:
        observation = TrackingObservation(frame_index=2, bbox=None)

        command = GimbalController().update(observation, self.frame)

        self.assertEqual(command.yaw_rate, 0)
        self.assertEqual(command.pitch_rate, 0)

    def test_pursuit_moves_forward_when_target_is_too_far(self) -> None:
        observation = TrackingObservation(
            frame_index=3,
            bbox=BoundingBox(x=590, y=310, width=100, height=100),
            estimated_distance_m=14.0,
        )

        command = PursuitController(desired_distance_m=10.0).update(observation, self.frame)

        self.assertGreater(command.forward_mps, 0)
        self.assertEqual(command.right_mps, 0)
        self.assertEqual(command.up_mps, 0)

    def test_pursuit_moves_backward_when_target_is_too_close(self) -> None:
        observation = TrackingObservation(
            frame_index=4,
            bbox=BoundingBox(x=590, y=310, width=100, height=100),
            estimated_distance_m=6.0,
        )

        command = PursuitController(desired_distance_m=10.0).update(observation, self.frame)

        self.assertLess(command.forward_mps, 0)

    def test_pursuit_lateral_and_yaw_follow_image_error(self) -> None:
        observation = TrackingObservation(
            frame_index=5,
            bbox=BoundingBox(x=1000, y=310, width=100, height=100),
            estimated_distance_m=10.0,
        )

        command = PursuitController().update(observation, self.frame)

        self.assertGreater(command.right_mps, 0)
        self.assertGreater(command.yaw_rate, 0)
        self.assertEqual(command.forward_mps, 0)


if __name__ == "__main__":
    unittest.main()
