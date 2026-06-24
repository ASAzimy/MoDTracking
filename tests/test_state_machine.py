import unittest

from modtracking import TrackingState, TrackingStateMachine


class TrackingStateMachineTests(unittest.TestCase):
    def test_nominal_tracking_and_reacquire_flow(self) -> None:
        machine = TrackingStateMachine()

        machine.transition(TrackingState.TAKEOFF)
        machine.transition(TrackingState.MANUAL)
        machine.target_selected()
        machine.tracker_locked()
        machine.run_pursuit()
        machine.target_lost()
        machine.begin_reacquire()
        machine.tracker_locked()

        self.assertEqual(machine.state, TrackingState.TRACKING)
        self.assertEqual(
            [state.value for state in machine.history],
            [
                "IDLE",
                "TAKEOFF",
                "MANUAL",
                "TARGET_SELECTED",
                "TRACKING",
                "PURSUIT",
                "LOST_TARGET",
                "REACQUIRE",
                "TRACKING",
            ],
        )

    def test_invalid_transition_is_rejected(self) -> None:
        machine = TrackingStateMachine()

        with self.assertRaises(ValueError):
            machine.run_pursuit()

    def test_manual_override_and_landing_are_available_from_pursuit(self) -> None:
        machine = TrackingStateMachine()
        machine.takeoff_complete()
        machine.target_selected()
        machine.tracker_locked()
        machine.run_pursuit()

        machine.manual_override()
        self.assertEqual(machine.state, TrackingState.MANUAL)

        machine.landed()
        self.assertEqual(machine.state, TrackingState.IDLE)

    def test_landing_is_available_from_lost_target(self) -> None:
        machine = TrackingStateMachine()
        machine.takeoff_complete()
        machine.target_selected()
        machine.tracker_locked()
        machine.target_lost()

        machine.landed()
        self.assertEqual(machine.state, TrackingState.IDLE)


if __name__ == "__main__":
    unittest.main()
