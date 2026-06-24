import unittest

from modtracking import BoundingBox, FrameSize, TrackingMetrics, TrackingObservation


class TrackingMetricsTests(unittest.TestCase):
    def test_metrics_accumulate_lock_rate_recovery_and_distance_error(self) -> None:
        metrics = TrackingMetrics(frame=FrameSize(width=100, height=100), desired_distance_m=10)

        metrics.add(
            TrackingObservation(
                frame_index=0,
                bbox=BoundingBox(x=40, y=40, width=20, height=20),
                estimated_distance_m=11,
            )
        )
        metrics.add(TrackingObservation(frame_index=1, bbox=None, confidence=0))
        metrics.add(TrackingObservation(frame_index=2, bbox=None, confidence=0))
        metrics.add(
            TrackingObservation(
                frame_index=4,
                bbox=BoundingBox(x=50, y=40, width=20, height=20),
                estimated_distance_m=8,
            )
        )

        summary = metrics.summary()

        self.assertEqual(summary["total_frames"], 4)
        self.assertEqual(summary["locked_frames"], 2)
        self.assertEqual(summary["lost_events"], 1)
        self.assertEqual(summary["tracking_success_rate"], 0.5)
        self.assertEqual(summary["average_recovery_frames"], 3)
        self.assertEqual(summary["average_distance_error_m"], 1.5)
        self.assertGreater(summary["average_center_error_px"], 0)

    def test_empty_metrics_summary_is_zeroed(self) -> None:
        metrics = TrackingMetrics(frame=FrameSize(width=640, height=480))

        self.assertEqual(
            metrics.summary(),
            {
                "total_frames": 0,
                "locked_frames": 0,
                "lost_events": 0,
                "tracking_success_rate": 0.0,
                "average_center_error_px": 0.0,
                "average_recovery_frames": 0.0,
                "average_distance_error_m": 0.0,
            },
        )


if __name__ == "__main__":
    unittest.main()
