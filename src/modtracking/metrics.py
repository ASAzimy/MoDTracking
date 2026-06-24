"""Evaluation metrics for pure object tracking and following experiments."""

from __future__ import annotations

from dataclasses import dataclass, field
from statistics import fmean

from modtracking.models import FrameSize, TrackingObservation


@dataclass
class TrackingMetrics:
    """Accumulates frame-level tracking and following performance metrics."""

    frame: FrameSize
    desired_distance_m: float = 10.0
    total_frames: int = 0
    locked_frames: int = 0
    lost_events: int = 0
    recovery_frames: list[int] = field(default_factory=list)
    center_errors_px: list[float] = field(default_factory=list)
    distance_errors_m: list[float] = field(default_factory=list)
    _currently_lost: bool = False
    _lost_at_frame: int | None = None

    def __post_init__(self) -> None:
        if self.desired_distance_m <= 0:
            raise ValueError("desired distance must be positive")

    def add(self, observation: TrackingObservation) -> None:
        self.total_frames += 1

        if observation.locked and observation.bbox is not None:
            self.locked_frames += 1
            self.center_errors_px.append(observation.bbox.center_error_px(self.frame))
            if observation.estimated_distance_m is not None:
                self.distance_errors_m.append(
                    abs(observation.estimated_distance_m - self.desired_distance_m)
                )
            if self._currently_lost and self._lost_at_frame is not None:
                self.recovery_frames.append(observation.frame_index - self._lost_at_frame)
                self._currently_lost = False
                self._lost_at_frame = None
            return

        if not self._currently_lost:
            self.lost_events += 1
            self._currently_lost = True
            self._lost_at_frame = observation.frame_index

    @property
    def tracking_success_rate(self) -> float:
        if self.total_frames == 0:
            return 0.0
        return self.locked_frames / self.total_frames

    @property
    def average_center_error_px(self) -> float:
        return fmean(self.center_errors_px) if self.center_errors_px else 0.0

    @property
    def average_recovery_frames(self) -> float:
        return fmean(self.recovery_frames) if self.recovery_frames else 0.0

    @property
    def average_distance_error_m(self) -> float:
        return fmean(self.distance_errors_m) if self.distance_errors_m else 0.0

    def summary(self) -> dict[str, float | int]:
        return {
            "total_frames": self.total_frames,
            "locked_frames": self.locked_frames,
            "lost_events": self.lost_events,
            "tracking_success_rate": self.tracking_success_rate,
            "average_center_error_px": self.average_center_error_px,
            "average_recovery_frames": self.average_recovery_frames,
            "average_distance_error_m": self.average_distance_error_m,
        }
