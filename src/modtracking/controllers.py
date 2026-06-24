"""Control helpers for gimbal centering and autonomous following."""

from __future__ import annotations

from dataclasses import dataclass

from modtracking.models import FrameSize, TrackingObservation


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


@dataclass(frozen=True)
class GimbalCommand:
    """Gimbal angular-rate command in normalized units or rad/s by integration."""

    yaw_rate: float = 0.0
    pitch_rate: float = 0.0


@dataclass(frozen=True)
class DroneCommand:
    """Body-frame pursuit command.

    ``forward_mps`` is positive forward, ``right_mps`` is positive right,
    ``up_mps`` is positive upward, and ``yaw_rate`` is positive clockwise/right.
    """

    forward_mps: float = 0.0
    right_mps: float = 0.0
    up_mps: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class GimbalController:
    """Centers the selected target in the camera frame before aircraft motion."""

    yaw_gain: float = 0.8
    pitch_gain: float = 0.8
    deadband_norm: float = 0.03
    max_yaw_rate: float = 1.0
    max_pitch_rate: float = 1.0

    def update(self, observation: TrackingObservation, frame: FrameSize) -> GimbalCommand:
        if observation.bbox is None:
            return GimbalCommand()

        target_x, target_y = observation.bbox.center
        center_x, center_y = frame.center
        x_error = _deadband((target_x - center_x) / (frame.width / 2.0), self.deadband_norm)
        y_error = _deadband((target_y - center_y) / (frame.height / 2.0), self.deadband_norm)

        return GimbalCommand(
            yaw_rate=_clamp(self.yaw_gain * x_error, self.max_yaw_rate),
            pitch_rate=_clamp(-self.pitch_gain * y_error, self.max_pitch_rate),
        )


@dataclass(frozen=True)
class PursuitController:
    """Converts tracking observations into smooth body-frame UAV commands."""

    desired_distance_m: float = 10.0
    distance_deadband_m: float = 0.75
    center_deadband_norm: float = 0.05
    forward_gain: float = 0.35
    lateral_gain: float = 0.8
    vertical_gain: float = 0.5
    yaw_gain: float = 0.6
    max_forward_mps: float = 3.0
    max_lateral_mps: float = 1.5
    max_vertical_mps: float = 1.0
    max_yaw_rate: float = 0.8

    def __post_init__(self) -> None:
        if self.desired_distance_m <= 0:
            raise ValueError("desired distance must be positive")

    def update(self, observation: TrackingObservation, frame: FrameSize) -> DroneCommand:
        if observation.bbox is None:
            return DroneCommand()

        target_x, target_y = observation.bbox.center
        center_x, center_y = frame.center
        x_error = _deadband((target_x - center_x) / (frame.width / 2.0), self.center_deadband_norm)
        y_error = _deadband((target_y - center_y) / (frame.height / 2.0), self.center_deadband_norm)

        distance_error = 0.0
        if observation.estimated_distance_m is not None:
            distance_error = _deadband(
                observation.estimated_distance_m - self.desired_distance_m,
                self.distance_deadband_m,
            )

        return DroneCommand(
            forward_mps=_clamp(self.forward_gain * distance_error, self.max_forward_mps),
            right_mps=_clamp(self.lateral_gain * x_error, self.max_lateral_mps),
            up_mps=_clamp(-self.vertical_gain * y_error, self.max_vertical_mps),
            yaw_rate=_clamp(self.yaw_gain * x_error, self.max_yaw_rate),
        )
