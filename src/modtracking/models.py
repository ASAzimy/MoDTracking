"""Shared data models for manual selection and tracker output."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True)
class FrameSize:
    """Camera frame dimensions in pixels."""

    width: int
    height: int

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("frame width and height must be positive")

    @property
    def center(self) -> tuple[float, float]:
        return (self.width / 2.0, self.height / 2.0)


@dataclass(frozen=True)
class BoundingBox:
    """Axis-aligned target box in image coordinates."""

    x: float
    y: float
    width: float
    height: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("bounding box width and height must be positive")

    @classmethod
    def from_xyxy(cls, left: float, top: float, right: float, bottom: float) -> "BoundingBox":
        return cls(x=left, y=top, width=right - left, height=bottom - top)

    @property
    def center(self) -> tuple[float, float]:
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)

    @property
    def area(self) -> float:
        return self.width * self.height

    def center_error_px(self, frame: FrameSize) -> float:
        target_x, target_y = self.center
        frame_x, frame_y = frame.center
        return hypot(target_x - frame_x, target_y - frame_y)


@dataclass(frozen=True)
class TrackingObservation:
    """Single-frame output from a tracker backend.

    A missing bounding box represents a lost target. Distance is optional because
    pure visual trackers may need a rangefinder, stereo camera, or calibrated
    target-size estimate to infer range.
    """

    frame_index: int
    bbox: BoundingBox | None
    confidence: float = 1.0
    estimated_distance_m: float | None = None

    def __post_init__(self) -> None:
        if self.frame_index < 0:
            raise ValueError("frame index cannot be negative")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        if self.estimated_distance_m is not None and self.estimated_distance_m <= 0:
            raise ValueError("estimated distance must be positive")

    @property
    def locked(self) -> bool:
        return self.bbox is not None and self.confidence > 0.0
