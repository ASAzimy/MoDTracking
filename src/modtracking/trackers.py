"""Interfaces for pure tracker backends initialized by manual selection."""

from __future__ import annotations

from typing import Any, Protocol

from modtracking.models import BoundingBox, TrackingObservation


class TrackerBackend(Protocol):
    """Protocol implemented by CSRT, MixFormer, OSTrack, or other pure trackers."""

    name: str

    def initialize(self, frame: Any, selection: BoundingBox) -> None:
        """Lock onto the user-drawn target box in the current frame."""

    def update(self, frame: Any, frame_index: int) -> TrackingObservation:
        """Return the next target observation without running object detection."""
