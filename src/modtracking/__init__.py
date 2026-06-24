"""Core primitives for a pure object tracking and UAV following system."""

from modtracking.controllers import DroneCommand, GimbalCommand, GimbalController, PursuitController
from modtracking.metrics import TrackingMetrics
from modtracking.models import BoundingBox, FrameSize, TrackingObservation
from modtracking.state_machine import TrackingState, TrackingStateMachine
from modtracking.trackers import TrackerBackend

__all__ = [
    "BoundingBox",
    "DroneCommand",
    "FrameSize",
    "GimbalCommand",
    "GimbalController",
    "PursuitController",
    "TrackingMetrics",
    "TrackingObservation",
    "TrackingState",
    "TrackingStateMachine",
    "TrackerBackend",
]
