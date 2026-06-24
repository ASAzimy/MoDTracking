"""State machine for manual flight, tracking, pursuit, and recovery."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class TrackingState(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    MANUAL = "MANUAL"
    TARGET_SELECTED = "TARGET_SELECTED"
    TRACKING = "TRACKING"
    PURSUIT = "PURSUIT"
    LOST_TARGET = "LOST_TARGET"
    REACQUIRE = "REACQUIRE"


ALLOWED_TRANSITIONS: dict[TrackingState, set[TrackingState]] = {
    TrackingState.IDLE: {TrackingState.TAKEOFF},
    TrackingState.TAKEOFF: {TrackingState.MANUAL, TrackingState.IDLE},
    TrackingState.MANUAL: {TrackingState.TARGET_SELECTED, TrackingState.IDLE},
    TrackingState.TARGET_SELECTED: {TrackingState.TRACKING, TrackingState.MANUAL},
    TrackingState.TRACKING: {
        TrackingState.PURSUIT,
        TrackingState.LOST_TARGET,
        TrackingState.MANUAL,
        TrackingState.IDLE,
    },
    TrackingState.PURSUIT: {
        TrackingState.TRACKING,
        TrackingState.LOST_TARGET,
        TrackingState.MANUAL,
        TrackingState.IDLE,
    },
    TrackingState.LOST_TARGET: {TrackingState.REACQUIRE, TrackingState.MANUAL, TrackingState.IDLE},
    TrackingState.REACQUIRE: {
        TrackingState.TRACKING,
        TrackingState.LOST_TARGET,
        TrackingState.MANUAL,
        TrackingState.IDLE,
    },
}


@dataclass
class TrackingStateMachine:
    """Strict transition manager for UAV tracking mode changes."""

    state: TrackingState = TrackingState.IDLE
    history: list[TrackingState] = field(default_factory=lambda: [TrackingState.IDLE])

    def can_transition(self, next_state: TrackingState) -> bool:
        return next_state in ALLOWED_TRANSITIONS[self.state]

    def transition(self, next_state: TrackingState) -> TrackingState:
        if not self.can_transition(next_state):
            raise ValueError(f"invalid transition: {self.state.value} -> {next_state.value}")
        self.state = next_state
        self.history.append(next_state)
        return self.state

    def takeoff_complete(self) -> TrackingState:
        if self.state == TrackingState.IDLE:
            self.transition(TrackingState.TAKEOFF)
        return self.transition(TrackingState.MANUAL)

    def target_selected(self) -> TrackingState:
        return self.transition(TrackingState.TARGET_SELECTED)

    def tracker_locked(self) -> TrackingState:
        return self.transition(TrackingState.TRACKING)

    def run_pursuit(self) -> TrackingState:
        return self.transition(TrackingState.PURSUIT)

    def target_lost(self) -> TrackingState:
        return self.transition(TrackingState.LOST_TARGET)

    def begin_reacquire(self) -> TrackingState:
        return self.transition(TrackingState.REACQUIRE)

    def manual_override(self) -> TrackingState:
        return self.transition(TrackingState.MANUAL)

    def landed(self) -> TrackingState:
        return self.transition(TrackingState.IDLE)
