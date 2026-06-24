# MoDTracking

## Vision-Based Pure Object Tracking and Autonomous Following System for UAV

MoDTracking is scoped as a **pure object tracking and following UAV system**.
It is not a general object detector and it is not an AI surveillance platform.
The operator manually selects a target in the live camera image, and the system
tracks and follows only that selected object.

## Core Goal

The system should allow an operator to draw a bounding box around any visible
object and then command the UAV to:

1. Track the selected object continuously.
2. Keep the target near the center of the camera frame.
3. Follow the target autonomously when pursuit is enabled.
4. Maintain a safe standoff distance of about 10 meters.
5. Recover tracking after fast motion or partial occlusion.
6. Switch safely between manual control and autonomous tracking modes.

Automatic object detection is intentionally out of scope for target selection.
The target is selected only by the user's bounding box.

## Operating Workflow

### 1. Manual Flight

The operator launches the GUI and controls the drone directly. The interface is
expected to show:

- Live HD camera feed
- Takeoff and land controls
- Forward, backward, left, right controls
- Up and down controls
- Yaw left and yaw right controls

### 2. Target Selection

The operator draws a box around any visible object, such as a person, vehicle,
boat, animal, or arbitrary object. When the mouse button is released:

- The selected box initializes the tracker.
- The target becomes locked.
- The system transitions from manual flight to tracking mode.

### 3. Pure Object Tracking

The tracker estimates target position, velocity, and motion direction from frame
to frame. Candidate trackers for research comparison include:

- CSRT
- MixFormer
- OSTrack

Other pure tracking backends, such as KCF, MOSSE, NanoTrack, or future research
trackers, can be added behind the same tracker interface.

### 4. Gimbal Tracking

Gimbal motion should be the first response to target image motion:

- Target left: rotate gimbal left.
- Target right: rotate gimbal right.
- Target up: pitch gimbal upward.
- Target down: pitch gimbal downward.

The goal is to keep the selected target near the image center before commanding
larger UAV body movement.

### 5. Autonomous Following

After tracking is stable, the operator can press **RUN PURSUIT**. During pursuit:

- Horizontal image error produces lateral motion.
- Range error produces forward or backward motion.
- Centering error produces yaw correction.
- The desired range is approximately 10 meters.
- Commands should be smooth and bounded to avoid oscillation or aggressive motion.

## State Machine

```text
IDLE
  |
TAKEOFF
  |
MANUAL
  |
TARGET_SELECTED
  |
TRACKING
  |
PURSUIT
  |
LOST_TARGET
  |
REACQUIRE
  |
TRACKING
```

The implementation exposes this flow through `TrackingStateMachine`. Manual
override and landing paths are safety-critical and should remain available from
airborne states.

## Repository Structure

```text
src/modtracking/
  controllers.py     Gimbal and UAV pursuit command helpers
  metrics.py         Tracking and following evaluation metrics
  models.py          Frame, bounding-box, and tracker-observation models
  state_machine.py   Manual/tracking/pursuit/recovery state machine
  trackers.py        Protocol for pure tracker backend implementations
tests/
  test_*.py          Focused unit tests for the core behavior
```

The current code is dependency-free and simulator-agnostic. ROS, Gazebo,
ArduPilot SITL, MAVLink, OpenCV, and deep tracker integrations can be added as
adapters around these core interfaces.

## Research Objectives

Experiments should report:

- **Tracking accuracy:** how well the tracker box remains on the selected target.
- **Tracking success rate:** percentage of frames where the target remains locked.
- **Center error:** distance between the target center and image center.
- **Recovery time:** frames or seconds required to recover after occlusion or loss.
- **Following accuracy:** error between actual range and the 10 meter target range.

## Expected Demonstration

1. Launch Gazebo.
2. Launch ArduPilot SITL.
3. Open the tracking GUI.
4. Take off to 5 meters.
5. Fly manually.
6. Draw a box around a moving object.
7. Confirm the tracker locks onto the selected object.
8. Press **RUN PURSUIT**.
9. Confirm the drone follows while keeping the target centered.
10. Press **LAND** and land safely.

## Development

Run the unit tests with:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests
```
