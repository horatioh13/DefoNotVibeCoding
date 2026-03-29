# Sulis: The Omniscient Autonomous Workbench Assistant

Sulis is a hackathon-built robotic workbench assistant designed to keep an engineering desk from descending into chaos. Using overhead vision, a gantry system, and a small multi-servo arm, it can detect tools left out on the bench, move towards them, and attempt to pick them up with a compliant gripper. The same platform can also act as a general lab assistant for manual positioning, tool fetching, and future light assembly tasks.

Built for BathHack 2026, the project combines scavenged printer hardware, ROS 2, GRBL, servo control, YOLO-based object detection, and a custom OpenGL/ImGui control interface.

## Inspiration

The second law of hackathons says the entropy of a workbench always increases. Sulis is our attempt to fight that law with robotics, computer vision, and a slightly unreasonable amount of determination.

## What It Does

- Detects tools and electronics on the workbench using YOLO models
- Displays live camera feeds in a custom Python GUI
- Lets the user command the gantry manually through the GUI
- Publishes homing and motion commands to the hardware controller over ROS 2
- Drives a 2-axis GRBL gantry plus a 4-servo end effector
- Supports joystick-driven teleoperation for the arm joints
- Saves detected object centers to timestamped text files for later targeting

## Hardware Overview

The physical system described by the code and project notes is:

- A 2-axis gantry salvaged from an Ender 3-style printer frame
- Belt-driven stepper axes controlled by an Arduino Uno + CNC shield running GRBL v1.1
- A 3-DOF arm made from metal gear servos and custom 3D-printed joints
- A compliant TPU gripper actuated by a small SG90 servo
- A PCA9685 servo board for arm and gripper control
- Raspberry Pi cameras for a top-down bench view and an end-effector view
- A Raspberry Pi 5 / Linux host running ROS 2 nodes and the GUI

## Software Overview

This repo is split into two ROS 2 Python packages plus a lightweight launcher:

- `mission_control`
  - Custom OpenGL + Dear ImGui GUI
  - Camera feed display
  - Manual gantry target selection
  - Gamepad teleop publisher
  - YOLO model assets and a standalone live detection script
- `sulis`
  - Hardware-facing ROS 2 node for the servos and GRBL gantry
  - GRBL controller helpers
  - Experimental gantry bridge and serial test scripts
- `launch.py`
  - Starts the GUI and ROS joystick node together

## How The Repo Works

### Main control path

1. The GUI node subscribes to two ROS image topics and renders them in an OpenGL dashboard.
2. The GUI publishes:
   - `/Gantry` as a `Float32MultiArray` target position
   - `/Home` as a `Bool` trigger
3. The `sulis.actuator` node subscribes to:
   - `/EffectorPositions` for shoulder, elbow, wrist, and gripper angles
   - `/Gantry` for absolute gantry moves
   - `/Home` for GRBL homing
4. `sulis.actuator` drives:
   - the PCA9685 servo board through `adafruit_servokit`
   - the gantry through a serial GRBL controller on `/dev/ttyUSB0`

### Detection path

The current object detection workflow in this repo is separate from the GUI render loop:

1. `mission_control/ncnn_model/open_live_preview.py` opens a webcam feed with OpenCV.
2. It runs two YOLO models in parallel:
   - `toolsbest.pt`
   - `electronicsbest.pt`
3. Pressing `c` saves the detections for the current frame to a `detections_*.txt` file.
4. The GUI's "Home to Lasted Object" button reads the newest detection file and uses the saved X coordinate to move the targeting slider.

Despite the `ncnn_model` directory name, the current preview script loads `.pt` weights with Ultralytics YOLO.

### Teleoperation path

`mission_control.teleop` reads the `/joy` topic and publishes:

- `/EffectorPositions` for the arm joints
- `/servo_node/delta_joint_cmds` as a `JointJog` message for gantry jogging

There is also a `sulis/gantry.py` bridge intended for MoveIt Servo / `JointJog` control, but the most complete path in this snapshot is the direct `/Gantry` route handled by `sulis.actuator`.

## Repository Layout

```text
.
|-- launch.py
|-- mission_control/
|   |-- launch/mission_launch.py
|   |-- mission_control/
|   |   |-- control_gui.py
|   |   |-- teleop.py
|   |   |-- gui/
|   |   |-- config/
|   |   `-- stream/
|   `-- ncnn_model/
|       |-- open_live_preview.py
|       |-- toolsbest.pt
|       `-- electronicsbest.pt
`-- sulis/
    |-- sulis/
    |   |-- actuator.py
    |   |-- grbl_controller.py
    |   |-- gantry.py
    |   |-- controller.py
    |   |-- main.py
    |   `-- example.py
    `-- test/
```

## Key Files

- `launch.py`
  - Launches the GUI and `joy_node`
- `mission_control/mission_control/control_gui.py`
  - Main dashboard, camera handling, manual gantry controls, detection script launcher
- `mission_control/mission_control/teleop.py`
  - Maps controller buttons and triggers to effector and gantry commands
- `mission_control/ncnn_model/open_live_preview.py`
  - Standalone dual-model object detection preview and capture logger
- `sulis/sulis/actuator.py`
  - Main hardware node for servos, homing, and direct gantry motion
- `sulis/sulis/grbl_controller.py`
  - Serial GRBL background thread with jogging, homing, and position polling
- `sulis/sulis/main.py`
  - Keyboard-based direct GRBL test script
- `sulis/sulis/example.py`
  - Minimal example of scripted gantry movement

## Dependencies

This project expects a ROS 2 Python workspace and several non-default Python packages.

### ROS 2 packages

- `rclpy`
- `launch`
- `launch_ros`
- `sensor_msgs`
- `std_msgs`
- `control_msgs`
- `trajectory_msgs`
- `ffmpeg_image_transport_msgs`
- `joy`

### Python packages used in the code

- `pyserial`
- `numpy`
- `opencv-python`
- `av`
- `ultralytics`
- `PyOpenGL`
- `glfw`
- `imgui[full]`
- `adafruit-circuitpython-servokit`

The GUI notes in `mission_control/mission_control/gui/IMGUI.md` also mention adding a virtual environment with `imgui[full]` to `PYTHONPATH`, and installing extra build-time packages such as `pyyaml`, `typeguard`, and `empy`.

## Setup

### 1. Build the ROS 2 workspace

From the workspace root:

```bash
colcon build
source install/setup.bash
```

If the GUI cannot import ImGui after building, create a Python virtual environment, install `imgui[full]`, and add that venv's `site-packages` directory to `PYTHONPATH` before sourcing the workspace.

### 2. Connect the hardware

The current code assumes:

- GRBL is available on `/dev/ttyUSB0`
- The servo board is reachable through `ServoKit(channels=16)`
- ROS image topics are published at:
  - `/picam_ros2/camera_80000/imx708_wide`
  - `/picam_ros2/camera_88000/imx708_wide`
- The object-detection preview camera is OpenCV camera index `1`

You will likely need to adjust these values for a different machine.

## Running The Project

### Quick demo path

Start the hardware node.

Because `sulis/setup.py` currently has a broken `console_scripts` entry, the safest development command in this repo snapshot is:

```bash
python3 sulis/sulis/actuator.py
```

In another terminal, start the GUI and joystick node:

```bash
python3 launch.py
```

If you want controller-driven arm commands as well, run:

```bash
ros2 run mission_control teleop
```

### Object detection preview

Run the standalone detection window:

```bash
python3 mission_control/ncnn_model/open_live_preview.py --path mission_control/ncnn_model
```

The preview script currently forces both YOLO models onto `cuda:0`. If you are running on CPU only, edit `open_live_preview.py` and remove or change those `.to('cuda:0')` calls first.

Controls:

- Press `c` to save the current detections to a timestamped text file
- Press `q` to quit

## GUI Controls

The GUI currently provides:

- Live main and end-effector camera panes
- Manual X/Y target sliders
- Direct numeric X/Y input
- A simple 2D canvas showing the current target point
- A `Begin Homing Sequence` button
- An `Open Object Detect` button
- A `Home to Lasted Object` button that reads the newest saved detection file

Internally, the GUI maps the displayed slider values onto gantry coordinates before publishing them:

- `x_mm = 560 - x * 5.6`
- `y_mm = 250 - y * 2.5`

## Controller Mapping

From `mission_control/mission_control/teleop.py`:

- `Triangle` selects shoulder control
- `Circle` selects elbow control
- `Cross` selects wrist control
- `Square` selects gripper control
- Triggers increment or decrement the selected joint target
- Left stick commands gantry jogging

## Known Limitations And Repo Notes

- The codebase is clearly mid-hackathon and a few parts are experimental or unfinished.
- `mission_control/launch/mission_launch.py` references `base` and `connection_server`, but those nodes are not fully present/exported in this repo snapshot.
- `control_gui.py` imports `FFMPEGPacket`, although the ffmpeg transport subscribers are currently commented out.
- `sulis/setup.py` has an unfinished `console_scripts` list formatting issue, so package entry points may need cleanup for a production-ready build.
- `sulis/gantry.py` appears to be an alternate or in-progress bridge for MoveIt Servo control.
- Inverse kinematics are not implemented yet; the arm is currently driven by direct angle commands.
- Detection files are read from the current working directory, so the GUI's saved-object targeting depends on where the preview script was launched from.
- Several hardcoded ports, topics, camera indices, and device paths will need adapting for another setup.

## Challenges

- Setting up GRBL and CNC shield axis configuration
- Getting ROS 2 packages and hardware interfaces to cooperate reliably
- Assembling the printed arm in time for the hackathon
- Working around camera issues and incomplete calibration
- Reaching a usable manipulation demo without inverse kinematics

## Accomplishments

- It moves
- It grips, mostly
- It detects tools
- It can move towards a detected object
- It ties together vision, UI, gantry control, and end-effector control in one demo

## What We Learned

- Robot arms are hard
- ROS 2 can be painful
- GRBL is unforgiving
- Cheap hobby servos are only helpful right up until they are not

## What's Next

- Finish inverse kinematics
- Run the vision model locally on the Pi
- Add reliable voice control
- Calibrate the cameras properly
- Improve grasping and repeatability
- Turn the current hackathon prototype into a more robust workbench robot

## Status

This repository is best described as a hackathon prototype with working subsystems rather than a polished product. The most important parts are present and understandable, and the code already demonstrates the full idea: detect clutter, visualise it, and move hardware in response.
