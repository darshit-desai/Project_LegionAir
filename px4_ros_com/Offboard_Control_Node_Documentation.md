# Offboard Control Node Documentation

## Overview

The provided code implements an **Offboard Control Node** in ROS 2 for controlling a drone using the PX4 flight stack. The node leverages PX4's offboard mode to send trajectory setpoints and velocity commands. Additionally, it subscribes to object detection topics and adjusts the drone's motion based on detected object positions.

---

## Key Features
1. Publishes **OffboardControlMode** messages to configure control settings.
2. Publishes **TrajectorySetpoint** messages to command velocity or position trajectories.
3. Publishes **VehicleCommand** messages to arm, disarm, or change the vehicle's mode.
4. Subscribes to:
   - **`detections` topic**: Receives object positions in the form of `geometry_msgs/PointStamped`.
   - **`object_available` topic**: Determines whether an object is detected (`std_msgs/Bool`).
5. Adjusts drone velocity based on object position and velocity using proportional and derivative gains.

---

## Code Breakdown

### Class: `OffboardControl`
This class handles all operations related to offboard control, including message publishing, subscribing, and drone command logic.

### Constructor
- Initializes publishers and subscribers.
- Defines timer to periodically send setpoints and vehicle commands.
- Logs the namespace for debugging.

#### Topics and Publishers:
- **OffboardControlMode**: Configures control parameters (e.g., velocity control is enabled).
- **TrajectorySetpoint**: Sends velocity commands to the drone.
- **VehicleCommand**: Sends arm, disarm, and mode change commands.

#### Subscriptions:
- **`detections`**:
  - Receives the detected object’s position (`PointStamped`).
  - Updates object coordinates (`object_position_x_`, `object_position_y_`, `object_position_z_`).
- **`object_available`**:
  - Indicates if an object is available for tracking (`Bool`).

### Main Functions

#### `arm()` and `disarm()`
Commands to arm or disarm the vehicle using the `VehicleCommand` message.

#### `publish_offboard_control_mode()`
Publishes the control mode for the drone. The node is configured to enable velocity control.

#### `publish_trajectory_setpoint()`
Publishes a trajectory setpoint based on detected object data.
- Proportional (`kpx`, `kpy`, `kpz`) and derivative (`kdx`, `kdy`, `kdz`) gains are used to compute velocity commands based on:
  - Object position.
  - Object velocity (calculated using time difference).
- If no object is available, velocity commands are set to zero.

#### `publish_vehicle_command()`
Sends commands to the flight controller. Used for mode changes and arming/disarming.

#### Timer Callback
Executed every 33ms (~30Hz):
- Publishes `OffboardControlMode` and `TrajectorySetpoint` messages.
- Arms the drone and switches to offboard mode after 10 setpoints.

#### Callbacks
1. **`detectionsCallback()`**:
   - Updates the object’s position when a new detection is received.
2. **`objectAvailableCallback()`**:
   - Updates the `object_available_` flag based on whether an object is detected.

---

## How It Works
1. The node initializes and sets up publishers and subscribers.
2. A timer periodically sends offboard control mode and trajectory setpoint messages.
3. When an object is detected, its position and velocity are used to compute velocity commands using PD control.
4. Commands are sent to the drone to move it towards the detected object.
5. The drone switches to offboard mode and arms itself after 10 setpoints.
6. If no object is available, the drone halts by sending zero velocity commands.

---

## Parameters
- **Proportional Gains**:
  - `kpx`, `kpy`, `kpz`: Define the responsiveness to positional errors.
- **Derivative Gains**:
  - `kdx`, `kdy`, `kdz`: Define the responsiveness to velocity errors.

---

## Key Points
- The node assumes PX4 is running in a compatible mode and the required topics are available.
- Velocity commands are relative to the detected object's position.
- The object tracking logic uses simple proportional-derivative control.
- Ensure proper ROS 2 environment setup to run the node.

---

## Example Usage
1. Launch the node with the required namespace:
   ```bash
   ros2 launch px4_ros_com offboard_node.launch.py
   ```
2. Ensure the `detections` and `object_available` topics are publishing data.
3. Monitor the drone's movement and logs for debugging.

---

## Future Improvements
- Add integral control for better handling of persistent errors.
- Enhance robustness by adding failsafe mechanisms.
- Support multiple detected objects and select the target dynamically.
