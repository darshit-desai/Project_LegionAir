# Static Transform Broadcaster Node Documentation

## Overview

This node publishes static transforms between predefined frames using the ROS 2 TF2 framework. It defines transformations between a parent and a child frame using translation and rotation (in roll-pitch-yaw format). The transforms are published repeatedly at a fixed interval.

---

## Features

1. **Static Transform Broadcasting**:
   - Publishes static transforms between frames such as `body` to `world` and `body` to `hires`.
   - Uses roll-pitch-yaw (RPY) angles to define rotations between frames.

2. **Periodic Publishing**:
   - Re-publishes transforms every 100ms to ensure availability in the TF2 tree.

3. **Customizable Transforms**:
   - Translations and rotations for each frame pair are defined in the `ExtrinsicTransform` structure, making it easy to add or modify transformations.

---

## Code Structure

### Struct: `ExtrinsicTransform`
Defines the relationship between a parent and child frame:
- **`parent`**: The parent frame.
- **`child`**: The child frame.
- **`T_child_wrt_parent`**: Translation vector [x, y, z] in meters.
- **`RPY_parent_to_child`**: Rotation (roll, pitch, yaw) in degrees.

### Function: `publish_static_transforms`
- Creates and sends static transforms to the TF2 tree.
- Converts roll-pitch-yaw angles to quaternions using Eigen's rotation utilities.
- Uses `geometry_msgs::msg::TransformStamped` to define and publish each transform.

### Main Function
1. **Node Initialization**:
   - Creates the `tf_publisher_cpp_node` node.
   - Initializes the static transform broadcaster.

2. **Timer Setup**:
   - Creates a wall timer to call `publish_static_transforms` every 100ms.

3. **Spin Loop**:
   - Spins the node to keep it alive and broadcasting.

---

## Transforms

The node currently defines the following transforms:

1. **Body to World**:
   - **Translation**: `[0.068, 0.0116, 0.0168]` meters.
   - **Rotation (RPY)**: `[0, 270, 0]` degrees.

2. **Body to Hires**:
   - **Translation**: `[0.068, -0.012, 0.015]` meters.
   - **Rotation (RPY)**: `[90, 270, 0]` degrees.

---

## Example Usage

### Running the Node
Launch the node with the following command:
```bash
ros2 run <package_name> tf_publisher_cpp_node
```

### TF2 Tree
After launching the node, the following transforms will be visible:
- `world` -> `body`
- `body` -> `hires`

---

## Adding or Modifying Transforms
To add new transforms or modify existing ones:
1. Update the `extrinsics` vector in the `publish_static_transforms` function.
2. Define the new `parent` and `child` frames.
3. Specify the translation and rotation values.

---

## Limitations
- Only static transforms are supported.
- Transformations are periodically re-published, which may not be necessary for all static TF2 setups.

---

## Future Improvements
- Add parameterization for defining transforms dynamically via a configuration file or ROS parameters.
- Support additional frame relationships as required by the application.

---

## Authors
- **Shivam Sehgal**

---

## License
Copyright (c) 2024.
