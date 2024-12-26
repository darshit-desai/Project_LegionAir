# Point Cloud Transformer Node Documentation

## Overview

This node processes point cloud data from a TOF (Time-of-Flight) camera, transforms it from the camera frame to the body frame using TF transformations, and publishes the transformed point cloud. The transformation includes filtering and applying affine transformations derived from translation and rotation matrices.

---

## Features

1. **Point Cloud Processing**:
   - Subscribes to point cloud data from a TOF camera.
   - Filters point cloud data by z-coordinate to remove points outside the specified depth range.

2. **TF Transformations**:
   - Uses ROS 2's TF2 library to retrieve the transformation between the source (TOF camera) and target frames.
   - Applies translation and rotation transformations to the filtered point cloud.

3. **Publishing**:
   - Publishes the transformed point cloud in the `hires` frame to a ROS topic.

---

## Code Structure

### Class: `PointCloudTransformer`
This class is responsible for subscribing to point cloud data, applying transformations, and publishing the processed point cloud.

#### Constructor
- Initializes the TF2 buffer and listener.
- Sets up ROS publishers and subscribers.
- Logs the namespace for easier debugging.

#### Subscribers
- **`<namespace>/tof_pc`**:
  - Subscribes to raw point cloud data from the TOF camera.

#### Publishers
- **`rgb_pcl`**:
  - Publishes the transformed point cloud.

#### Key Member Variables
- **`translation_vector`**: Stores the translation component of the transformation.
- **`rotation_matrix`**: Stores the rotation matrix derived from the quaternion.
- **`tf_buffer_` and `tf_listener_`**: Used to fetch TF2 transformations.

---

## Workflow

### 1. Subscribing to Point Cloud Data
- The node subscribes to the `<namespace>/tof_pc` topic for point cloud data in the TOF camera frame.
- The data is converted from a ROS PointCloud2 message to a PCL point cloud for further processing.

### 2. Filtering
- Points are filtered based on their z-coordinate (depth) to retain points within the range of 0.4 to 1.6 meters.

### 3. Transformation
- The node retrieves the transformation from the `hires` frame to the TOF camera frame using the TF2 buffer.
- Translation and rotation components are extracted from the transformation.
- A transformation matrix is constructed and applied to the filtered point cloud.

### 4. Publishing Transformed Point Cloud
- The transformed point cloud is converted back to a ROS PointCloud2 message and published to the `rgb_pcl` topic in the `hires` frame.

---

## Example Usage

### Running the Node
Launch the node with the following command:
```bash
ros2 run <package_name> pc_transformer --ros-args -r __ns:=<namespace>
```

### Input and Output Topics
- **Input**:
  - `<namespace>/tof_pc`: Raw point cloud data from the TOF camera.
- **Output**:
  - `rgb_pcl`: Transformed point cloud in the `hires` frame.

---

## Parameters

### Hardcoded Parameters
- **Z-Coordinate Filtering**:
  - Minimum: `0.4` meters.
  - Maximum: `1.6` meters.

---

## Limitations
- The node assumes the `hires` frame and TOF camera frame transformations are available via TF2.
- Filtering parameters are hardcoded and may require modification for different datasets or environments.

---

## Future Improvements
- Add dynamic parameter configuration for filtering thresholds.
- Improve error handling for missing or invalid TF2 transformations.
- Extend support for multiple point cloud inputs and transformations.

---

## Authors
- **Shivam Sehgal** (ssehgal7@umd.edu)
- **Darshit Desai** (darshit@umd.edu)

---

## Copyright
Copyright (c) 2024.
