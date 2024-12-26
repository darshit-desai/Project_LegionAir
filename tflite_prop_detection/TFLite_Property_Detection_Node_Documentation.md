# TFLite Property Detection Node Documentation

## Overview

This node processes bounding box detections from a TensorFlow Lite model and PointCloud2 data to compute and publish the centroid of detected objects. It subscribes to the following topics:

1. `/tflite_data` - Provides bounding box detections.
2. `/rgb_pcl` - Provides PointCloud2 data in the camera frame.

The node publishes the computed centroid and whether an object is detected. It uses Eigen and PCL libraries for point cloud processing and matrix operations.

---

## Key Features
1. Subscribes to **`/tflite_data`**:
   - Reads bounding box coordinates and detection confidence.
   - Publishes whether an object is detected using `std_msgs/Bool`.
2. Subscribes to **`/rgb_pcl`**:
   - Processes PointCloud2 data to extract points within the bounding box.
   - Projects points into the image plane using intrinsic camera parameters.
   - Computes the centroid of filtered points.
3. Publishes:
   - **Centroid**: `geometry_msgs/PointStamped`.
   - **Object Availability**: `std_msgs/Bool`.

---

## Code Breakdown

### Class: `TFLitePropDetectionNode`
Handles all operations, including subscribing to topics, processing detections and point clouds, and publishing results.

#### Constructor
- Initializes publishers and subscribers.
- Configures intrinsic camera matrices (`K_pcl_` and `K_`) for point projection.
- Sets default bounding box values and node parameters.

#### Publishers
- **`detections`** (`geometry_msgs/PointStamped`): Publishes the 3D centroid of detected objects.
- **`object_available`** (`std_msgs/Bool`): Indicates if an object is detected.

#### Subscribers
- **`/tflite_data`**:
  - Receives bounding box coordinates (`x_min`, `x_max`, `y_min`, `y_max`) and confidence scores.
  - Updates bounding box values and publishes object availability.
- **`/rgb_pcl`**:
  - Processes PointCloud2 data.
  - Projects points into the image plane using intrinsic parameters.
  - Filters points within the bounding box.
  - Computes the centroid of filtered points.

### Main Functions

#### `aidectionCallback()`
- Receives bounding box data and updates bounding box values if confidence exceeds a threshold.
- Publishes `object_available` if valid bounding box data exists.

#### `pclCallback()`
- Converts PointCloud2 data to PCL format.
- Projects points into the image plane using the camera's intrinsic matrix (`K_pcl_`).
- Filters points within the bounding box.
- Computes the centroid of filtered points and transforms it into the FRD (Forward-Right-Down) frame.
- Publishes the centroid and updates `object_available`.

#### Camera Intrinsics
- **`K_pcl_`**: Used for projecting 3D points onto the image plane.
- **`K_`**: Used for transforming 2D image points back into 3D coordinates.

#### Timer Logic
- Resets bounding box values and sets `object_available` to `false` if no detections are received within 0.07 seconds.

---

## How It Works
1. The node subscribes to `/tflite_data` for bounding box detections and `/rgb_pcl` for PointCloud2 data.
2. Bounding box data is updated based on detection confidence and published as `object_available`.
3. PointCloud2 data is:
   - Projected onto the image plane.
   - Filtered to extract points within the bounding box.
   - Used to compute the centroid of the detected object.
4. The centroid is transformed to the FRD frame and published.
5. If no detections occur within 0.07 seconds, the node resets bounding box values and sets `object_available` to `false`.

---

## Key Points
- Bounding box and point cloud processing occurs in real-time.
- Uses a camera's intrinsic matrix for projection and filtering.
- Ensures robust detection by resetting values when no detections are received.

---

## Parameters
- **Intrinsic Camera Matrices**:
  - `K_pcl_`: Projects 3D points into the 2D image plane.
  - `K_`: Transforms image points back into 3D coordinates.
- **Bounding Box Values**:
  - `bbox_x_min`, `bbox_x_max`, `bbox_y_min`, `bbox_y_max`.

---

## Example Usage
1. Launch the node with the required namespace:
   ```bash
   ros2 run <package_name> tflite_prop_detection_node --ros-args -r __ns:=<namespace>
   ```
2. Ensure `/tflite_data` and `/rgb_pcl` topics are publishing valid data.
3. Monitor the `detections` and `object_available` topics for centroid information and object availability.

---

## Future Improvements
- Add support for multiple objects and bounding boxes.
- Incorporate additional checks for filtering noisy point clouds.
- Enhance the centroid computation with clustering algorithms for robust results.
