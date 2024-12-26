#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2
from voxl_mpa_to_ros.msg import AiDetection
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import copy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import time

class TFLitePropDetectionNode:
    def __init__(self):
        rospy.init_node("tflite_prop_detection_node")
        rospy.Subscriber("/tflite_data", AiDetection, self.aidection_callback)
        rospy.Subscriber("/tflite", Image, self.image_callback)
        rospy.Subscriber("/rgb_pcl", PointCloud2, self.pcl_callback)
        # self.pub = rospy.Publisher("/tflite_prop_dets", Image, queue_size=10)
        # self.pub_points = rospy.Publisher("/dets_with_points", Image, queue_size=10)
        # Publisher for object centroid point
        self.pub_object_centroid = rospy.Publisher(
            "/detections", PointStamped, queue_size=15
        )
        # Publisher for object availability signal
        self.pub_object_available = rospy.Publisher(
            "/object_available", Bool, queue_size=15
        )
        self.kd_tree = None
        self.cv_bridge = CvBridge()
        self.bbox = None
        self.last_detection_time = rospy.get_time()
        self.last_pcl_callback_time = rospy.get_time()
        self.cv_image_undistorted = None
        self.centroid_data_available = False
        self.centroid = None
        self.filtered_points = None
        self.projected_points = None
        self.points_homo = None
        self.points_depth_filt = None
        # Calibration matrix and distortion coefficients
        self.K_pcl = np.array(
            [
                [756.3252575983485, 0, 565.8764531779865, 0],
                [0, 751.995016895224, 360.3127057589527, 0],
                [0, 0, 1, 0],
            ]
        )
        self.K = np.array(
            [
                [756.3252575983485, 0, 565.8764531779865],
                [0, 751.995016895224, 360.3127057589527],
                [0, 0, 1],
            ]
        )
        self.dist_coeffs = np.array(
            [
                0.08809895427832855,
                -0.1568187367908102,
                -0.001712246293793639,
                -0.001384262087904217,
                0,
            ]
        )
        self.points = None
        self.processing_fps = 0.0
        self.object_centroid_msg = PointStamped()

    def aidection_callback(self, msg):
        if rospy.get_time() - self.last_detection_time > 0.07:
            self.bbox = None
            self.pub_object_available.publish(Bool(False))
        if self.bbox is not None:
            # if self.centroid_data_available:
            # self.draw_bbox_with_confidence(self.cv_image_undistorted)
            self.pub_object_available.publish(Bool(self.centroid_data_available))
        if msg.class_confidence > 0:
            self.bbox = (msg.x_min, msg.y_min, msg.x_max, msg.y_max)
            # self.class_name = msg.class_name
            # self.class_confidence = msg.class_confidence * 100
            # self.det_confidence = msg.detection_confidence * 100
            self.last_detection_time = rospy.get_time()

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            print("Shape of the image: ", cv_image.shape)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        self.cv_image_undistorted = cv2.undistort(cv_image, self.K, self.dist_coeffs)

        try:
            self.pub.publish(
                self.cv_bridge.cv2_to_imgmsg(self.cv_image_undistorted, "bgr8")
            )
        except CvBridgeError as e:
            rospy.logerr(e)

    def pcl_callback(self, msg):
        start_time = time.time()
        # Calculate processing FPS
        self.processing_fps = 1.0 / (rospy.get_time() - self.last_pcl_callback_time)
        self.last_pcl_callback_time = rospy.get_time()
        rospy.loginfo("Processing FPS: %d", self.processing_fps)
        # Check if bbox is available
        if self.bbox is None:
            # # Publish original image if bbox is not available
            # try:
            #     self.pub_points.publish(
            #         self.cv_bridge.cv2_to_imgmsg(self.cv_image_undistorted, "bgr8")
            #     )
            # except CvBridgeError as e:
            #     rospy.logerr(e)
            return

        # Convert 3D points to image coordinates
        self.points = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        ).T
        self.points_depth_filt = self.points[:, self.points[2] <= 1.5]
        print("Shape after depth filtering: ", self.points_depth_filt.shape)
        self.points_homo = np.vstack(
            (self.points_depth_filt, np.ones((1, self.points_depth_filt.shape[1])))
        )
        self.projected_points = np.dot(self.K_pcl, self.points_homo)
        self.projected_points[0:2] /= self.projected_points[2]
        self.projected_points = self.projected_points.astype(int)
        self.projected_points[-1] = self.points_depth_filt[-1]
        
        # Filter points based on depth and bounding box
        if self.bbox is not None:
            self.filtered_points = self.projected_points[
                :,
                (self.projected_points[0] >= self.bbox[0])
                & (self.projected_points[1] >= self.bbox[1])
                & (self.projected_points[0] <= self.bbox[2])
                & (self.projected_points[1] <= self.bbox[3]),
            ]
            print(self.filtered_points.shape)
        else:
            self.filtered_points = None
            print("Empty filtered array")
        

        # print(filtered_points.shape)
        # Find the 3d centroid of the tracked object
        if self.filtered_points.size != 0:
            self.centroid = np.mean(self.filtered_points, axis=1)
            # Publish object centroid point
            self.object_centroid_msg.header.stamp = rospy.Time.now()
            self.object_centroid_msg.point.x = (
                (self.centroid[0] - self.K[0, 2]) * self.centroid[2] / self.K_pcl[0, 0]
            )
            self.object_centroid_msg.point.y = (
                (self.centroid[1] - self.K[1, 2]) * self.centroid[2] / self.K_pcl[1, 1]
            )
            self.object_centroid_msg.point.z = self.centroid[2]
            self.pub_object_centroid.publish(self.object_centroid_msg)
            # Publish object availability signal as True
            self.centroid_data_available = True
        else:
            # If no detections, publish object availability signal as False
            self.centroid_data_available = False
        print("Calculated run time: ", start_time - time.time())
        # Draw points on image
        # try:
        #     pcl_image = copy.deepcopy(self.cv_image_undistorted)
        #     if filtered_points.size != 0:
        #         for i in range(filtered_points.shape[1]):
        #             point = filtered_points[:, i]
        #             x, y, z = point.astype(int)
        #             cv2.circle(pcl_image, (x, y), 2, (0, 0, 255), -1)

        #     # Draw processing FPS on the image
        #     font = cv2.FONT_HERSHEY_SIMPLEX
        #     font_scale = 0.5
        #     font_thickness = 2
        #     text = f"FPS: {processing_fps:.2f}"
        #     text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        #     cv2.putText(
        #         pcl_image,
        #         text,
        #         (pcl_image.shape[1] - text_size[0] - 10, text_size[1] + 10),
        #         font,
        #         font_scale,
        #         (255, 255, 255),
        #         font_thickness,
        #     )

        #     self.pub_points.publish(self.cv_bridge.cv2_to_imgmsg(pcl_image, "bgr8"))
        # except CvBridgeError as e:
        #     rospy.logerr(e)

    def draw_bbox_with_confidence(self, image):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 2
        text = f"{self.class_name}: DC: {self.det_confidence:.3f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]

        x_min = int(self.bbox[0])
        y_min = int(self.bbox[1])
        x_max = int(self.bbox[2])
        y_max = int(self.bbox[3])

        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.rectangle(
            image,
            (x_min, y_min - text_size[1]),
            (x_min + text_size[0], y_min),
            (0, 255, 0),
            -1,
        )
        cv2.putText(
            image,
            text,
            (x_min, y_min - 5),
            font,
            font_scale,
            (255, 255, 255),
            font_thickness,
        )

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = TFLitePropDetectionNode()
    node.run()
