#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs import do_transform_cloud

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node("point_cloud_transformer", anonymous=True)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create publishers for transformed point clouds
        self.body_pcl_pub = rospy.Publisher("/body_pcl", PointCloud2, queue_size=10)

        # Subscribe to the /tof_pc topic
        rospy.Subscriber("/tof_pc", PointCloud2, self.point_cloud_callback_body)
    def point_cloud_callback_body(self, tof_pc_msg):
        try:
            # Transform point cloud from world to body frame
            body_pcl_msg = self.transform_point_cloud_body(tof_pc_msg, "world", "body")
            self.body_pcl_pub.publish(body_pcl_msg)

            # Perform sanity check
            # self.transform_known_point()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed, skipping point cloud transformation.")

    def transform_point_cloud_body(self, input_pcl, source_frame, target_frame):
        try:
            # Attempt to get transform from source_frame to target_frame
            transform_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))

            # Transform the input point cloud
            transformed_pcl_msg = do_transform_cloud(input_pcl, transform_stamped)

            return transformed_pcl_msg

        except tf2_ros.LookupException:
            rospy.logwarn("Transform lookup failed, skipping point cloud transformation.")
            return None
    def transform_known_point(self):
        # Create a known point in the RGB frame
        point_rgb = PointStamped()
        point_rgb.header.frame_id = "world"
        point_rgb.header.stamp = rospy.Time(0)
        point_rgb.point.x = 0.0
        point_rgb.point.y = 1.0
        point_rgb.point.z = 0.0

        try:
            # Transform the known point from "rgb" to "body"
            point_body = self.tf_buffer.transform(point_rgb, "body")

            # Print the transformed point in Body frame
            rospy.loginfo("Transformed Point in Body frame: x={}, y={}, z={}".format(
                point_body.point.x, point_body.point.y, point_body.point.z))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed for the known point.")



    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        pcl_transformer = PointCloudTransformer()
        pcl_transformer.run()
    except rospy.ROSInterruptException:
        pass
