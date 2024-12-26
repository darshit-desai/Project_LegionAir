/**
 * @file pc_transform.cpp
 * @brief Node that transforms point cloud data from TOF camera frame to body frame
 * @authors Shivam Sehgal (ssehgal7@umd.edu)
 *          Darshit Desai (darshit@umd.edu)
 * @version 0.1
 * @date 2024-02-17
 *
 * @copyright Copyright (c) 2024
 *
 */


#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <cstring>
#include <string>
#include <stdexcept>



// Node class that handles point cloud transformation from TOF camera frame to body frame
class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer() : Node("pc_transformer") {
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Get node namespace for topic names
    node_namespace_ = this->get_namespace();
    transform_namespace = node_namespace_;
    if (!transform_namespace.empty() && transform_namespace.front() == '/') {
        transform_namespace.erase(0, 1);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Namespace in pc_transform: " << node_namespace_);
    std::string topic_name_tof = node_namespace_ + "/tof_pc";
    
    // Create subscriber for input point cloud
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name_tof, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(), std::bind(&PointCloudTransformer::pc_callback, this, std::placeholders::_1));
    
    // Create publisher for transformed point cloud
    std::string topic_name_rgb = node_namespace_ + "/rgb_pcl";
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_rgb, rclcpp::SensorDataQoS());

    // Initialize transform variables
    translation_vector = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    rotation_matrix = Eigen::Matrix3d::Zero();
  }

private:
  // Callback function for processing incoming point clouds
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg, *cloud);
    
    // Filter points by z-coordinate (depth)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.4, 1.6);
    pass.filter(*cloud_filtered);

    try {
      // Look up transform from point cloud frame to target frame
      geometry_msgs::msg::TransformStamped transform_stamped;
      std::string frame_id = transform_namespace + pc_msg->header.frame_id;
      transform_stamped = tf_buffer_->lookupTransform(transform_namespace  + "hires", frame_id, tf2::TimePointZero);
      
      // Extract translation and rotation from transform
      translation_vector[0] = transform_stamped.transform.translation.x;
      translation_vector[1] = transform_stamped.transform.translation.y;
      translation_vector[2] = transform_stamped.transform.translation.z;
      tf_quat[0] = transform_stamped.transform.rotation.x;
      tf_quat[1] = transform_stamped.transform.rotation.y;
      tf_quat[2] = transform_stamped.transform.rotation.z;
      tf_quat[3] = transform_stamped.transform.rotation.w;
      
      // Convert quaternion to rotation matrix
      q = Eigen::Quaterniond(tf_quat[3], tf_quat[0], tf_quat[1], tf_quat[2]);
      rotation_matrix = q.toRotationMatrix();
      
      // Create affine transform from translation and rotation
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translation() << translation_vector[0], translation_vector[1], translation_vector[2];
      transform.rotate(rotation_matrix.cast<float>());
      
      // Apply transform to point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform);
      
      // Convert back to ROS message and publish
      sensor_msgs::msg::PointCloud2 transformed_pc;
      pcl::toROSMsg(*transformed_cloud, transformed_pc);
      transformed_pc.header.frame_id = transform_namespace + "hires";
      pc_pub_->publish(transformed_pc);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Failure: %s\n", ex.what());
      return;
    }
  }

private:
  // TF buffer and listener for coordinate transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // ROS publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  
  // Transform variables
  Eigen::Vector3d translation_vector;
  double tf_quat[4];
  Eigen::Quaterniond q;
  Eigen::Matrix3d rotation_matrix;
  std::string node_namespace_;
  std::string transform_namespace;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
