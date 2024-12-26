/**
 * @file tf_publisher_cpp_node.cpp
 * @brief Node that publishes static transforms for a drone
 * @authors Shivam Sehgal (ssehgal7@umd.edu)
 *          Darshit Desai (darshit@umd.edu)
 * @version 0.1
 * @date 2024-02-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster_node.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

// Structure to hold transform parameters between two frames
struct ExtrinsicTransform {
    std::string parent;              // Name of parent frame
    std::string child;               // Name of child frame
    std::vector<double> T_child_wrt_parent;    // Translation vector [x,y,z]
    std::vector<double> RPY_parent_to_child;   // Roll-Pitch-Yaw angles in degrees
};

// Function to publish static transforms between coordinate frames
void publish_static_transforms(const std::shared_ptr<tf2_ros::StaticTransformBroadcaster>& static_broadcaster, const rclcpp::Node::SharedPtr& node) {
    // Define the transforms between different coordinate frames
    // Format: parent frame, child frame, translation [x,y,z], rotation [roll,pitch,yaw] in degrees
    // Get node namespace
    std::string node_namespace = node->get_namespace();
    std::string transform_namespace = node_namespace;
    if (!transform_namespace.empty() && transform_namespace.front() == '/') {
        transform_namespace.erase(0, 1);
    }
    // RCLCPP_INFO_STREAM(node->get_logger(), "Namespace in tf_publisher: " << node_namespace);
    
    std::vector<ExtrinsicTransform> extrinsics = {
        {transform_namespace + "body", transform_namespace + "world", {0.068, 0.0116, 0.0168}, {0, 270, 0}},    // Transform from body to world frame
        {transform_namespace + "body", transform_namespace + "hires", {0.068, -0.012, 0.015}, {90, 270, 0}}     // Transform from body to high-res camera frame
    };

    // Vector to store all transforms
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    for (const auto& extrinsic : extrinsics) {
        geometry_msgs::msg::TransformStamped static_transform;
        // Set header information
        static_transform.header.stamp = node->now();
        static_transform.header.frame_id = extrinsic.parent;
        static_transform.child_frame_id = extrinsic.child;
        
        // Set translation components
        static_transform.transform.translation.x = extrinsic.T_child_wrt_parent[0];
        static_transform.transform.translation.y = extrinsic.T_child_wrt_parent[1];
        static_transform.transform.translation.z = extrinsic.T_child_wrt_parent[2];
        
        // Convert rotation from RPY angles to quaternion
        double roll = extrinsic.RPY_parent_to_child[0];
        double pitch = extrinsic.RPY_parent_to_child[1];
        double yaw = extrinsic.RPY_parent_to_child[2];
        // Convert degrees to radians
        double roll_rad = roll * M_PI / 180;
        double pitch_rad = pitch * M_PI / 180;
        double yaw_rad = yaw * M_PI / 180;
        
        // Create rotation using Eigen's angle-axis representation
        Eigen::AngleAxisd rotation_vector_x(roll_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotation_vector_y(pitch_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation_vector_z(yaw_rad, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = rotation_vector_x * rotation_vector_y * rotation_vector_z;
        
        // Set quaternion components
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_transforms.push_back(static_transform);
    }
    // Broadcast all transforms
    static_broadcaster->sendTransform(static_transforms);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // Create ROS node
    auto node = rclcpp::Node::make_shared("tf_publisher_cpp_node");
    // Create transform broadcaster
    auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // Create timer to periodically publish transforms
    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [=]() {
        publish_static_transforms(static_broadcaster, node);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
