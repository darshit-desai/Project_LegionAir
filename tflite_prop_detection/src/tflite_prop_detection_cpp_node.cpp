/**
 * @file tflite_prop_detection_cpp_node.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief CPP Node that subscribes to the pointcloud2 topic /tof_pc and /tflite_data
 * and publishes the centroid of the detected points inside that bounding box
 * @version 0.1
 * @date 2024-02-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "voxl_msgs/msg/aidetection.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;  // For time literals

class TFLitePropDetectionNode : public rclcpp::Node {
public:
    TFLitePropDetectionNode() : Node("tflite_prop_detection_node"),
                                bbox_x_max_(-std::numeric_limits<int>::max()),
                                bbox_x_min_(-std::numeric_limits<int>::max()),
                                bbox_y_max_(-std::numeric_limits<int>::max()),
                                bbox_y_min_(-std::numeric_limits<int>::max()),
                                image_width_(1024),
                                image_height_(768) {
        node_namespace_ = this->get_namespace();
       
        std::string topic_name_tflite = node_namespace_ + "/tflite_data";
        std::string topic_name_pcl = node_namespace_ + "/rgb_pcl";

        sub_tflite_data_ = this->create_subscription<voxl_msgs::msg::Aidetection>(
            topic_name_tflite, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(), std::bind(&TFLitePropDetectionNode::aidectionCallback, this, std::placeholders::_1));

        sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_pcl, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(), std::bind(&TFLitePropDetectionNode::pclCallback, this, std::placeholders::_1));

        pub_object_centroid_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            node_namespace_ + "/detections", rclcpp::QoS(10).best_effort());

        pub_object_available_ = this->create_publisher<std_msgs::msg::Bool>(
            node_namespace_ + "/object_available", rclcpp::QoS(10).best_effort());

        last_detection_time_ = this->now();
        last_pcl_callback_time_ = this->now();


	    K_pcl_ << 756.3252575983485, 0, 0.0, 0,
                  0, 751.995016895224, 0.0, 0,
                  0, 0, 1, 0;
        K_ << 756.3252575983485, 0, 565.8764531779865,
              0, 751.995016895224, 360.3127057589527,
              0, 0, 1;
    }

private:

    // Callback function for the tflite_data topic
    void aidectionCallback(const voxl_msgs::msg::Aidetection::SharedPtr msg) {
        
        // save the bounding box coordinates
        // note the last time of detection
        if (msg->class_confidence > 0) {
            last_detection_time_ = this->now();
            bbox_x_max_ = msg->x_max;
            bbox_x_min_ = msg->x_min;
            bbox_y_max_ = msg->y_max;
            bbox_y_min_ = msg->y_min;
        }

        // if valid bounding box coordinates are available, publish that the object is available
        if (bbox_x_max_ > 0 && bbox_x_min_ > 0 && bbox_y_max_ > 0 && bbox_y_min_ > 0) {
            std_msgs::msg::Bool available;
            available.data = true;
            pub_object_available_->publish(available);
        }
    }

    // Callback function for the pointcloud2 topic
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        double processing_fps = 1.0 / (this->now() - last_pcl_callback_time_).seconds();
        // std::cout << "Processing FPS: " << processing_fps << std::endl;

        // if the detection is not available for more than 0.07 seconds, set the bounding box coordinates to negative max value
        // and publish that the object is not available
        if ((this->now() - last_detection_time_).seconds() > 0.07) {
            // Set bbox coordinates to negative max value if the detection is not available
            // std::cout << "Detection not available" << std::endl;
            bbox_x_max_ = -std::numeric_limits<int>::max();
            bbox_x_min_ = -std::numeric_limits<int>::max();
            bbox_y_max_ = -std::numeric_limits<int>::max();
            bbox_y_min_ = -std::numeric_limits<int>::max();
            std_msgs::msg::Bool available;
            available.data = false;
            pub_object_available_->publish(available);
        }


        // if the bounding box coordinates are not available, return
        if (bbox_x_max_ == -std::numeric_limits<int>::max() || bbox_x_min_ == -std::numeric_limits<int>::max() ||
            bbox_y_max_ == -std::numeric_limits<int>::max() || bbox_y_min_ == -std::numeric_limits<int>::max()) {
            // std::cout << "No bbox" << std::endl;
            return;
        }

        // Convert the pointcloud2 message to pcl pointcloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // create a 4XN matrix to store the points
        // 1000 is multiplied to convert the points from meters to millimeters
        Eigen::MatrixXf points(4, cloud.size());
        int column_count = 0;
        for (size_t i = 0; i < cloud.size(); ++i) {
            points(0, column_count) = cloud.points[i].x * 1000.0;
            points(1, column_count) = cloud.points[i].y * 1000.0;
            points(2, column_count) = cloud.points[i].z * 1000.0;
            points(3, column_count) = 1.0;
            column_count++;
        }
        points.resize(4, column_count);

        // Project the points to the image plane
        Eigen::MatrixXf projected_points;

        // transformation for the points was done in the camera frame in your point cloud package
        // using the transfomation created from body to world 
        // the 3D points are in the camera frame, so we need to project them to the image plane
        projected_points = K_pcl_ * points;
        projected_points.row(0) = projected_points.row(0).array() / projected_points.row(2).array();
        projected_points.row(1) = projected_points.row(1).array() / projected_points.row(2).array();
        
        projected_points.row(0) = projected_points.row(0).array() + image_width_ / 2;
        projected_points.row(1) = projected_points.row(1).array() + image_height_ / 2;

        int count_filtered_points = 0;
        std::vector<Eigen::Vector3f> filtered_points;
        for (int i = 0; i < projected_points.cols(); ++i) {
            if (projected_points(0, i) > bbox_x_min_ && projected_points(0, i) < bbox_x_max_ &&
                projected_points(1, i) > bbox_y_min_ && projected_points(1, i) < bbox_y_max_) {
                filtered_points.push_back(projected_points.col(i));
                count_filtered_points++;
            }
        }
        
        // std::cout << "Count of filtered points: " << count_filtered_points << std::endl;
        
        if (!filtered_points.empty()) {
            Eigen::Vector3f centroid(0.0, 0.0, 0.0);
            for (auto& point : filtered_points) {
                point(0) = ((point(0) - (image_width_ / 2)) * point(2) / K_(0, 0));
                point(1) = ((point(1) - (image_height_ / 2)) * point(2) / K_(1, 1));
            }
            for (const auto& point : filtered_points) {
                centroid += point;
            }
            centroid /= filtered_points.size();

            // Transform centroid from RDF to FRD
            Eigen::Vector3f centroid_FRD;
            centroid_FRD(0) = centroid(2);  // Forward in FRD
            centroid_FRD(1) = centroid(0);  // Right in FRD
            centroid_FRD(2) = centroid(1);  // Down in FRD
            std::cout << "Centroid: " << centroid_FRD(0) << ", " << centroid_FRD(1) << ", " << centroid_FRD(2) << " filtererd point count" <<  count_filtered_points << std::endl;
            // std::cout << "Centroid: " << centroid_FRD << std::endl;
            auto centroid_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
            centroid_msg->header.stamp = this->now();
            centroid_msg->point.x = centroid_FRD(0);
            centroid_msg->point.y = centroid_FRD(1);
            centroid_msg->point.z = centroid_FRD(2);
            pub_object_centroid_->publish(*centroid_msg);
            object_available_.data = true;
        }
        else {
            object_available_.data = false;
        }

        last_pcl_callback_time_ = this->now();
    }

    rclcpp::Subscription<voxl_msgs::msg::Aidetection>::SharedPtr sub_tflite_data_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_object_centroid_;
    std_msgs::msg::Bool object_available_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_object_available_;
    rclcpp::Time last_detection_time_;
    rclcpp::Time last_pcl_callback_time_;
    int bbox_x_min_;
    int bbox_y_min_;
    int bbox_x_max_;
    int bbox_y_max_;
    Eigen::Matrix<float, 3, 4> K_pcl_;
    Eigen::Matrix3f K_;
    int image_width_;
    int image_height_;
    int id_;
    std::string node_namespace_;
    std::string param_namespace_;
    bool drone_detected_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFLitePropDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
