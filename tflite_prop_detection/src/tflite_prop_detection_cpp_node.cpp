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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <voxl_mpa_to_ros/AiDetection.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>
/**
 * @brief TFLitePropDetectionNode class that subscribes to the pointcloud2 topic /tof_pc and /tflite_data
 * 
 */
class TFLitePropDetectionNode {
public:
    TFLitePropDetectionNode() : K_pcl_(Eigen::Matrix<float, 3, 4>::Zero()), K_(Eigen::Matrix3f::Zero()) {
        ros::NodeHandle nh;
        sub_tflite_data_ = nh.subscribe("/tflite_data", 1, &TFLitePropDetectionNode::aidectionCallback, this);
        sub_pcl_ = nh.subscribe("/rgb_pcl", 1, &TFLitePropDetectionNode::pclCallback, this);
        pub_object_centroid_ = nh.advertise<geometry_msgs::PointStamped>("/detections", 15);
        pub_object_available_ = nh.advertise<std_msgs::Bool>("/object_available", 15);
        last_detection_time_ = ros::Time::now();
        last_pcl_callback_time_ = ros::Time::now();
        // Initialize K_pcl_ with appropriate values and then divide it by 1000 to convert it to meters
        // K_pcl_ << 756.3252575983485, 0, 565.876453177986, 0,
        //           0, 751.995016895224, 360.3127057589527, 0,
        //           0, 0, 1, 0;
        K_pcl_ << 756.3252575983485, 0, 0.0, 0,
                  0, 751.995016895224, 0.0, 0,
                  0, 0, 1, 0;

        K_ << 756.3252575983485, 0, 565.8764531779865,
              0, 751.995016895224, 360.3127057589527,
              0, 0, 1;
        image_width_ = 1024;
        image_height_ = 768;
    }

    void aidectionCallback(const voxl_mpa_to_ros::AiDetection::ConstPtr& msg) {
        if ((ros::Time::now() - last_detection_time_).toSec() > 0.07) {
            // Set bbox coordinates to negative max value if the detection is not available
            bbox_x_max_ = -std::numeric_limits<int>::max();
            bbox_x_min_ = -std::numeric_limits<int>::max();
            bbox_y_max_ = -std::numeric_limits<int>::max();
            bbox_y_min_ = -std::numeric_limits<int>::max();
            std_msgs::Bool available;
            available.data = false;
            pub_object_available_.publish(available);
        }
        if(bbox_x_max_ > 0 && bbox_x_min_ > 0 && bbox_y_max_ > 0 && bbox_y_min_ > 0)
         {
            std_msgs::Bool available;
            // Print the bbox values bbox_x_min_, bbox_y_min_, bbox_x_max_, bbox_y_max_
            std::cout << "Bbox values: " << bbox_x_min_ << " " << bbox_y_min_ << " " << bbox_x_max_ << " " << bbox_y_max_ << std::endl;
            available.data = true;
            pub_object_available_.publish(available);
        }

        if (msg->class_confidence > 0) {
            last_detection_time_ = ros::Time::now();
            bbox_x_max_ = msg->x_max;
            bbox_x_min_ = msg->x_min;
            bbox_y_max_ = msg->y_max;
            bbox_y_min_ = msg->y_min;
        }
    }

    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        double processing_fps = 1.0 / (ros::Time::now() - last_pcl_callback_time_).toSec();
        std::cout << "Processing FPS: " << processing_fps << std::endl;
        // Check if bbox values are negative infinity
        if (bbox_x_max_ == -std::numeric_limits<int>::max() || bbox_x_min_ == -std::numeric_limits<int>::max() || bbox_y_max_ == -std::numeric_limits<int>::max() || bbox_y_min_ == -std::numeric_limits<int>::max()){
            //Debug statement i am here
            std::cout << "No bbox" << std::endl;
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        // Define a 4xN Eigen librarymatrix to store the points
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
        Eigen::MatrixXf projected_points;
        // Do the matrix multiplication of K_pcl_ which is 3x4 matrix and points which is 4xN matrix
        projected_points = K_pcl_ * points;
        // Print shape of projected_points
        // std::cout << "Projected Points shape: " << projected_points.rows() << " " << projected_points.cols() << std::endl;
        // Homogenize the projected_points first two rows by dividing by the third row and store it in projected_points
        projected_points.row(0) = projected_points.row(0).array() / projected_points.row(2).array();
        projected_points.row(1) = projected_points.row(1).array() / projected_points.row(2).array();
        // Print max x and y values
        // std::cout << "Projected before adding image width and height, Max x and y values: " << projected_points.row(0).maxCoeff() << " " << projected_points.row(1).maxCoeff() << std::endl;
        // Print min x and y values
        // std::cout << "Projected before adding image width and height, Min x and y values: " << projected_points.row(0).minCoeff() << " " << projected_points.row(1).minCoeff() << std::endl;
        
        // Add the image width and height from the projected_points first and second row respectively
        projected_points.row(0) = projected_points.row(0).array() + image_width_ / 2;
        projected_points.row(1) = projected_points.row(1).array() + image_height_ / 2;
        // Print max x and y values
        // std::cout << "Projected after adding image width and height, Max x and y values: " << projected_points.row(0).maxCoeff() << " " << projected_points.row(1).maxCoeff() << std::endl;
        // Print min x and y values
        // std::cout << "Projected after adding image width and height, Min x and y values: " << projected_points.row(0).minCoeff() << " " << projected_points.row(1).minCoeff() << std::endl;
        // Filter the points that are inside the bounding box by typecasting the projected_points to int and checking if they are inside the bounding box
        int count_filtered_points = 0;
        std::vector<Eigen::Vector3f> filtered_points;
        for (int i = 0; i < projected_points.cols(); ++i) {
            // First check x bounds
            if (projected_points(0, i) > bbox_x_min_ && projected_points(0, i) < bbox_x_max_) {
                // Then check y bounds
                if (projected_points(1, i) > bbox_y_min_ && projected_points(1, i) < bbox_y_max_) {
                    filtered_points.push_back(projected_points.col(i));
                    count_filtered_points++;
                }
            }
        }
        
        // Print the count of filtered points
        std::cout << "Count of filtered points: " << count_filtered_points << std::endl;
        // Find max x and y and min x and y in the filtered points
        // double max_x = std::numeric_limits<float>::min();
        // double max_y = std::numeric_limits<float>::min();
        // double min_x = std::numeric_limits<float>::max();
        // double min_y = std::numeric_limits<float>::max();
        // for (const auto& point : filtered_points) {
        //     if (point(0) > max_x) {
        //         max_x = point(0);
        //     }
        //     if (point(0) < min_x) {
        //         min_x = point(0);
        //     }
        //     if (point(1) > max_y) {
        //         max_y = point(1);
        //     }
        //     if (point(1) < min_y) {
        //         min_y = point(1);
        //     }
        // }
        // // Print the max x and y and min x and y for the filtered points
        // std::cout << "Max x for filtered points: " << max_x << std::endl;
        // std::cout << "Max y for filtered points: " << max_y << std::endl;
        // std::cout << "Min x for filtered points: " << min_x << std::endl;
        // std::cout << "Min y for filtered points: " << min_y << std::endl;
        // Print filtered points size
        // std::cout << "Filtered points size: " << filtered_points.size() << std::endl;
        // Compute centroid from the filtered points
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
            // Print the centroid
            std::cout << "Centroid: " << centroid << std::endl;
            geometry_msgs::PointStamped centroid_msg;
            centroid_msg.header.stamp = ros::Time::now();
            centroid_msg.point.x = centroid(0);
            centroid_msg.point.y = centroid(1);
            centroid_msg.point.z = centroid(2);
            pub_object_centroid_.publish(centroid_msg);
            object_available_.data = true;
        }
        else {
            object_available_.data = false;
        }

        last_pcl_callback_time_ = ros::Time::now();
    }

        

private:
    ros::Subscriber sub_tflite_data_;
    ros::Subscriber sub_pcl_;
    ros::Publisher pub_object_centroid_;
    std_msgs::Bool object_available_;
    ros::Publisher pub_object_available_;
    ros::Time last_detection_time_;
    ros::Time last_pcl_callback_time_;
    int bbox_x_min_;
    int bbox_y_min_;
    int bbox_x_max_;
    int bbox_y_max_;
    Eigen::Matrix<float, 3, 4> K_pcl_;
    Eigen::Matrix3f K_;
    int image_width_;
    int image_height_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tflite_prop_detection_node");
    TFLitePropDetectionNode node;
    ros::spin();
    return 0;
}
