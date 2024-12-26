// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <eigen3/Eigen/Dense>

// using namespace std::chrono_literals;

// class KalmanFilterNode : public rclcpp::Node
// {
// public:
//     KalmanFilterNode() : Node("kalman_filter_node"), kf_(6, 3)  // Initialize KalmanFilter with dimensions
//     {
//         // Initialize the Kalman filter
//         int n = 6; // Number of state variables
//         int m = 3; // Number of measurements

//         kf_.F = Eigen::MatrixXd::Identity(n, n);
//         double dt = 1.0 / 30.0;
//         kf_.F(0, 1) = dt;
//         kf_.F(2, 3) = dt;
//         kf_.F(4, 5) = dt;

//         kf_.H = Eigen::MatrixXd::Zero(m, n);
//         kf_.H(0, 0) = 1.0;
//         kf_.H(1, 2) = 1.0;
//         kf_.H(2, 4) = 1.0;

//         kf_.Q = Eigen::MatrixXd::Identity(n, n) * 0.01;
//         kf_.R = Eigen::MatrixXd::Identity(m, m) * 0.1;
//         kf_.P = Eigen::MatrixXd::Identity(n, n);

//         predicted_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
//             "/uav_1/predicted_position", rclcpp::QoS(10).best_effort());
//         std::string topic_name_detections = "/uav_1/detections";
//         std::string topic_name_object_available = "/uav_1/object_available";
//         detections_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             topic_name_detections, rclcpp::QoS(10).best_effort(), 
//             std::bind(&KalmanFilterNode::detections_callback, this, std::placeholders::_1));
//         bbox_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//             topic_name_object_available, rclcpp::QoS(10).best_effort(), 
//             std::bind(&KalmanFilterNode::bbox_callback, this, std::placeholders::_1));

//         timer_ = this->create_wall_timer(33ms, std::bind(&KalmanFilterNode::update, this));

//         object_available_ = false;
//         initialized_ = false;
//     }

// private:
//     struct KalmanFilter
//     {
//         KalmanFilter(int state_dim, int meas_dim)
//         {
//             x = Eigen::VectorXd::Zero(state_dim);
//             F = Eigen::MatrixXd::Identity(state_dim, state_dim);
//             P = Eigen::MatrixXd::Identity(state_dim, state_dim);
//             Q = Eigen::MatrixXd::Identity(state_dim, state_dim);
//             H = Eigen::MatrixXd::Zero(meas_dim, state_dim);
//             R = Eigen::MatrixXd::Identity(meas_dim, meas_dim);
//             K = Eigen::MatrixXd::Zero(state_dim, meas_dim);
//         }

//         Eigen::VectorXd x; // State vector
//         Eigen::MatrixXd F; // State transition matrix
//         Eigen::MatrixXd P; // Covariance matrix
//         Eigen::MatrixXd Q; // Process noise covariance
//         Eigen::MatrixXd H; // Measurement matrix
//         Eigen::MatrixXd R; // Measurement noise covariance
//         Eigen::MatrixXd K; // Kalman gain

//         void predict()
//         {
//             x = F * x;
//             P = F * P * F.transpose() + Q;
//         }

//         void update(const Eigen::VectorXd &z)
//         {
//             Eigen::VectorXd y = z - H * x;
//             Eigen::MatrixXd S = H * P * H.transpose() + R;
//             K = P * H.transpose() * S.inverse();
//             x = x + K * y;
//             int size = x.size();
//             Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
//             P = (I - K * H) * P;
//         }
//     };

//     void detections_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
//     {
//         last_detection_time_ = this->get_clock()->now();
//         if (!initialized_)
//         {
//             kf_.x(0) = msg->point.x;
//             kf_.x(2) = msg->point.y;
//             kf_.x(4) = msg->point.z;
//             initialized_ = true;
//         }
//         else
//         {
//             Eigen::VectorXd z(3);
//             z << msg->point.x, msg->point.y, msg->point.z;
//             kf_.update(z);
//         }
//     }

//     void bbox_callback(const std_msgs::msg::Bool::SharedPtr msg)
//     {
//         object_available_ = msg->data;
//     }

//     void update()
//     {
//         if (initialized_)
//         {
//             auto current_time = this->get_clock()->now();
//             if (object_available_ && (current_time - last_detection_time_).seconds() < 1.0 / 30.0)
//             {
//                 kf_.predict();
//                 publish_prediction();
//             }
//             else if ((current_time - last_detection_time_).seconds() >= 1.0 / 30.0)
//             {
//                 kf_.predict();
//                 publish_prediction();
//             }
//         }
//     }

//     void publish_prediction()
//     {
//         auto msg = geometry_msgs::msg::PointStamped();
//         msg.header.stamp = this->get_clock()->now();
//         msg.point.x = kf_.x(0);
//         msg.point.y = kf_.x(2);
//         msg.point.z = kf_.x(4);
//         std::cout << "Predicted position: " << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << std::endl;
//         predicted_pos_pub_->publish(msg);
//     }

//     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predicted_pos_pub_;
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detections_sub_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bbox_sub_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     KalmanFilter kf_;
//     rclcpp::Time last_detection_time_;
//     bool object_available_;
//     bool initialized_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<KalmanFilterNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <eigen3/Eigen/Dense>

using namespace std::chrono_literals;

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kalman_filter_node"), kf_(6, 3)  // Initialize KalmanFilter with dimensions
    {
        // Initialize the Kalman filter
        int n = 6; // Number of state variables (position and velocity for x, y, z)
        int m = 3; // Number of measurements (position x, y, z)

        kf_.F = Eigen::MatrixXd::Identity(n, n);
        double dt = 1.0 / 30.0;
        kf_.F(0, 1) = dt;
        kf_.F(2, 3) = dt;
        kf_.F(4, 5) = dt;

        kf_.H = Eigen::MatrixXd::Zero(m, n);
        kf_.H(0, 0) = 1.0;
        kf_.H(1, 2) = 1.0;
        kf_.H(2, 4) = 1.0;

        kf_.Q = Eigen::MatrixXd::Identity(n, n) * 0.01;
        kf_.R = Eigen::MatrixXd::Identity(m, m) * 0.1;
        kf_.P = Eigen::MatrixXd::Identity(n, n);

        predicted_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/uav_1/predicted_position", rclcpp::QoS(10).best_effort());
        predicted_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/uav_1/predicted_twist", rclcpp::QoS(10).best_effort());

        std::string topic_name_detections = "/uav_1/detections";
        std::string topic_name_object_available = "/uav_1/object_available";
        detections_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            topic_name_detections, rclcpp::QoS(10).best_effort(), 
            std::bind(&KalmanFilterNode::detections_callback, this, std::placeholders::_1));
        bbox_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            topic_name_object_available, rclcpp::QoS(10).best_effort(), 
            std::bind(&KalmanFilterNode::bbox_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(33ms, std::bind(&KalmanFilterNode::update, this));

        object_available_ = false;
        initialized_ = false;
    }

private:
    struct KalmanFilter
    {
        KalmanFilter(int state_dim, int meas_dim)
        {
            x = Eigen::VectorXd::Zero(state_dim);
            F = Eigen::MatrixXd::Identity(state_dim, state_dim);
            P = Eigen::MatrixXd::Identity(state_dim, state_dim);
            Q = Eigen::MatrixXd::Identity(state_dim, state_dim);
            H = Eigen::MatrixXd::Zero(meas_dim, state_dim);
            R = Eigen::MatrixXd::Identity(meas_dim, meas_dim);
            K = Eigen::MatrixXd::Zero(state_dim, meas_dim);
        }

        Eigen::VectorXd x; // State vector
        Eigen::MatrixXd F; // State transition matrix
        Eigen::MatrixXd P; // Covariance matrix
        Eigen::MatrixXd Q; // Process noise covariance
        Eigen::MatrixXd H; // Measurement matrix
        Eigen::MatrixXd R; // Measurement noise covariance
        Eigen::MatrixXd K; // Kalman gain

        void predict()
        {
            x = F * x;
            P = F * P * F.transpose() + Q;
        }

        void update(const Eigen::VectorXd &z)
        {
            Eigen::VectorXd y = z - H * x;
            Eigen::MatrixXd S = H * P * H.transpose() + R;
            K = P * H.transpose() * S.inverse();
            x = x + K * y;
            int size = x.size();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
            P = (I - K * H) * P;
        }
    };

    void detections_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_detection_time_ = this->get_clock()->now();
        if (!initialized_)
        {
            kf_.x(0) = msg->point.x;
            kf_.x(2) = msg->point.y;
            kf_.x(4) = msg->point.z;
            initialized_ = true;
        }
        else
        {
            Eigen::VectorXd z(3);
            z << msg->point.x, msg->point.y, msg->point.z;
            kf_.update(z);
        }
    }

    void bbox_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        object_available_ = msg->data;
    }

    void update()
    {
        if (initialized_)
        {
            auto current_time = this->get_clock()->now();
            if (object_available_ && (current_time - last_detection_time_).seconds() < 1.0 / 30.0)
            {
                kf_.predict();
                publish_prediction();
            }
            else if ((current_time - last_detection_time_).seconds() >= 1.0 / 30.0)
            {
                kf_.predict();
                publish_prediction();
            }
        }
    }

    void publish_prediction()
    {
        // Publish position
        auto pos_msg = geometry_msgs::msg::PointStamped();
        pos_msg.header.stamp = this->get_clock()->now();
        pos_msg.point.x = kf_.x(0);
        pos_msg.point.y = kf_.x(2);
        pos_msg.point.z = kf_.x(4);
        std::cout << "Predicted position: " << pos_msg.point.x << ", " << pos_msg.point.y << ", " << pos_msg.point.z << std::endl;
        predicted_pos_pub_->publish(pos_msg);

        // Publish twist (velocity)
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.twist.linear.x = kf_.x(1);
        twist_msg.twist.linear.y = kf_.x(3);
        twist_msg.twist.linear.z = kf_.x(5);
        std::cout << "Predicted velocity: " << twist_msg.twist.linear.x << ", " << twist_msg.twist.linear.y << ", " << twist_msg.twist.linear.z << std::endl;
        predicted_twist_pub_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predicted_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr predicted_twist_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detections_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bbox_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    KalmanFilter kf_;
    rclcpp::Time last_detection_time_;
    bool object_available_;
    bool initialized_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
