#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/subscription_options.hpp>
#include <drone_swarm_msgs/msg/drone_status.hpp>
#include <drone_swarm_msgs/msg/move_drone.hpp>
#include <drone_swarm_msgs/srv/drop_drone.hpp>
#include <drone_swarm_msgs/srv/set_anchor_and_neighbours.hpp>
#include <drone_swarm_msgs/srv/calc_angle.hpp>
#include <drone_swarm_msgs/srv/move_drone.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp" ///< Include the rcl_interfaces library


using DropDroneSrv = drone_swarm_msgs::srv::DropDrone;
using SetAnchorAndNeighbours = drone_swarm_msgs::srv::SetAnchorAndNeighbours;
using CalcAngle = drone_swarm_msgs::srv::CalcAngle;
using MoveDroneSrv = drone_swarm_msgs::srv::MoveDrone;
class DroneNode : public rclcpp::Node
{
    public:
        DroneNode(std::string node_name): Node(node_name) {

        //----------------------------------------------
        // Parameters
        //----------------------------------------------
        rcl_interfaces::msg::ParameterDescriptor descriptor_id;
        descriptor_id.description = "Drone ID";
        descriptor_id.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("id", 0, descriptor_id);
        id_ = this->get_parameter("id").as_int();

        rcl_interfaces::msg::ParameterDescriptor descriptor_anchor;
        descriptor_anchor.description = "Anchor";
        descriptor_anchor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        this->declare_parameter("anchor", false, descriptor_anchor);
        anchor_ = this->get_parameter("anchor").as_bool();

        rcl_interfaces::msg::ParameterDescriptor descriptor_phase_angle;
        descriptor_phase_angle.description = "Phase Angle";
        descriptor_phase_angle.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("phase_angle", 0.0, descriptor_phase_angle);
        phase_angel_ = this->get_parameter("phase_angle").as_double();

        target_phase_angel_ = phase_angel_;
        target_x_ = 0.0;
        target_y_ = 0.0;
        temp_target_x_ = 0.0;
        temp_target_y_ = 0.0;
        alpha_ = 0.0;
        land_ = false;

        rcl_interfaces::msg::ParameterDescriptor descriptor_radius;
        descriptor_radius.description = "Radius";
        descriptor_radius.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("radius", 1.0, descriptor_radius);
        radius_ = this->get_parameter("radius").as_double();

        rcl_interfaces::msg::ParameterDescriptor descriptor_neighbour_left;
        descriptor_neighbour_left.description = "Neighbor Left";
        descriptor_neighbour_left.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("neighbour_left", 2, descriptor_neighbour_left);
        neighbour_left_ = this->get_parameter("neighbour_left").as_int();

        rcl_interfaces::msg::ParameterDescriptor descriptor_neighbour_right;
        descriptor_neighbour_right.description = "Neighbour Right";
        descriptor_neighbour_right.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("neighbour_right", 1, descriptor_neighbour_right);
        neighbour_right_ = this->get_parameter("neighbour_right").as_int();
        
        // Initialize the callback group for the publisher and subscriber of this node
        pub_sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.callback_group = pub_sub_group_;
        status_msg_=drone_swarm_msgs::msg::DroneStatus();
        std::string topic_name = "drone_status_" + std::to_string(id_);
        publisher_ = this->create_publisher<drone_swarm_msgs::msg::DroneStatus>(topic_name, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DroneNode::timer_callback, this), pub_sub_group_);
        std::string topic_name_left = "drone_status_" + std::to_string(neighbour_left_);
        // Print neighbour_left_ id
        RCLCPP_INFO(this->get_logger(), "Neighbour Left ID: %d", neighbour_left_);
        std::string topic_name_right = "drone_status_" + std::to_string(neighbour_right_);  
        // Print neighbour_right_ id
        RCLCPP_INFO(this->get_logger(), "Neighbour Right ID: %d", neighbour_right_);
        subscriber_left_ = this->create_subscription<drone_swarm_msgs::msg::DroneStatus>(topic_name_left, 10, std::bind(&DroneNode::callback_left, this, std::placeholders::_1), sub_opt);
        subscriber_right_ = this->create_subscription<drone_swarm_msgs::msg::DroneStatus>(topic_name_right, 10, std::bind(&DroneNode::callback_right, this, std::placeholders::_1), sub_opt);
        // Create MoveDrone Publisher
        std::string topic_name_move = "move_drone_" + std::to_string(id_);
        move_msg_ = drone_swarm_msgs::msg::MoveDrone();
        publisher_move_ = this->create_publisher<drone_swarm_msgs::msg::MoveDrone>(topic_name_move, 10);
        timer_2 = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DroneNode::Move_drone_callback, this), pub_sub_group_);

        // Drop node service server
        std::string service_name_drop_drone_ = "drop_drone_" + std::to_string(id_);
        service_1_ = this->create_service<DropDroneSrv>(
        service_name_drop_drone_,
        std::bind(&DroneNode::handle_drop_done_request,
                  this, std::placeholders::_1, std::placeholders::_2));

        std::string service_name_move_drone_ = "move_drone_" + std::to_string(id_);
        service_move_ = this->create_service<MoveDroneSrv>(
        service_name_move_drone_,
        std::bind(&DroneNode::handle_move_drone_request,
                  this, std::placeholders::_1, std::placeholders::_2));

        // Set anchor and neighbors service server
        std::string service_name_anch_neigh = "set_anchor_and_neighbours" + std::to_string(id_);
        service_2_ = this->create_service<SetAnchorAndNeighbours>(
        service_name_anch_neigh,
        std::bind(&DroneNode::handle_anchor_and_neighbours_request,
                  this, std::placeholders::_1, std::placeholders::_2));

        // Calculate the phase angle and radius for the left drone
        std::string service_name_calc_angle= "calc_angle_" + std::to_string(id_);
        service_3_ = this->create_service<CalcAngle>(service_name_calc_angle,std::bind(&DroneNode::handle_calc_angle, 
        this, std::placeholders::_1, std::placeholders::_2));

        mutex_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        mutex_group_2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        mutex_group_3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        }
    private:

    void handle_drop_done_request(
      const std::shared_ptr<DropDroneSrv::Request> request,
      std::shared_ptr<DropDroneSrv::Response> response);
    // Service object
    rclcpp::Service<DropDroneSrv>::SharedPtr service_1_;

    void handle_move_drone_request(
      const std::shared_ptr<MoveDroneSrv::Request> request,
      std::shared_ptr<MoveDroneSrv::Response> response);
    // Service object
    rclcpp::Service<MoveDroneSrv>::SharedPtr service_move_;

    void handle_anchor_and_neighbours_request(
      const std::shared_ptr<SetAnchorAndNeighbours::Request> request,
      std::shared_ptr<SetAnchorAndNeighbours::Response> response);
    // Service object
    rclcpp::Service<SetAnchorAndNeighbours>::SharedPtr service_2_;

    void set_anchor_and_neighbours_response_callback(
    rclcpp::Client<SetAnchorAndNeighbours>::SharedFuture future);

    void handle_calc_angle(
      const std::shared_ptr<CalcAngle::Request> request,
      std::shared_ptr<CalcAngle::Response> response);
    // Service object
    rclcpp::Service<CalcAngle>::SharedPtr service_3_;

    void calc_angle_response_callback(rclcpp::Client<CalcAngle>::SharedFuture future);

    rclcpp::CallbackGroup::SharedPtr mutex_group_;
    rclcpp::CallbackGroup::SharedPtr mutex_group_2_;
    rclcpp::CallbackGroup::SharedPtr mutex_group_3_;

    rclcpp::CallbackGroup::SharedPtr pub_sub_group_;

    drone_swarm_msgs::msg::DroneStatus status_msg_;
    rclcpp::Publisher<drone_swarm_msgs::msg::DroneStatus>::SharedPtr publisher_;
    rclcpp::Publisher<drone_swarm_msgs::msg::MoveDrone>::SharedPtr publisher_move_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_2;
    drone_swarm_msgs::msg::MoveDrone move_msg_;
    rclcpp::Subscription<drone_swarm_msgs::msg::DroneStatus>::SharedPtr subscriber_left_;
    rclcpp::Subscription<drone_swarm_msgs::msg::DroneStatus>::SharedPtr subscriber_right_;
    void callback_left(const drone_swarm_msgs::msg::DroneStatus::SharedPtr msg);
    void callback_right(const drone_swarm_msgs::msg::DroneStatus::SharedPtr msg);
    void timer_callback();
    void Move_drone_callback();
    bool anchor_;
    int id_ ;
    double phase_angel_ ; /// deg
    double target_phase_angel_ ; /// deg
    double radius_ ; /// m
    int neighbour_left_ ;
    int neighbour_right_ ;
    float target_x_ ;
    float target_y_ ;
    float temp_target_x_ ;
    float temp_target_y_ ;
    float alpha_ ;
    float temp_alpha_ ;
    bool land_;

    // Left
    double phase_angel_left_ ; /// deg
    double radius_left_ ; /// m

    // Right
    double phase_angel_right_ ; /// deg
    double radius_right_ ; /// m



};