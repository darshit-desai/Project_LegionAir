#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		// std::string px4_namespace = this->get_namespace();
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/uav_1/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/uav_1/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/uav_1/fmu/in/vehicle_command", 10);
		std::string topic_name_detections = "/uav_1/predicted_position";
        std::string topic_name_object_available = "/uav_1/object_available";
		// Subscription to "detections" topic
        sub_detections_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            topic_name_detections, rclcpp::QoS(10).best_effort(), 
            std::bind(&OffboardControl::detectionsCallback, this, std::placeholders::_1));

        // Subscription to "object_available" topic
        sub_object_available_ = this->create_subscription<std_msgs::msg::Bool>(
            topic_name_object_available, rclcpp::QoS(10).best_effort(), 
            std::bind(&OffboardControl::objectAvailableCallback, this, std::placeholders::_1));

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(33ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_detections_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_object_available_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	int object_position_x_;
	int object_position_y_;
	int object_position_z_;
	bool object_available_;
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void detectionsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
	void objectAvailableCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

void OffboardControl::detectionsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
	object_position_x_ = msg->point.x;
	object_position_y_ = msg->point.y;
	object_position_z_ = msg->point.z;
}
void OffboardControl::objectAvailableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	object_available_ = msg->data;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}



/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint trajectory_msg;
	trajectory_msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
	if(object_available_){
		RCLCPP_INFO_STREAM(this->get_logger(), "Object position: " << object_position_x_ << " " << object_position_y_ << " " << object_position_z_);
		float vel = 0.2f;	
		if(object_position_x_ < 1150){
			int error = 1150 - object_position_x_;
			error = abs(error);
			trajectory_msg.velocity[0] = -vel;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Moving backwards");
		}
		else if(object_position_x_ > 1200){
			trajectory_msg.velocity[0] = vel;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Moving forwards");
		}
		else{
			trajectory_msg.velocity[0] = 0.0f;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Staying still");
		}
		if(object_position_y_ < -50){
			trajectory_msg.velocity[1] = -vel;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Moving left");
		}
		else if(object_position_y_ > 50){
			trajectory_msg.velocity[1] = vel;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Moving right");
		}
		else{
			trajectory_msg.velocity[1] = 0.0f;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Staying still");
		}
		if(object_position_z_ < -50){
			trajectory_msg.velocity[2] = -vel;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Moving up");
		}
		else if(object_position_z_ > 50){
			trajectory_msg.velocity[2] = vel;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Moving down");
		}
		else{
			trajectory_msg.velocity[2] = 0.0f;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Staying still");
		}

	}
	else{
		RCLCPP_INFO_STREAM(this->get_logger(), "Object not available");
		trajectory_msg.velocity[0] = 0.0f;
		trajectory_msg.velocity[1] = 0.0f;
		trajectory_msg.velocity[2] = 0.0f;
	}
	trajectory_msg.position[0] = NAN;
	trajectory_msg.position[1] = NAN;
	trajectory_msg.position[2] = NAN;
	trajectory_msg.acceleration[0] = NAN;
	trajectory_msg.acceleration[1] = NAN;
	trajectory_msg.acceleration[2] = NAN;
	trajectory_msg.yaw = NAN;
	trajectory_msg.yawspeed = 0.0f;

	trajectory_setpoint_publisher_->publish(trajectory_msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
