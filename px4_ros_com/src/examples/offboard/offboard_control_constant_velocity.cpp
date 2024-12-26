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
		//
		object_available_ = false;
		// getting the namespace of the node
		node_namespace_ = this->get_namespace();

		RCLCPP_INFO_STREAM(this->get_logger(), "Node namespace: " << node_namespace_);
		// define the topics
		std::string offboard_control_mode_topic = node_namespace_ + "/fmu/in/offboard_control_mode";
		std::string trajectory_setpoint_topic = node_namespace_ + "/fmu/in/trajectory_setpoint";
		std::string vehicle_command_topic = node_namespace_ + "/fmu/in/vehicle_command";
		std::string detections_topic = node_namespace_ + "/detections";
		std::string object_available_topic = node_namespace_ + "/object_available";

		// create the publishers for the drone flight controller
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(offboard_control_mode_topic, 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(trajectory_setpoint_topic, 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(vehicle_command_topic, 10);
		
		// Subscription to "detections" topic
        sub_detections_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            detections_topic, rclcpp::QoS(10).best_effort(), 
            std::bind(&OffboardControl::detectionsCallback, this, std::placeholders::_1));

        // Subscription to "object_available" topic
        sub_object_available_ = this->create_subscription<std_msgs::msg::Bool>(
            object_available_topic, rclcpp::QoS(10).best_effort(), 
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
			if (offboard_setpoint_counter_ < 12) {
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
	int prev_position_x_;
	int prev_position_y_;
	int prev_position_z_;
	rclcpp::Time prev_time_;
	rclcpp::Time curr_time_;
	std::string node_namespace_;
	bool object_available_;
	void publish_offboard_control_mode();
	void publish_velcoity_control();
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
		float kpx = 0.0013f;
		float kpy = 0.0013f;
		float kpz = 0.0013f;	
		float kdx = 0.0000f;
		float kdy = 0.0000f;
		float kdz = 0.0000f;
		float dx = 0.0f;
		float dy = 0.0f;
		float dz = 0.0f;
			

		curr_time_ = this->now();
		if(prev_time_.seconds() != 0){
		
		float dt = (curr_time_ - prev_time_).seconds();

		dx = ((object_position_x_- prev_position_x_)/dt);
		dy = (object_position_y_ - prev_position_y_)/dt;
		dz = (object_position_z_- prev_position_z_)/dt;

			
		}
		
		// if (prev_time_.seconds() != 0){
		// RCLCPP_INFO_STREAM(this->get_logger(),"After Prev time is not 0");
		// float dt = (curr_time_ - prev_time_).seconds();
		// float dx = ((object_position_x_- prev_position_x_)/dt);
		// float dy = (object_position_y_ - prev_position_y_)/dt;
		// float dz = (object_position_z_- prev_position_z_)/dt;


		// RCLCPP_INFO_STREAM(this->get_logger(), "Velocity2: " << kdx*dx << " " << kdy*dy << " " << kdz*dz << " Time: " << dt);

		// trajectory_msg.velocity[0] = kpx*(object_position_x_-1200) + kdx*dx;
		// trajectory_msg.velocity[1] = kpy*(object_position_y_) + kdy*dy;
		// trajectory_msg.velocity[2] = kpz*(object_position_z_) + kdz*dz;
			
		// }
		// else{
		// RCLCPP_INFO_STREAM(this->get_logger(),"Prev time is 0");
		// trajectory_msg.velocity[0] = kpx*(object_position_x_-1200);
		// trajectory_msg.velocity[1] = kpy*(object_position_y_);
		// trajectory_msg.velocity[2] = kpz*(object_position_z_);
		// }

		trajectory_msg.velocity[0] = kpx*(object_position_x_-950) + kdx*dx;
		trajectory_msg.velocity[1] = kpy*(object_position_y_) + kdy*dy;
		trajectory_msg.velocity[2] = kpz*(object_position_z_) + kdz*dz;
		
		RCLCPP_INFO_STREAM(this->get_logger(), "Velocity1: " << trajectory_msg.velocity[0] << " " << trajectory_msg.velocity[1] << " " << trajectory_msg.velocity[2]);

		prev_position_x_ = object_position_x_;
		prev_position_y_ = object_position_y_;
		prev_position_z_ = object_position_z_;

		prev_time_ = this->now();
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
