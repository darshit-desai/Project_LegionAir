/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info.
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <drone_swarm_msgs/msg/move_drone.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

const rmw_qos_profile_t rmw_qos_profile_offboard_pub =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		std::string px4_namespace = this->get_namespace();
		rmw_qos_profile_t qos_profile = rmw_qos_profile_offboard_pub;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		rclcpp::QoS qos_assured(rclcpp::KeepLast(5));
		qos_assured.best_effort();
		qos_assured.durability_volatile();
		moving_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions moving_options;
		moving_options.callback_group = moving_callback_group_;
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(px4_namespace+"/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(px4_namespace+"/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(px4_namespace+"/fmu/in/vehicle_command", qos);
		vehicle_local_position_sub_ = this->create_subscription<VehicleLocalPosition>(px4_namespace+"/fmu/out/vehicle_local_position", qos_assured, std::bind(&OffboardControl::feedback_position_callback, this, std::placeholders::_1), moving_options);
		vehicle_control_mode_sub_ = this->create_subscription<VehicleControlMode>(px4_namespace+"/fmu/out/vehicle_control_mode", qos_assured, std::bind(&OffboardControl::vehicle_mode_callback, this, std::placeholders::_1), moving_options);
		std::string move_drone_topic = "/move_drone_" + std::to_string(id_);
		move_drone_sub_ = this->create_subscription<drone_swarm_msgs::msg::MoveDrone>(move_drone_topic, 10, std::bind(&OffboardControl::target_position_callback, this, std::placeholders::_1), moving_options);
		offboard_setpoint_counter_ = 0;
		// position controller runs at 50Hz
		timer_ = this->create_wall_timer(20ms, std::bind(&OffboardControl::timer_callback, this), moving_callback_group_);
		vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(px4_namespace+"/fmu/out/vehicle_status", qos_assured, std::bind(&OffboardControl::status_callback, this, std::placeholders::_1), moving_options);
	}
	void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
	void arm();
	void disarm();
	void land();
	void timer_callback();
	void engage_offBoard_mode();
	void target_position_callback(const drone_swarm_msgs::msg::MoveDrone::SharedPtr msg);
	void feedback_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void vehicle_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
	// Body frame positions
	float x_position = 0.0; 
	float y_position = 0.0;
	bool land_var = false;
	float alpha_yaw = 0.0;
	// ID of the drone
	int id_ = 0;
	// Current position feedback
	float x_feedback = 0.0;
	float y_feedback = 0.0;
	// Target position
	float x_target = 0.0;
	float y_target = 0.0;
	bool vehicle_mode_offboard_ = false;

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<drone_swarm_msgs::msg::MoveDrone>::SharedPtr move_drone_sub_;
	rclcpp::CallbackGroup::SharedPtr moving_callback_group_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
	rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y);
	void publish_vehicle_command(VehicleCommand msg);
};

void OffboardControl::target_position_callback(const drone_swarm_msgs::msg::MoveDrone::SharedPtr msg)
{
	x_position = msg->target_pos_x.data;
	y_position = msg->target_pos_y.data;
	alpha_yaw = msg->alpha.data;
	land_var = msg->land.data;
	// RCLCPP_INFO_STREAM(this->get_logger(), "Received position: x=" << x_position << " y=" << y_position << " alpha=" << alpha_yaw);
}

void OffboardControl::timer_callback() {
	RCLCPP_INFO_STREAM(this->get_logger(), "Offboard setpoint counter: " << offboard_setpoint_counter_);	
	if (offboard_setpoint_counter_ == 50) {
		// Change to Offboard mode after 50 setpoints (1s)
		this->engage_offBoard_mode();
		
		// Arm the vehicle
		this->arm();
	}
	if (offboard_setpoint_counter_ == 15550){
		// Land and cancel timer after (11s)
		this->land();

		this->timer_->cancel();
	}
	if (offboard_setpoint_counter_ < 25000) {
		// offboard_control_mode needs to be paired with trajectory_setpoint
		publish_offboard_control_mode();
		if (vehicle_mode_offboard_) {
			publish_trajectory_setpoint(x_position, y_position);
		}
		if (land_var) {
			this->land();
			this->timer_->cancel();
		}

		offboard_setpoint_counter_++;
	}
}

void OffboardControl::feedback_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	VehicleLocalPosition vehicle_local_position = *msg;
	// RCLCPP_INFO_STREAM(this->get_logger(), "I heard something in feedback_position_callback");
	// RCLCPP_INFO_STREAM(this->get_logger(), "Feedback position: x=" << vehicle_local_position.x << " y=" << vehicle_local_position.y << " z=" << vehicle_local_position.z);
	x_feedback = vehicle_local_position.x;
	y_feedback = vehicle_local_position.y;
}

void OffboardControl::status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
	px4_msgs::msg::VehicleStatus vehicle_status = *msg;
	vehicle_mode_offboard_ = vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	VehicleCommand msg{};

	msg.param1 = 1;
	msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	VehicleCommand msg{};

	msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to Land the vehicle
 */
void OffboardControl::land()
{
	VehicleCommand msg{};

	msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;

	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Land command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};

	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Engage the offboard control mode.
 */
void OffboardControl::engage_offBoard_mode()
{
	VehicleCommand msg{};

	msg.param1 = 1;
	msg.param2 = 6;
	msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
	msg.target_system = 1;
    	msg.target_component = 1;
    	msg.source_system = 1;
    	msg.source_component = 1;
    	msg.from_external = true;

	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, -0.5};
	msg.yaw = 0.0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command message
 */
void OffboardControl::publish_vehicle_command(VehicleCommand msg)
{
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::vehicle_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
	VehicleControlMode vehicle_control_mode = *msg;
	vehicle_mode_offboard_ = vehicle_control_mode.flag_control_offboard_enabled;
	RCLCPP_INFO_STREAM(this->get_logger(), "I heard something in vehicle_mode_callback");
	RCLCPP_INFO_STREAM(this->get_logger(), "Vehicle mode: offboard=" << vehicle_mode_offboard_);
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
