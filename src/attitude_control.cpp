#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		vehicle_attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		sensor_combined_subscriber_ = this->create_subscription<SensorGps>("/fmu/out/vehicle_gps_position", qos, std::bind(&OffboardControl::sensor_combined_callback, this, _1));
		

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_vehicle_attitude_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
	rclcpp::Subscription<SensorGps>::SharedPtr sensor_combined_subscriber_;

	std::atomic<uint64_t> timestamp_;   

	uint64_t offboard_setpoint_counter_;  

	void publish_offboard_control_mode();
	void publish_vehicle_attitude_setpoint();
	void loiter_control();

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

	SensorGps::SharedPtr sensor_data;
	void sensor_combined_callback(const SensorGps::SharedPtr msg) {sensor_data = msg;}
};


void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}


void OffboardControl::publish_vehicle_attitude_setpoint()
{
    VehicleAttitudeSetpoint msg{};

    double roll_value = sin(static_cast<double>(this->get_clock()->now().nanoseconds() / 1000000) / 2000);  // Adjust time scaling if needed

    double half_roll = roll_value / 3.0;
    msg.q_d = {float(cos(half_roll)), float(sin(half_roll)), 0.0, 0.0};  // Quaternion for roll (q_d = [w, x, y, z])

    msg.thrust_body = {0.35, 0.0, 0.0};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    vehicle_attitude_setpoint_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published roll: %f radians (converted to quaternion)", roll_value);
}


void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;

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