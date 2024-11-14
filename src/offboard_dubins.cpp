#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <bohoso/msg/team_telemetry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <stdint.h>

#include <bacok/dubinsPath.h>
#include <bacok/frameTransport.h>
#include <bacok/geolocation.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace bohoso::msg;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		team2_telemetry_subscriber_ = this->create_subscription<TeamTelemetry>("/team2/telemetry", qos, std::bind(&OffboardControl::team2_telemetry_callback, this, _1));
		sensor_combined_subscriber_ = this->create_subscription<SensorGps>("/fmu/out/vehicle_gps_position", qos, std::bind(&OffboardControl::sensor_combined_callback, this, _1));
		Vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControl::attitude_callback, this, _1));

		auto timer_callback = [this]() -> void {
			RCLCPP_INFO(this->get_logger(), "W: %f", attitude_data->q[0]);
			RCLCPP_INFO(this->get_logger(), "X: %f", attitude_data->q[1]);
			RCLCPP_INFO(this->get_logger(), "Y: %f", attitude_data->q[2]);
			RCLCPP_INFO(this->get_logger(), "Z: %f", attitude_data->q[3]);
			
			FrameTransport::Quaternion quaternion;
			quaternion.w = attitude_data->q[0];
			quaternion.x = attitude_data->q[1];
			quaternion.y = attitude_data->q[2];
			quaternion.z = attitude_data->q[3];

			FrameTransport::Euler euler = FrameTransport::toEuler(quaternion);
			RCLCPP_INFO(this->get_logger(), "roll: %f", euler.roll*180/M_PI);
			RCLCPP_INFO(this->get_logger(), "pitch: %f", euler.pitch*180/M_PI);
			RCLCPP_INFO(this->get_logger(), "yaw: %f", euler.yaw*180/M_PI);
			RCLCPP_INFO(this->get_logger(), "\n\n\n");

			// double turn_radius = 30;
			// geo_utils::Position position;
			

		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<TeamTelemetry>::SharedPtr team2_telemetry_subscriber_;
	rclcpp::Subscription<SensorGps>::SharedPtr sensor_combined_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr Vehicle_attitude_subscriber_;

	std::atomic<uint64_t> timestamp_;   


	SensorGps::SharedPtr sensor_data;
	void sensor_combined_callback(const SensorGps::SharedPtr msg) {sensor_data = msg;}

    VehicleAttitude::SharedPtr attitude_data;
    void attitude_callback(const VehicleAttitude::SharedPtr msg) {attitude_data = msg;}

	TeamTelemetry::SharedPtr team2_telemetry;
	void team2_telemetry_callback(const TeamTelemetry::SharedPtr msg) {team2_telemetry = msg;}


	
};


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}