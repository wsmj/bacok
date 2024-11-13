#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <stdint.h>

#include <bacok/dubinsPath.h>
#include <bacok/frameTransport.h>

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
		sensor_combined_subscriber_ = this->create_subscription<SensorGps>("/fmu/out/vehicle_gps_position", qos, std::bind(&OffboardControl::sensor_combined_callback, this, _1));
		Vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControl::attitude_callback, this, _1));


		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				// this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}

			// publish_offboard_control_mode();
			// publish_vehicle_attitude_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<SensorGps>::SharedPtr sensor_combined_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr Vehicle_attitude_subscriber_;

	std::atomic<uint64_t> timestamp_;   

	uint64_t offboard_setpoint_counter_;  

	SensorGps::SharedPtr sensor_data;
	void sensor_combined_callback(const SensorGps::SharedPtr msg) {sensor_data = msg;}

    // VehicleAttitude::SharedPtr attsitude_data;
    void attitude_callback(const VehicleAttitude::SharedPtr msg) {
		Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
		// q.normalize();
		Eigen::Vector3d euler = quaternion::quaternion_to_euler(q);

		RCLCPP_INFO(this->get_logger(), "euler: %f, %f, %f", euler[0], euler[1], euler[2]);
		RCLCPP_INFO(this->get_logger(), "quater: %f, %f, %f, %f", msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    }
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