#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/vehicle>
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
		sensor_combined_subscriber_ = this->create_subscription<SensorGps>("/fmu/out/vehicle_gps_position", qos, std::bind(&OffboardControl::sensor_combined_callback, this, _1));
		Vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControl::attitude_callback, this, _1));
        
        auto timer_callback = [this]() -> void {
            RCLCPP_INFO(this->get_logger(), "team number 1");
            RCLCPP_INFO(this->get_logger(), "uav_latitude: %f", sensor_data->latitude_deg);
            RCLCPP_INFO(this->get_logger(), "uav_longitude: %f", sensor_data->longitude_deg);
            RCLCPP_INFO(this->get_logger(), "uav_altitude: %f", sensor_data->altitude_msl_m);
            

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

    VehicleAttitude::SharedPtr attitude_data;
    void attitude_callback(const VehicleAttitude::SharedPtr msg) {attitude_data = msg;}
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