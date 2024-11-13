#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
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

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // After 10 setpoints, switch to offboard mode and arm the vehicle
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard mode
                this->arm();
                this->takeoff();
            }

            // Publish the offboard control mode
            publish_offboard_control_mode();

            // Send trajectory setpoints for takeoff (airspeed and altitude)
            publish_trajectory_setpoint();

            // Increment the counter up to a limit
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
		
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void takeoff();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void OffboardControl::takeoff() {
    // Parameters for takeoff:
    // param1: Minimum pitch (degrees, set to 0 for standard takeoff)
    // param2: Empty (set to 0)
    // param3: Empty (set to 0)
    // param4: Yaw angle in radians (optional, use NAN if not needed)
    // param5: Latitude (NAN for current location)
    // param6: Longitude (NAN for current location)
    // param7: Altitude above ground level (e.g., 50 meters)
    
    publish_vehicle_command(
        VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,  // Takeoff command
        0.0,                                      // Minimum pitch
        0.0,                                      // Unused
        0.0,                                      // Unused
        NAN,                                      // Yaw (optional)
        NAN,                                      // Latitude (use current)
        NAN,                                      // Longitude (use current)
        50.0                                      // Altitude (e.g., 50 meters)
    );

    RCLCPP_INFO(this->get_logger(), "Takeoff command sent");
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
 * @brief Publish a trajectory setpoint
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
    msg.velocity = {15.0, 0.0, -2.0};  // Forward airspeed of 15 m/s, slight descent (-2.0 m/s)
    msg.position = {NAN, NAN, NAN};    // We don't specify position during takeoff
    msg.yaw = NAN;                     // No yaw control
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
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
