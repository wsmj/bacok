#include <bacok/frameTransport.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>

#define PI M_PI

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Eigen::Quaterniond q(0.707, 0.0, 0.707, 0.0); // Example quaternion

    q.normalize();

    Eigen::Vector3d euler = quaternion::quaternion_to_euler(q);

    std::cout << "Quaternion to Euler Conversion:\n";
    std::cout << "Quaternion: [w=" << q.w() << ", x=" << q.x() << ", y=" << q.y() << ", z=" << q.z() << "]\n";
    std::cout << "Euler Angles (roll, pitch, yaw): [" << euler[0]*180/M_PI << ", " << euler[1]*180/M_PI << ", " << euler[2]*180/M_PI << "]\n";


    rclcpp::shutdown();

    return 0;
}
