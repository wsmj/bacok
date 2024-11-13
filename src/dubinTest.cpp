#include <bacok/dubinsPath.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>

#define PI M_PI

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    double turn_radius = 1.0;
    double x0 = 0.0, y0 = 0.0, theta0 = 0.0;
    double x1 = 10.0, y1 = 10.0, theta1 = PI / 2;

    dubins_path_planner::DubinsPathCalculator calculator(turn_radius);
    dubins_path_planner::DubinsPath best_path = calculator.computeDubinsPath(x0, y0, theta0, x1, y1, theta1);

    std::cout << "The best path type is: ";
    switch (best_path.type) {
        case dubins_path_planner::LSL: std::cout << "LSL"; break;
        case dubins_path_planner::RSR: std::cout << "RSR"; break;
        case dubins_path_planner::LSR: std::cout << "LSR"; break;
        case dubins_path_planner::RSL: std::cout << "RSL"; break;
        case dubins_path_planner::RLR: std::cout << "RLR"; break;
        case dubins_path_planner::LRL: std::cout << "LRL"; break;
    }
    std::cout << " with a total length of: " << best_path.length << std::endl;

    rclcpp::shutdown();

    return 0;
}
