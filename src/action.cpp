#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_robot_interfaces/action/navigate.hpp"

#include <chrono>
#include <cmath>

using Navigate = my_robot_interfaces::action::Navigate;
using namespace std::chrono_literals;

class NavigateActionServer : public rclcpp::Node
{
public:
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  NavigateActionServer() : Node("navigate_action_server")
  {
    this->action_server_ = rclcpp_action::create_server<Navigate>(
      this,
      "navigate",
      std::bind(&NavigateActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigateActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigateActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request to navigate to x=%f, y=%f, z=%f", goal->x, goal->y, goal->z);
    (void)uuid;  // Unused variable
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    std::thread{std::bind(&NavigateActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Navigate::Result>();
    auto feedback = std::make_shared<Navigate::Feedback>();

    float current_position[3] = {0.0, 0.0, 0.0};
    float goal_position[3] = {goal->x, goal->y, goal->z};

    while (distance_to_goal(current_position, goal_position) > 0.1)
    {
      // Simulate movement towards the goal
      move_toward_goal(current_position, goal_position);
      
      feedback->distance_to_goal = distance_to_goal(current_position, goal_position);
      RCLCPP_INFO(this->get_logger(), "Current distance to goal: %f", feedback->distance_to_goal);

      // Publish feedback
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(1s);  // Simulate time taken for movement
    }

    if (rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      result->success = true;
      goal_handle->succeed(result);
    }
  }

  float distance_to_goal(float current_pos[3], float goal_pos[3])
  {
    return std::sqrt(
      std::pow(goal_pos[0] - current_pos[0], 2) +
      std::pow(goal_pos[1] - current_pos[1], 2) +
      std::pow(goal_pos[2] - current_pos[2], 2));
  }

  void move_toward_goal(float current_pos[3], float goal_pos[3])
  {
    float step = 0.5;
    for (int i = 0; i < 3; i++)
    {
      if (current_pos[i] < goal_pos[i])
        current_pos[i] += step;
      else if (current_pos[i] > goal_pos[i])
        current_pos[i] -= step;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
