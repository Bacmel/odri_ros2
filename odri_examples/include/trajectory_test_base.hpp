#pragma once

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "odri_ros2_msgs/srv/transition_command.hpp"
#include "odri_ros2_msgs/msg/robot_state.hpp"
#include "odri_ros2_msgs/msg/robot_command.hpp"

namespace odri_ros2_identification
{

enum class TrajectoryTestTypes
{
    Wave,
    Stretch,
    Offline,
    NbTrajectoryTestTypes
};

static std::map<std::string, TrajectoryTestTypes> TrajectoryTestTypes_init_map()
{
    std::map<std::string, TrajectoryTestTypes> m;
    m.clear();
    m.insert({"Wave", TrajectoryTestTypes::Wave});
    m.insert({"Stretch", TrajectoryTestTypes::Stretch});
    m.insert({"Offline", TrajectoryTestTypes::Offline});
    return m;
}

static const std::map<std::string, TrajectoryTestTypes> TrajectoryTestTypes_map = TrajectoryTestTypes_init_map();

class TrajectoryTestAbstract : public rclcpp::Node
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryTestAbstract(const std::string& node_name);
    virtual ~TrajectoryTestAbstract();

  private:
    rclcpp::Publisher<odri_ros2_msgs::msg::RobotCommand>::SharedPtr   pub_robot_command_;
    rclcpp::Client<odri_ros2_msgs::srv::TransitionCommand>::SharedPtr client_odri_interface_;



  struct TrajectoryTestAbstractParams {

  } base_params_;
};

class TrajectoryTestFactory
{
  public:
  ~TrajectoryTestFactory();

  static std::shared_ptr<TrajectoryTestAbstract> create(const std::string& node_name, const TrajectoryTestTypes& test_type);

  private:
  TrajectoryTestFactory();
};

}  // namespace odri_ros2_identification