#pragma once

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "odri_msgs/srv/transition_command.hpp"
#include "odri_msgs/msg/robot_state.hpp"

class FlyingArmIdentification : public rclcpp::Node
{
    public:
    explicit FlyingArmIdentification(const std::string &node_name);
    virtual ~FlyingArmIdentification();

    private:
    void callbackTimerChangeCommand();
    void callbackTimerPublishCommand();

    void                                     callbackRobotState(const odri_msgs::msg::RobotState::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult callbackParameters(const std::vector<rclcpp::Parameter> &parameters);

    private:
    rclcpp::TimerBase::SharedPtr timer_change_command_;
    rclcpp::TimerBase::SharedPtr timer_publish_command_;

    struct CommandParams {
        Eigen::Vector2d pos;
        Eigen::Vector2d vel;
        Eigen::Vector2d acc;
        Eigen::Vector2d t;
    } command_params_;
};
