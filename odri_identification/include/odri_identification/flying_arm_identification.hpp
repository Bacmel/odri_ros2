#pragma once

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "odri_msgs/srv/transition_command.hpp"
#include "odri_msgs/msg/robot_state.hpp"
#include "odri_msgs/msg/robot_command.hpp"

class FlyingArmIdentification : public rclcpp::Node
{
    public:
    explicit FlyingArmIdentification(const std::string &node_name);
    virtual ~FlyingArmIdentification();

    private:
    void declareParameters();
    void createCommands();

    void callbackTimerPublishCommand();
    void callbackRobotState(const odri_msgs::msg::RobotState::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult callbackParameters(const std::vector<rclcpp::Parameter> &parameters);

    private:
    rclcpp::TimerBase::SharedPtr timer_publish_command_;

    rclcpp::Subscription<odri_msgs::msg::RobotState>::SharedPtr  sub_robot_state_;
    rclcpp::Publisher<odri_msgs::msg::RobotCommand>::SharedPtr   pub_robot_command_;
    rclcpp::Client<odri_msgs::srv::TransitionCommand>::SharedPtr client_odri_interface_;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    odri_msgs::msg::RobotCommand msg_robot_command_;

    struct SetPoint {
        double t;
        double pos[2];
        double vel[2];
        double acc[2];
    };

    struct CommandParams {
        std::vector<SetPoint> commands;
        std::size_t curr_idx{0};
    } command_params_;

    bool            brought_to_init_;
    Eigen::Vector2d pos_error_;

    struct Params {
        std::string trajectory_csv_path;
        bool publish_commands;
    } params_;

    bool        got_initial_position_;
    std::size_t counter_initial_position_;
};
