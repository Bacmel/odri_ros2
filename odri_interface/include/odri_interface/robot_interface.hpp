#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "odri_msgs/msg/driver_state.hpp"
#include "odri_msgs/msg/master_board_state.hpp"
#include "odri_msgs/msg/robot_command.hpp"
#include "odri_msgs/msg/robot_state.hpp"
#include "odri_msgs/srv/transition_command.hpp"

#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

namespace odri_interface
{

class RobotInterface : public rclcpp::Node
{
  public:
    explicit RobotInterface(const std::string &node_name);
    virtual ~RobotInterface();

  private:
    void declareParameters();
    void createOdriRobot(const std::string &robot_yaml_path);

    void callbackTimerSendCommands();
    void callbackRobotCommand(const odri_msgs::msg::RobotCommand::SharedPtr msg);

    void transitionRequest(const std::shared_ptr<odri_msgs::srv::TransitionCommand::Request>  request,
                           const std::shared_ptr<odri_msgs::srv::TransitionCommand::Response> response);

    bool smDisable(std::string &message);
    bool smStartCalibration(std::string &message);
    bool smFinishCalibration(std::string &message);
    bool smEnable(std::string &message);
    bool smStop(std::string &message);

  private:
    rclcpp::TimerBase::SharedPtr timer_send_commands_;

    rclcpp::Publisher<odri_msgs::msg::RobotState>::SharedPtr pub_robot_state_;

    rclcpp::Subscription<odri_msgs::msg::RobotCommand>::SharedPtr subs_motor_commands_;

    rclcpp::Service<odri_msgs::srv::TransitionCommand>::SharedPtr service_sm_transition_;

    std::shared_ptr<odri_control_interface::Robot> odri_robot_;

    odri_msgs::msg::RobotState robot_state_msg_;

    std::chrono::high_resolution_clock::time_point t_last_mb_command_;

    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;

    Eigen::VectorXd des_torques_;
    Eigen::VectorXd des_positions_;
    Eigen::VectorXd des_velocities_;
    Eigen::VectorXd des_pos_gains_;
    Eigen::VectorXd des_vel_gains_;
    Eigen::VectorXd max_currents_;

    struct Params
    {
        std::string robot_yaml_path;
    } params_;

    enum class SmStates
    {
        Disabled,
        Calibrating,
        Idle,
        Enabled,
        NbStates
    };
    enum class SmTransitions
    {
        Disable,
        Calibrate,
        Enable,
        Stop,
        NbTransitions
    };

    SmStates                                          sm_active_state_;
    static const std::map<std::string, SmTransitions> sm_transitions_map;
    static const std::map<SmStates, std::string>      sm_states_map;

    static std::map<std::string, SmTransitions> createSmTransitionsMap();
    static std::map<SmStates, std::string>      createSmStatesMap();
};

}  // namespace odri_interface