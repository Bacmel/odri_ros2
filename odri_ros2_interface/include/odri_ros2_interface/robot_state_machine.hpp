#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "odri_ros2_msgs/msg/driver_state.hpp"
#include "odri_ros2_msgs/msg/master_board_state.hpp"
#include "odri_ros2_msgs/msg/robot_command.hpp"
#include "odri_ros2_msgs/msg/robot_state.hpp"
#include "odri_ros2_msgs/msg/state_command.hpp"
#include "odri_ros2_msgs/srv/transition_command.hpp"

#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

namespace odri_ros2_interface
{

enum class StateType
{
    Idle,
    Calibrating,
    Enabled,
    Running,
    NbStates
};
enum class TransitionType
{
    Enable,
    Calibrate,
    Start,
    Disable,
    Stop,
    NbTransitions
};

static std::map<TransitionType, std::pair<StateType, StateType>> transitionStates_init_map()
{
    std::map<TransitionType, std::pair<StateType, StateType>> m;
    m.clear();
    m.insert({TransitionType::Enable, {StateType::Idle, StateType::Calibrating}});
    m.insert({TransitionType::Enable, {StateType::Calibrating, StateType::Enabled}});
    m.insert({TransitionType::Start, {StateType::Enabled, StateType::Running}});
    m.insert({TransitionType::Disable, {StateType::Enabled, StateType::Idle}});
    m.insert({TransitionType::Stop, {StateType::Running, StateType::Idle}});

    return m;
}

static std::map<StateType, std::string> stateName_init_map()
{
    std::map<StateType, std::string> m;
    m.clear();
    m.insert({StateType::Idle, "IDLE"});
    m.insert({StateType::Calibrating, "CALIBRATING"});
    m.insert({StateType::Enabled, "ENABLED"});
    m.insert({StateType::Running, "RUNNING"});

    return m;
}

static std::map<std::string, TransitionType> nameTransition_init_map()
{
    std::map<std::string, TransitionType> m;
    m.clear();
    m.insert({"enable", TransitionType::Enable});
    m.insert({"calibrate", TransitionType::Calibrate});
    m.insert({"start", TransitionType::Start});
    m.insert({"disable", TransitionType::Disable});
    m.insert({"stop", TransitionType::Stop});

    return m;
}

// This static variable stores which states a transition relates. origin and destination are stored as std pair
static const std::map<TransitionType, std::pair<StateType, StateType>> transition_states_map =
    transitionStates_init_map();
static const std::map<StateType, std::string>      state_name_map      = stateName_init_map();
static const std::map<std::string, TransitionType> name_transition_map = nameTransition_init_map();

class RobotStateMachine : public rclcpp::Node
{
  public:
    explicit RobotStateMachine(const std::string &node_name);
    virtual ~RobotStateMachine();

  private:
    void declareParameters();
    void createOdriRobot(const std::string &robot_yaml_path);

    void callbackTimerSendCommand();
    void callbackRobotCommand(const odri_ros2_msgs::msg::RobotCommand::SharedPtr msg);

    void transitionRequest(const std::shared_ptr<odri_ros2_msgs::srv::TransitionCommand::Request>  request,
                           const std::shared_ptr<odri_ros2_msgs::srv::TransitionCommand::Response> response);

    bool smDisable();
    bool smCalibrate();
    bool smEnable();
    bool smStart();
    bool smStop();

  private:
    rclcpp::TimerBase::SharedPtr timer_send_commands_;

    rclcpp::Publisher<odri_ros2_msgs::msg::RobotState>::SharedPtr  pub_robot_state_;
    rclcpp::Publisher<odri_ros2_msgs::msg::StateCommand>::SharedPtr  pub_state_command_;
    rclcpp::Subscription<odri_ros2_msgs::msg::RobotCommand>::SharedPtr sub_motor_command_;
    rclcpp::Service<odri_ros2_msgs::srv::TransitionCommand>::SharedPtr srv_sm_transition_;

    std::shared_ptr<odri_control_interface::Robot> odri_robot_;

    odri_ros2_msgs::msg::RobotState robot_state_msg_;
    odri_ros2_msgs::msg::StateCommand state_command_msg_;

    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;
    Eigen::VectorXd torques_;

    Eigen::VectorXd des_torques_;
    Eigen::VectorXd des_positions_;
    Eigen::VectorXd des_velocities_;
    Eigen::VectorXd des_pos_gains_;
    Eigen::VectorXd des_vel_gains_;
    Eigen::VectorXd max_currents_;

    struct Params
    {
        std::string     robot_yaml_path;
        Eigen::VectorXd safety_damping;
        Eigen::VectorXd safety_position;
        Eigen::VectorXd safety_gain;
        Eigen::VectorXd safety_current;
    } params_;

    StateType current_state_;
};

}  // namespace odri_ros2_interface