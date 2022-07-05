#include "odri_ros2_interface/robot_interface.hpp"

namespace odri_ros2_interface
{
RobotInterface::RobotInterface(const std::string &node_name) : rclcpp::Node(node_name)
{
    declareParameters();
    createOdriRobot(params_.robot_yaml_path);

    service_sm_transition_ = create_service<odri_ros2_msgs::srv::TransitionCommand>(
        std::string(std::string(get_name()) + "/state_transition").c_str(),
        std::bind(&RobotInterface::transitionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    pub_robot_state_     = create_publisher<odri_ros2_msgs::msg::RobotState>("robot_state", 1);
    subs_motor_commands_ = create_subscription<odri_ros2_msgs::msg::RobotCommand>(
        "robot_command", rclcpp::QoS(1),
        std::bind(&RobotInterface::callbackRobotCommand, this, std::placeholders::_1));

    timer_send_commands_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                             std::bind(&RobotInterface::callbackTimerSendCommands, this));

    positions_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    velocities_ = positions_;
    torques_    = positions_;

    des_torques_    = positions_;
    des_positions_  = positions_;
    des_velocities_ = positions_;
    des_pos_gains_  = positions_;
    des_vel_gains_  = positions_;
    max_currents_   = positions_;

    sm_active_state_ = SmStates::Disabled;
}

RobotInterface::~RobotInterface() {}

void RobotInterface::declareParameters()
{
    declare_parameter<std::string>("robot_yaml_path", "");
    get_parameter<std::string>("robot_yaml_path", params_.robot_yaml_path);
}

void RobotInterface::createOdriRobot(const std::string &robot_yaml_path)
{
    odri_robot_ = odri_control_interface::RobotFromYamlFile(robot_yaml_path);
    odri_robot_->Start();
    odri_robot_->WaitUntilReady();
}

void RobotInterface::callbackTimerSendCommands()
{
    odri_robot_->ParseSensorData();

    positions_  = odri_robot_->joints->GetPositions();
    velocities_ = odri_robot_->joints->GetVelocities();
    torques_    = odri_robot_->joints->GetMeasuredTorques();

    robot_state_msg_.header.stamp = get_clock()->now();
    robot_state_msg_.motor_states.clear();

    for (long int i = 0; i < positions_.size(); ++i)
    {
        odri_ros2_msgs::msg::MotorState m_state;

        m_state.position = positions_(i);
        m_state.velocity = velocities_(i);
        m_state.torque   = torques_(i);

        robot_state_msg_.motor_states.push_back(m_state);
    }
    pub_robot_state_->publish(robot_state_msg_);

    switch (sm_active_state_)
    {
        case SmStates::Enabled:
            odri_robot_->joints->SetTorques(des_torques_);  // WARNING: mixing current and torques. Change the msg
            odri_robot_->joints->SetDesiredPositions(des_positions_);
            odri_robot_->joints->SetDesiredVelocities(des_velocities_);
            odri_robot_->joints->SetPositionGains(des_pos_gains_);
            odri_robot_->joints->SetVelocityGains(des_vel_gains_);
            odri_robot_->joints->SetMaximumCurrents(
                max_currents_(0));  // WARNING: Max current is common for all joints
            break;

        default:
            Eigen::VectorXd vec_zero = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
            odri_robot_->joints->SetTorques(vec_zero);
            odri_robot_->joints->SetDesiredPositions(vec_zero);
            odri_robot_->joints->SetDesiredVelocities(vec_zero);
            odri_robot_->joints->SetPositionGains(vec_zero);
            odri_robot_->joints->SetVelocityGains(vec_zero);
            break;
    }

    if (!odri_robot_->SendCommand())
    {
        odri_robot_->ReportError();
        createOdriRobot(params_.robot_yaml_path);
        sm_active_state_ = SmStates::Disabled;
        RCLCPP_INFO_STREAM(get_logger(), "\nError detected: odri in DESABLED state.");
    }
}

void RobotInterface::callbackRobotCommand(const odri_ros2_msgs::msg::RobotCommand::SharedPtr msg)
{
    if (sm_active_state_ == SmStates::Enabled)
    {
        for (std::size_t i = 0; i < msg->motor_commands.size(); ++i)
        {
            des_torques_(i)    = msg->motor_commands[i].torque_ref;
            des_positions_(i)  = msg->motor_commands[i].position_ref;
            des_velocities_(i) = msg->motor_commands[i].velocity_ref;
            des_pos_gains_(i)  = msg->motor_commands[i].kp;
            des_vel_gains_(i)  = msg->motor_commands[i].kd;
            max_currents_(i)   = msg->motor_commands[i].i_sat;
        }
    }
}

void RobotInterface::transitionRequest(const std::shared_ptr<odri_ros2_msgs::srv::TransitionCommand::Request>  request,
                                       const std::shared_ptr<odri_ros2_msgs::srv::TransitionCommand::Response> response)
{
    RCLCPP_INFO_STREAM(get_logger(), "Service request received");

    SmTransitions command = SmTransitions::NbTransitions;
    if (sm_transitions_map.find(request->command) != sm_transitions_map.end())
    {
        command = sm_transitions_map.at(request->command);
    }

    switch (command)
    {
        case SmTransitions::Disable:
            if (sm_active_state_ == SmStates::Calibrating)
            {
                response->accepted = smDisable(response->message);
                if (response->accepted)
                {
                    sm_active_state_ = SmStates::Disabled;
                }
            }
            else if (sm_active_state_ == SmStates::Enabled)
            {
                response->accepted = smDisable(response->message);
                if (response->accepted)
                {
                    sm_active_state_ = SmStates::Idle;
                }
            }
            else
            {
                response->accepted = false;
                response->message  = "Cannot disable the odri Interface. It is already in DISABLED/IDLE state.";
            }
            break;

        case SmTransitions::Calibrate:
            if (sm_active_state_ == SmStates::Disabled || sm_active_state_ == SmStates::Idle)
            {
                sm_active_state_   = SmStates::Disabled;
                response->accepted = smStartCalibration(response->message);
                if (response->accepted)
                {
                    sm_active_state_ = SmStates::Calibrating;
                }
            }
            else if (sm_active_state_ == SmStates::Calibrating)
            {
                response->accepted = smFinishCalibration(response->message);
                if (response->accepted)
                {
                    sm_active_state_ = SmStates::Idle;
                }
            }
            else
            {
                response->accepted = false;
                response->message  = "Cannot calibrate the odri Interface. It is not in the DISABLED/IDLE state.";
            }
            break;

        case SmTransitions::Enable:
            response->accepted = smEnable(response->message);
            if (response->accepted)
            {
                sm_active_state_ = SmStates::Enabled;
            }
            break;

        case SmTransitions::Stop:
            response->accepted = smStop(response->message);
            if (response->accepted)
            {
                sm_active_state_ = SmStates::Disabled;
            }
            break;

        default:
            response->accepted = false;
            response->message  = "Command: " + request->command +
                                " does not exist. Possible options are: 'enable'|'disable'|'calibrate'|'stop'";
            RCLCPP_WARN_STREAM(get_logger(), response->message);
            break;
    }

    response->result = sm_states_map.at(sm_active_state_);
    RCLCPP_WARN_STREAM(get_logger(), response->message);
}

bool RobotInterface::smDisable(std::string &message)
{
    sm_active_state_ = SmStates::Disabled;
    message          = "ODRI disabled";
    return true;
}

bool RobotInterface::smStartCalibration(std::string &message)
{
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
    odri_robot_->GetJoints()->SetPositionOffsets(zero_vec);

    RCLCPP_INFO_STREAM(get_logger(), "\nCalibration procedure: Finding indexes...");
    odri_robot_->RunCalibration(zero_vec);

    RCLCPP_INFO_STREAM(get_logger(),
                       "\nCalibration procedure: Put all joints in their respective zero positions. Call state "
                       "transition service with 'calibrate' again");

    message = "Calibration running. Do required steps.";

    return true;
}

bool RobotInterface::smFinishCalibration(std::string &message)
{
    RCLCPP_INFO_STREAM(get_logger(), "\nThese are the offsets");
    Eigen::VectorXd current_pos = odri_robot_->GetJoints()->GetPositions();

    for (long int i = 0; i < current_pos.size(); i++)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << current_pos(i));
    }

    odri_robot_->GetJoints()->SetPositionOffsets(-current_pos);

    message = "Calibration done";

    return true;
}

bool RobotInterface::smEnable(std::string &message)
{
    if (sm_active_state_ == SmStates::Idle)
    {
        message = "Odri enabled";
        return true;
    }
    else
    {
        message = "Cannot enable the odri Interface. It is not in the IDLE state.";
        return false;
    }
}

bool RobotInterface::smStop(std::string &message)
{
    odri_robot_->ReportError();
    createOdriRobot(params_.robot_yaml_path);
    message = "Odri disabled";
    return true;
}

std::map<std::string, RobotInterface::SmTransitions> RobotInterface::createSmTransitionsMap()
{
    std::map<std::string, SmTransitions> m;
    m["disable"]   = SmTransitions::Disable;
    m["calibrate"] = SmTransitions::Calibrate;
    m["enable"]    = SmTransitions::Enable;
    m["stop"]      = SmTransitions::Stop;
    return m;
}

std::map<RobotInterface::SmStates, std::string> RobotInterface::createSmStatesMap()
{
    std::map<SmStates, std::string> m;
    m[SmStates::Disabled]    = "disabled";
    m[SmStates::Calibrating] = "calibrating";
    m[SmStates::Idle]        = "idle";
    m[SmStates::Enabled]     = "enabled";
    return m;
}

const std::map<std::string, RobotInterface::SmTransitions> RobotInterface::sm_transitions_map =
    RobotInterface::createSmTransitionsMap();
const std::map<RobotInterface::SmStates, std::string> RobotInterface::sm_states_map =
    RobotInterface::createSmStatesMap();

}  // namespace odri_ros2_interface

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_ros2_interface::RobotInterface> master_board_iface =
        std::make_shared<odri_ros2_interface::RobotInterface>("RobotInterface");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(master_board_iface);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}