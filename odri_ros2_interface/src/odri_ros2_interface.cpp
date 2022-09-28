#include "odri_ros2_interface/odri_ros2_interface.hpp"

namespace odri_ros2_interface
{

OdriRos2Interface::OdriRos2Interface(const std::string &node_name) : rclcpp::Node(node_name)
{
    declareParameters();
    createOdriRobot(params_.robot_yaml_path);

    pub_robot_state_ = create_publisher<odri_ros2_msgs::msg::RobotState>("robot_state", 1);

    sub_robot_command_ = create_subscription<odri_ros2_msgs::msg::RobotCommand>(
        "robot_command", rclcpp::QoS(1),
        std::bind(&OdriRos2Interface::callbackRobotCommand, this, std::placeholders::_1));

    srv_transition_command_ = create_service<odri_ros2_msgs::srv::TransitionCommand>(
        "transition_command",
        std::bind(&OdriRos2Interface::callbackTransitionCommand, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    timer_exchange_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                        std::bind(&OdriRos2Interface::callbackTimerExchange, this));

    // Init eigen vectors
    vec_zero_ = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());

    positions_  = vec_zero_;
    velocities_ = vec_zero_;
    torques_    = vec_zero_;

    des_torques_    = vec_zero_;
    des_positions_  = vec_zero_;
    des_velocities_ = vec_zero_;
    des_pos_gains_  = vec_zero_;
    des_vel_gains_  = vec_zero_;
    max_currents_   = vec_zero_;
}

OdriRos2Interface::~OdriRos2Interface() {}

void OdriRos2Interface::declareParameters()
{
    std::vector<double> tmp;
    declare_parameter<std::string>("robot_yaml_path", "");
    get_parameter<std::string>("robot_yaml_path", params_.robot_yaml_path);
    declare_parameter<std::vector<double> >("parking_position", std::vector<double>());
    get_parameter<std::vector<double> >("parking_position", tmp);
    params_.parking_position = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

}

void OdriRos2Interface::createOdriRobot(const std::string &robot_yaml_path)
{
    odri_robot_ = odri_control_interface::RobotFromYamlFile(robot_yaml_path);
    odri_robot_->Start();
    odri_robot_->WaitUntilReady();
}

void OdriRos2Interface::callbackTimerExchange()
{
    // First, send robot state
    publishRobotState();

    // Second, send robot command regarding the state
    switch (current_state_)
    {
        case StateType::Running: {
            odri_robot_->joints->SetTorques(des_torques_);
            odri_robot_->joints->SetDesiredPositions(des_positions_);
            odri_robot_->joints->SetDesiredVelocities(des_velocities_);
            odri_robot_->joints->SetPositionGains(des_pos_gains_);
            odri_robot_->joints->SetVelocityGains(des_vel_gains_);
            odri_robot_->joints->SetMaximumCurrents(
                max_currents_(0));  // WARNING: Max current is common for all joints
            break;
        }
        case StateType::Enabled: {
            odri_robot_->joints->RunSafetyController();
            odri_robot_->joints->SetDesiredPositions(params_.parking_position);
            break;
        }
        default: {
            odri_robot_->joints->SetTorques(vec_zero_);
            odri_robot_->joints->SetDesiredPositions(vec_zero_);
            odri_robot_->joints->SetDesiredVelocities(vec_zero_);
            odri_robot_->joints->SetPositionGains(vec_zero_);
            odri_robot_->joints->SetVelocityGains(vec_zero_);
            break;
        }
    }

    if (!odri_robot_->SendCommand())
    {
        if (smStop())
        {
            RCLCPP_INFO_STREAM(get_logger(), "odri go to ENABLED state.");
        }
        else
        {
            smDisable();
            RCLCPP_INFO_STREAM(get_logger(), "odri go to DISABLED state.");
        }
    }
}

void OdriRos2Interface::callbackRobotCommand(const odri_ros2_msgs::msg::RobotCommand::SharedPtr msg)
{
    if (current_state_ == StateType::Running)
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

void OdriRos2Interface::callbackTransitionCommand(
    const std::shared_ptr<odri_ros2_msgs::srv::TransitionCommand::Request>  request,
    const std::shared_ptr<odri_ros2_msgs::srv::TransitionCommand::Response> response)
{
    RCLCPP_INFO_STREAM(get_logger(), "Service request received");

    TransitionType command = TransitionType::NbTransitions;
    if (name_transition_map.find(request->command) != name_transition_map.end())
    {
        command = name_transition_map.at(request->command);
    }

    switch (command)
    {
        case TransitionType::Enable:
            response->accepted = smEnable();
            break;
        case TransitionType::Start:
            response->accepted = smStart();
            break;
        case TransitionType::Disable:
            response->accepted = smDisable();
            break;
        case TransitionType::Stop:
            response->accepted = smStop();
            break;
        case TransitionType::Calibrate:
            response->accepted = smCalibrate();
            break;
        default:
            response->accepted = false;
            response->message  = "Command: " + request->command +
                                " does not exist. Possible options are: 'enable'|'disable'|'calibrate'|'start'|'stop'";
            RCLCPP_WARN_STREAM(get_logger(), response->message);
            break;
    }
    if (response->accepted)
    {
        response->message = "Command: " + request->command + " accepted.";
    }
    else
    {
        response->message = "Command: " + request->command + " not accepted.";
    }
    response->result = state_name_map.at(current_state_);
    RCLCPP_WARN_STREAM(get_logger(), response->message);
}

void OdriRos2Interface::publishRobotState()
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
}


bool OdriRos2Interface::smDisable()
{
    createOdriRobot(params_.robot_yaml_path);
    current_state_ = StateType::Idle;
    return true;
}

bool OdriRos2Interface::smCalibrate()
{
    if (current_state_ == StateType::Idle || current_state_ == StateType::Enabled)
    {
        const Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
        odri_robot_->GetJoints()->SetPositionOffsets(zero_vec);

        RCLCPP_INFO_STREAM(get_logger(), "\nCalibration procedure: Finding indexes...");
        odri_robot_->RunCalibration(zero_vec);

        RCLCPP_INFO_STREAM(get_logger(),
                           "\nCalibration procedure: Put all joints in their respective zero positions. Call state "
                           "transition service with 'enable'");

        current_state_ = StateType::Calibrating;
        return true;
    }
    else
    {
        return false;
    }
}

bool OdriRos2Interface::smEnable()
{
    if (current_state_ == StateType::Calibrating)
    {
        RCLCPP_INFO_STREAM(get_logger(), "\nThese are the offsets");
        Eigen::VectorXd current_pos = odri_robot_->GetJoints()->GetPositions();

        for (long int i = 0; i < current_pos.size(); i++)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << current_pos(i));
        }

        odri_robot_->GetJoints()->SetPositionOffsets(-current_pos);

        RCLCPP_INFO_STREAM(get_logger(), "Calibration done");

        current_state_ = StateType::Enabled;
        return true;
    }
    else if (current_state_ == StateType::Idle)
    {
        odri_robot_->RunCalibration(params_.parking_position);
        current_state_ = StateType::Enabled;
        return true;
    }
    else
    {
        return false;
    }
}

bool OdriRos2Interface::smStart()
{
    if (current_state_ == StateType::Enabled)
    {
        current_state_ = StateType::Running;
        return true;
    }
    else
    {
        return false;
    }
}

bool OdriRos2Interface::smStop()
{
    if (current_state_ == StateType::Running)
    {
        odri_robot_->GetJoints()->RunSafetyController();
        current_state_ = StateType::Enabled;
        return true;
    }
    else if (current_state_ == StateType::Calibrating)
    {
        return smDisable();
    }
    else
    {
        return false;
    }
}

}  // namespace odri_ros2_interface