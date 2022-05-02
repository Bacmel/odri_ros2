#include "odri_interface/robot_interface.hpp"

namespace odri_interface
{
RobotInterface::RobotInterface(const std::string& node_name) : rclcpp::Node(node_name)
{
    declareParameters();
    odri_robot_ = odri_control_interface::RobotFromYamlFile(params_.robot_yaml_path);
    odri_robot_->Start();
    odri_robot_->WaitUntilReady();

    service_sm_transition_ = create_service<odri_msgs::srv::TransitionCommand>(
        std::string(std::string(get_name()) + "/state_transition").c_str(),
        std::bind(&RobotInterface::transitionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    pub_robot_state_     = create_publisher<odri_msgs::msg::RobotState>("robot_state", 1);
    subs_motor_commands_ = create_subscription<odri_msgs::msg::RobotCommand>(
        "robot_command", rclcpp::QoS(1),
        std::bind(&RobotInterface::callbackRobotCommand, this, std::placeholders::_1));

    timer_send_commands_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                             std::bind(&RobotInterface::callbackTimerSendCommands, this));

    positions_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    velocities_ = positions_;

    des_torques_    = positions_;
    des_positions_  = positions_;
    des_velocities_ = positions_;
    des_pos_gains_  = positions_;
    des_vel_gains_  = positions_;
    max_currents_   = positions_;

    sm_active_state_ = SmStates::Idle;
}

RobotInterface::~RobotInterface() {}

void RobotInterface::declareParameters()
{
    declare_parameter<std::string>("robot_yaml_path", "");
    declare_parameter<std::string>("adapter_name", "eno1");
    declare_parameter<int>("n_slaves", 1);

    get_parameter<std::string>("robot_yaml_path", params_.robot_yaml_path);
    get_parameter<std::string>("adapter_name", params_.adapter_name);
    int n_slaves;
    get_parameter<int>("n_slaves", n_slaves);
    params_.n_slaves = n_slaves;
}

void RobotInterface::callbackTimerSendCommands()
{
    odri_robot_->ParseSensorData();

    positions_  = odri_robot_->joints->GetPositions();
    velocities_ = odri_robot_->joints->GetVelocities();

    robot_state_msg_.header.stamp = get_clock()->now();
    robot_state_msg_.motor_states.clear();

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        odri_msgs::msg::MotorState m_state;

        m_state.position = positions_(i);
        m_state.velocity = velocities_(i);

        robot_state_msg_.motor_states.push_back(m_state);
    }
    pub_robot_state_->publish(robot_state_msg_);

    switch (sm_active_state_) {
        case SmStates::Enabled:
            odri_robot_->joints->SetTorques(des_torques_);  // WARNING: mixing current and torques. Change the msg
            odri_robot_->joints->SetDesiredPositions(des_positions_);
            odri_robot_->joints->SetDesiredVelocities(des_velocities_);
            odri_robot_->joints->SetPositionGains(des_pos_gains_);
            odri_robot_->joints->SetVelocityGains(des_vel_gains_);
            odri_robot_->joints->SetMaximumCurrents(
                max_currents_(0));  // WARNING: Max current is common for all joints
            break;
        case SmStates::Idle:
        case SmStates::Calibrating:
            Eigen::VectorXd vec_zero = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
            odri_robot_->joints->SetTorques(vec_zero);
            odri_robot_->joints->SetDesiredPositions(vec_zero);
            odri_robot_->joints->SetDesiredVelocities(vec_zero);
            odri_robot_->joints->SetPositionGains(vec_zero);
            odri_robot_->joints->SetVelocityGains(vec_zero);
            break;
    }

    if(odri_robot_->SendCommandAndWaitEndOfCycle(0.001) == false) // Test for security
    {
        sm_active_state_ = SmStates::Idle;
        Eigen::VectorXd vec_zero = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
        odri_robot_->joints->SetTorques(vec_zero);
        odri_robot_->joints->SetDesiredPositions(vec_zero);
        odri_robot_->joints->SetDesiredVelocities(vec_zero);
        odri_robot_->joints->SetPositionGains(vec_zero);
        odri_robot_->joints->SetVelocityGains(vec_zero);
    }
}

void RobotInterface::callbackRobotCommand(const odri_msgs::msg::RobotCommand::SharedPtr msg)
{
    for (std::size_t i = 0; i < msg->motor_commands.size(); ++i) {
        des_torques_(i)    = msg->motor_commands[i].current_ref;
        des_positions_(i)  = msg->motor_commands[i].position_ref;
        des_velocities_(i) = msg->motor_commands[i].velocity_ref;
        des_pos_gains_(i)  = msg->motor_commands[i].kp;
        des_vel_gains_(i)  = msg->motor_commands[i].kd;
        max_currents_(i)   = msg->motor_commands[i].i_sat;
    }
}

void RobotInterface::transitionRequest(const std::shared_ptr<odri_msgs::srv::TransitionCommand::Request>  request,
                                       const std::shared_ptr<odri_msgs::srv::TransitionCommand::Response> response)
{
    RCLCPP_INFO_STREAM(get_logger(), "Service request received");

    SmTransitions command = SmTransitions::NbTransitions;
    if (sm_transitions_map.find(request->command) != sm_transitions_map.end()) {
        command = sm_transitions_map.at(request->command);
    }

    switch (command) {
        case SmTransitions::Enable:
            response->accepted = smEnable(response->message);
            if (response->accepted) {
                sm_active_state_ = SmStates::Enabled;
            }
            break;

        case SmTransitions::Disable:
            response->accepted = smDisable(response->message);
            if (response->accepted) {
                sm_active_state_ = SmStates::Idle;
            }
            break;

        case SmTransitions::Calibrate:
            if (sm_active_state_ == SmStates::Idle) {
                response->accepted = smCalibrateFromIdle(response->message);
                if (response->accepted) {
                    sm_active_state_ = SmStates::Calibrating;
                }
            } else if (sm_active_state_ == SmStates::Calibrating) {
                response->accepted = smCalibrateFromCalibrating(response->message);
                if (response->accepted) {
                    sm_active_state_ = SmStates::Idle;
                }
            } else {
                response->accepted = false;
                response->message  = "Cannot CALIBRATE the odri Interface. It is not in the IDLE/CALIBRATING state.";
            }
            break;
        default:
            response->accepted = false;
            response->message  = "Command: " + request->command +
                                " does not exist. Possible options are: 'enable'|'disable'|'calibrate'";
            RCLCPP_WARN_STREAM(get_logger(), response->message);
            break;
    }

    response->result = sm_states_map.at(sm_active_state_);
    RCLCPP_WARN_STREAM(get_logger(), response->message);
}

bool RobotInterface::smEnable(std::string& message)
{
    if (sm_active_state_ == SmStates::Idle) {
        if (odri_robot_->GetJoints()->SawAllIndices()) {
            message = "ODRI enabled";
            return true;
        } else {
            message = "Cannot enable ODRI. Run calibration first.";
            return false;
        }
    } else {
        message = "Cannot ENABLE the odri Interface. It is not in the IDLE state.";
        return false;
    }
}

bool RobotInterface::smDisable(std::string& message)
{
    if (sm_active_state_ != SmStates::Idle) {
        message = "ODRI disabled";
        return true;
    } else {
        message = "ODRI interface is already in the IDLE state.";
        return false;
    }
}

bool RobotInterface::smCalibrateFromIdle(std::string& message)
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

bool RobotInterface::smCalibrateFromCalibrating(std::string& message)
{
    RCLCPP_INFO_STREAM(get_logger(), "\nThese are the offsets");
    Eigen::VectorXd current_pos = odri_robot_->GetJoints()->GetPositions();

    for (size_t i = 0; i < current_pos.size(); i++) {
        RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << current_pos(i));
    }

    odri_robot_->GetJoints()->SetPositionOffsets(-current_pos);

    message = "Calibration done";

    return true;
}

std::map<std::string, RobotInterface::SmTransitions> RobotInterface::createSmTransitionsMap()
{
    std::map<std::string, SmTransitions> m;
    m["enable"]    = SmTransitions::Enable;
    m["disable"]   = SmTransitions::Disable;
    m["calibrate"] = SmTransitions::Calibrate;

    return m;
}

std::map<RobotInterface::SmStates, std::string> RobotInterface::createSmStatesMap()
{
    std::map<SmStates, std::string> m;
    m[SmStates::Idle]        = "idle";
    m[SmStates::Enabled]     = "enabled";
    m[SmStates::Calibrating] = "calibrating";

    return m;
}

const std::map<std::string, RobotInterface::SmTransitions> RobotInterface::sm_transitions_map =
    RobotInterface::createSmTransitionsMap();
const std::map<RobotInterface::SmStates, std::string> RobotInterface::sm_states_map =
    RobotInterface::createSmStatesMap();

}  // namespace odri_interface

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_interface::RobotInterface> master_board_iface =
        std::make_shared<odri_interface::RobotInterface>("RobotInterface");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(master_board_iface);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}