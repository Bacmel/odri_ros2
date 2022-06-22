#include "odri_identification/flying_arm_identification.hpp"

#include <cmath>

FlyingArmIdentification::FlyingArmIdentification(const std::string &node_name) : Node(node_name)
{
    sub_robot_state_ = create_subscription<odri_msgs::msg::RobotState>(
        "robot_state", rclcpp::QoS(1),
        std::bind(&FlyingArmIdentification::callbackRobotState, this, std::placeholders::_1));

    pub_robot_command_ = create_publisher<odri_msgs::msg::RobotCommand>("robot_command", 1);

    timer_publish_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                               std::bind(&FlyingArmIdentification::callbackTimerPublishCommand, this));

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FlyingArmIdentification::callbackParameters, this, std::placeholders::_1));

    client_odri_interface_ = create_client<odri_msgs::srv::TransitionCommand>("/robot_interface/state_transition");

    got_initial_position_     = false;
    counter_initial_position_ = 0;

    declareParameters();
    createCommands();

    declare_parameter<bool>("publish_commands", false);
    get_parameter<bool>("publish_commands", params_.publish_commands);

    brought_to_init_ = false;
}

FlyingArmIdentification::~FlyingArmIdentification() {}

void FlyingArmIdentification::declareParameters()
{
    declare_parameter<std::string>("trajectory_csv_path", "");
    get_parameter<std::string>("robot_yaml_path", params_.trajectory_csv_path);
}

void FlyingArmIdentification::createCommands()
{
    std::vector<std::string> row;
    std::string              line, word;
    std::istringstream       value;
    SetPoint                 set_point;

    std::fstream file(params_.trajectory_csv_path, std::ios::in);
    if (file.is_open())
    {
        getline(file, line);         // remove first line of csv
        while (getline(file, line))  // TODO: Refactoring
        {
            std::stringstream str(line);
            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.t;

            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.pos[0];

            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.pos[1];

            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.vel[0];

            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.vel[1];

            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.acc[0];

            getline(str, word, ',');
            value = std::istringstream(word);
            value >> set_point.acc[1];

            command_params_.commands.push_back(set_point);
        }
    }
}

void FlyingArmIdentification::callbackTimerPublishCommand()
{
    msg_robot_command_.header.stamp = get_clock()->now();
    msg_robot_command_.motor_commands.clear();

    if (params_.publish_commands)
    {
        for (std::size_t i = 0; i < 2; ++i)
        {
            odri_msgs::msg::MotorCommand command;

            if (brought_to_init_)
            {
                command.position_ref = command_params_.commands.at(command_params_.curr_idx).pos[i];
                command.velocity_ref = command_params_.commands.at(command_params_.curr_idx).vel[i];
                command.kp           = 5.;
            }
            else
            {
                command.position_ref = 0;
                command.velocity_ref = 0;
                command.kp           = 1;
                brought_to_init_     = pos_error_.norm() < 2e-1;
                std::cout << "Error Norm: " << pos_error_.norm() << std::endl;
            }

            command.kd    = 0.1;
            command.i_sat = 4.;

            msg_robot_command_.motor_commands.push_back(command);
        }
        command_params_.curr_idx++;
        pub_robot_command_->publish(msg_robot_command_);
    }
}

void FlyingArmIdentification::callbackRobotState(const odri_msgs::msg::RobotState::SharedPtr msg)
{
    pos_error_(0) = msg->motor_states[0].position;
    pos_error_(1) = msg->motor_states[1].position;
}

rcl_interfaces::msg::SetParametersResult FlyingArmIdentification::callbackParameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason     = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "publish_commands")
        {
            params_.publish_commands = param.as_bool();
            if (params_.publish_commands)
            {
                while (!client_odri_interface_->wait_for_service(std::chrono::seconds(1)))
                {
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }

                auto request     = std::make_shared<odri_msgs::srv::TransitionCommand::Request>();
                request->command = "enable";
                auto response_received_callback =
                    [this](rclcpp::Client<odri_msgs::srv::TransitionCommand>::SharedFuture future) {
                        auto result              = future.get();
                        params_.publish_commands = result->result == "enabled";
                        command_params_.curr_idx = result->result == "enabled" ? 0 : command_params_.curr_idx;
                        RCLCPP_INFO_STREAM(get_logger(), "Result: " << result->result);
                    };
                auto res_client = client_odri_interface_->async_send_request(request, response_received_callback);
            }
        }
    }
    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<FlyingArmIdentification> flying_arm = std::make_shared<FlyingArmIdentification>("flying_arm");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(flying_arm);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}