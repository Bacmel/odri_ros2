#include "odri_examples/example_robot.hpp"

#include <cmath>

ExampleRobot::ExampleRobot(const std::string &node_name) : Node(node_name)
{
    sub_robot_state_ = create_subscription<odri_ros2_msgs::msg::RobotState>(
        "robot_state", rclcpp::QoS(1), std::bind(&ExampleRobot::callbackRobotState, this, std::placeholders::_1));

    pub_robot_command_ = create_publisher<odri_ros2_msgs::msg::RobotCommand>("robot_command", 1);

    timer_change_command_  = create_wall_timer(std::chrono::duration<double>(1),
                                               std::bind(&ExampleRobot::callbackTimerChangeCommand, this));
    timer_publish_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                               std::bind(&ExampleRobot::callbackTimerPublishCommand, this));

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ExampleRobot::callbackParameters, this, std::placeholders::_1));

    client_odri_interface_ = create_client<odri_ros2_msgs::srv::TransitionCommand>("/robot_interface/state_transition");

    got_initial_position_     = false;
    counter_initial_position_ = 0;

    wave_params_.amplitude = M_PI / 3;  // reduction for security
    wave_params_.freq      = 0.05;
    wave_params_.t         = 0;
    wave_params_.dt        = 0.001;

    declare_parameter<bool>("publish_commands", false);

    brought_to_init_ = false;
}

ExampleRobot::~ExampleRobot() {}

void ExampleRobot::callbackTimerChangeCommand() {}

void ExampleRobot::callbackTimerPublishCommand()
{
    msg_robot_command_.header.stamp = get_clock()->now();
    msg_robot_command_.motor_commands.clear();

    if (params_.publish_commands)
    {
        for (std::size_t i = 0; i < 2; ++i)
        {
            odri_ros2_msgs::msg::MotorCommand command;

            if (brought_to_init_)
            {
                command.position_ref = wave_params_.amplitude * sin(2 * M_PI * wave_params_.freq * wave_params_.t);
                command.velocity_ref = 2. * M_PI * wave_params_.freq * wave_params_.amplitude *
                                       cos(2 * M_PI * wave_params_.freq * wave_params_.t);
                command.kp = 5.;
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
        wave_params_.t += wave_params_.dt;
        pub_robot_command_->publish(msg_robot_command_);
    }
}

void ExampleRobot::callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg)
{
    pos_error_(0) = msg->motor_states[0].position;
    pos_error_(1) = msg->motor_states[1].position;
}

rcl_interfaces::msg::SetParametersResult ExampleRobot::callbackParameters(
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

                auto request     = std::make_shared<odri_ros2_msgs::srv::TransitionCommand::Request>();
                request->command = "enable";
                auto response_received_callback =
                    [this](rclcpp::Client<odri_ros2_msgs::srv::TransitionCommand>::SharedFuture future) {
                        auto result              = future.get();
                        params_.publish_commands = result->result == "enabled";
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
    std::shared_ptr<ExampleRobot> example_driver = std::make_shared<ExampleRobot>("example_driver");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(example_driver);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}