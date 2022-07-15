#include "odri_examples/stretch_robot.hpp"

#include <cmath>

StretchRobot::StretchRobot(const std::string &node_name) : Node(node_name)
{
    sub_robot_state_ = create_subscription<odri_ros2_msgs::msg::RobotState>(
        "robot_state", rclcpp::QoS(1), std::bind(&StretchRobot::callbackRobotState, this, std::placeholders::_1));

    pub_robot_command_ = create_publisher<odri_ros2_msgs::msg::RobotCommand>("robot_command", 1);

    timer_change_command_  = create_wall_timer(std::chrono::duration<double>(1),
                                               std::bind(&StretchRobot::callbackTimerChangeCommand, this));
    timer_publish_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                               std::bind(&StretchRobot::callbackTimerPublishCommand, this));

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&StretchRobot::callbackParameters, this, std::placeholders::_1));

    client_odri_interface_ = create_client<odri_ros2_msgs::srv::TransitionCommand>("/robot_state_machine/state_transition");

    got_initial_position_     = false;
    counter_initial_position_ = 0;

    stretch_params_.nb        = 1.0;
    stretch_params_.freq      = 1.0;
    stretch_params_.t         = 0;
    stretch_params_.dt        = 0.001;

    declare_parameter<bool>("publish_commands", false);

    brought_to_init_ = false;
}

StretchRobot::~StretchRobot() {}

void StretchRobot::callbackTimerChangeCommand() {}

void StretchRobot::callbackTimerPublishCommand()
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
                if(i==0) {
                    double omega = 2*M_PI*stretch_params_.freq;
                    double cst = M_PI*stretch_params_.nb;
                    double A = cst*omega;
                    command.position_ref = - (A/omega)*cos(omega*stretch_params_.t)+cst;
                    command.velocity_ref = A*sin(omega*stretch_params_.t);
                    command.kp = 5.;
                } else {
                    command.position_ref = 0;
                    command.velocity_ref = 0;
                    command.kp           = 1;
                }
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
        stretch_params_.t += stretch_params_.dt;
        pub_robot_command_->publish(msg_robot_command_);
    }
}

void StretchRobot::callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg)
{
    pos_error_(0) = msg->motor_states[0].position;
    pos_error_(1) = msg->motor_states[1].position;
}

rcl_interfaces::msg::SetParametersResult StretchRobot::callbackParameters(
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
                request->command = "start";
                auto response_received_callback =
                    [this](rclcpp::Client<odri_ros2_msgs::srv::TransitionCommand>::SharedFuture future) {
                        auto result              = future.get();
                        params_.publish_commands = result->result == "RUNNING";
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
    std::shared_ptr<StretchRobot> stretch_driver = std::make_shared<StretchRobot>("stretch_driver");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stretch_driver);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}