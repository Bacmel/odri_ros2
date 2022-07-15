#include "odri_ros2_identification/trajectory_test_base.hpp"

#include "odri_ros2_identification/trajectory_test/wave.hpp"
#include "odri_ros2_identification/trajectory_test/stretch.hpp"
#include "odri_ros2_identification/trajectory_test/offline.hpp"

namespace odri_ros2_identification
{
TrajectoryTestAbstract::TrajectoryTestAbstract(const std::string& node_name) : Node{node_name}
{
    initializeParameters();

    pub_robot_command_ = create_publisher<odri_ros2_msgs::msg::RobotCommand>("robot_command", 1);

    sub_robot_state_ = create_subscription<odri_ros2_msgs::msg::RobotState>(
        "robot_state", rclcpp::QoS(1),
        std::bind(&TrajectoryTestAbstract::callbackRobotState, this, std::placeholders::_1));

    timer_publish_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(base_params_.periode),
                                               std::bind(&TrajectoryTestAbstract::callbackTimerPublishCommand, this));
}

TrajectoryTestAbstract::~TrajectoryTestAbstract() {}

void TrajectoryTestAbstract::initializeParameters()
{
    declare_parameter("periode", 2.5);
    get_parameter("periode", base_params_.periode);
}

bool TrajectoryTestAbstract::is_running()
{
    return base_params_.state=="running";
}

void TrajectoryTestAbstract::callbackTimerPublishCommand()
{
    if(is_running())
    {
        updatecommand();
        pub_robot_command_.publish(base_params_.command);
    }
    else
    {
        restartCommand();
    }
}
void TrajectoryTestAbstract::callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg);

}  // namespace odri_ros2_identification