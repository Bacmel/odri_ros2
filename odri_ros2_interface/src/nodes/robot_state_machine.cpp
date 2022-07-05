#include "odri_ros2_interface/robot_state_machine.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_ros2_interface::RobotStateMachine> robot_state_machine =
        std::make_shared<odri_ros2_interface::RobotStateMachine>("RobotStateMachine");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(robot_state_machine);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}