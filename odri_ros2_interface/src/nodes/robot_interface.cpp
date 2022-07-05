#include "odri_ros2_interface/robot_interface.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_ros2_interface::RobotInterface> robot_interface =
        std::make_shared<odri_ros2_interface::RobotInterface>("RobotInterface");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(robot_interface);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}