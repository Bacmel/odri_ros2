#include "odri_ros2_interface/odri_ros2_interface.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<odri_ros2_interface::OdriRos2Interface> odri_ros2_interface =
        std::make_shared<odri_ros2_interface::OdriRos2Interface>("OdriRos2Interface");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(odri_ros2_interface);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}