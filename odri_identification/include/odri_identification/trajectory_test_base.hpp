#pragma once

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "odri_ros2_msgs/srv/transition_command.hpp"
#include "odri_ros2_msgs/msg/robot_state.hpp"
#include "odri_ros2_msgs/msg/robot_command.hpp"

namespace odri_ros2_identification
{

enum class TrajectoryTestTypes
{
    Wave,
    Stretch,
    Offline,
    NbTrajectoryTestTypes
};

static std::map<std::string, TrajectoryTestTypes> TrajectoryTestTypes_init_map()
{
    std::map<std::string, TrajectoryTestTypes> m;
    m.clear();
    m.insert({"Wave", TrajectoryTestTypes::Wave});
    m.insert({"Stretch", TrajectoryTestTypes::Stretch});
    m.insert({"Offline", TrajectoryTestTypes::Offline});
    return m;
}

static const std::map<std::string, TrajectoryTestTypes> TrajectoryTestTypes_map = TrajectoryTestTypes_init_map();

class TrajectoryTestAbstract : public rclcpp::Node
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryTestAbstract(const std::string& node_name);
    virtual ~TrajectoryTestAbstract();

  private:
    virtual void initializeParameters();
    virtual void createCommands();
    virtual void updateCommand();
    virtual void restartCommand();

    bool is_running();
    void callbackTimerPublishCommand();
    void callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg);

    rclcpp::Publisher<odri_ros2_msgs::msg::RobotCommand>::SharedPtr    pub_robot_command_;
    rclcpp::Subscriber<odri_ros2_msgs::msg::RobotState>::SharedPtr sub_robot_state_;
    rclcpp::TimerBase::SharedPtr                                       timer_publish_command_;

    struct TrajectoryTestAbstractParams
    {
        odri_ros2_msgs::msg::RobotCommand command;

        std::string state{"idle"};
        double periode{2.5};
    } base_params_;
};

class TrajectoryTestFactory
{
  public:
    ~TrajectoryTestFactory();

    static std::shared_ptr<TrajectoryTestAbstract> create(const std::string&         node_name,
                                                          const TrajectoryTestTypes& test_type);

  private:
    TrajectoryTestFactory();
};

}  // namespace odri_ros2_identification