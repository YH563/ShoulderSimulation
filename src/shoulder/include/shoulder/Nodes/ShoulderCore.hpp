#pragma once

#include "YAlgorithm/YDynamics.hpp"
#include "YAlgorithm/YMotion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include "tf2_ros/transform_broadcaster.hpp"
#include "type_utils.hpp"

namespace Shoulder{
    using namespace std::chrono_literals;
    using namespace Dynamics;
    using namespace Motion;
    using Pose = Sophus::SE3d;

    Pose startPose = Pose::exp(Sophus::Vector6d(0, 0, -0.3, 0, 0, 0));
    Pose targetPose = DataType::RandomPose();
    double mass = 1.0;
    double length = 0.3;
    double r = 0.025;

    class ShoulderCore : public rclcpp::Node
    {
    public:
        ShoulderCore(int totalTime) : Node("shoulder_core")
        {
            initialize(totalTime);
        }


    private:
        // 初始化函数
        void initialize(double totalTime);

        // 回调函数
        void update();

        // 发布坐标
        void publishTF(const Pose& pose);

        // ROS组件
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Time startTime_;

        // 算法组件
        std::unique_ptr<TrajectoryGenerator> trajectoryGenerator_;
        std::unique_ptr<DynamicsCalculator> dynamicsCalculator_;
    };
}