#pragma once

#include "YAlgorithm/YDynamics.hpp"
#include "YAlgorithm/YMotion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include "tf2_ros/transform_broadcaster.hpp"
#include "type_utils.hpp"
#include "wrench_interfaces/srv/wrench_data.hpp"


namespace Shoulder{
    using namespace std::chrono_literals;
    using namespace Dynamics;
    using namespace Motion;

    class ShoulderCore : public rclcpp::Node
    {
    public:
        ShoulderCore(int totalTime) : Node("shoulder_core")
        {
            initialize(totalTime);
            RCLCPP_INFO(this->get_logger(), "ShoulderCore node has been started.");
        }

    private:
        // 初始化函数
        void initialize(double totalTime);

        // 回调函数
        void update();

        // 发布坐标
        void publishTF(const Pose& pose);

        // 发送请求
        void sendRequest(const std::vector<double>& time, 
            const std::vector<Twist>& muscleForces,
            const std::vector<Pose>& trajectory);

        // ROS组件
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Time startTime_;
        rclcpp::Client<wrench_interfaces::srv::WrenchData>::SharedPtr client_;

        // 算法组件
        std::unique_ptr<TrajectoryGenerator> trajectoryGenerator_;
        std::unique_ptr<DynamicsCalculator> dynamicsCalculator_;
    };
}