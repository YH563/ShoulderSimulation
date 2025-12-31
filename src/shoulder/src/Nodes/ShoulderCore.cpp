#include "Nodes/ShoulderCore.hpp"

namespace Shoulder{

    void ShoulderCore::initialize(double totalTime){
        trajectoryGenerator_ = std::make_unique<TrajectoryGenerator>(startPose, targetPose, totalTime);
        dynamicsCalculator_ = std::make_unique<DynamicsCalculator>(length, r, mass);

        // ROS组件
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        startTime_ = this->now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ShoulderCore::update, this)
        );
    }

    void ShoulderCore::update(){
        double elapsed_time = (this->now() - startTime_).seconds();
        auto currentPose = trajectoryGenerator_->Update(elapsed_time);
        dynamicsCalculator_->Update(*trajectoryGenerator_);
        publishTF(currentPose);
    }

    void ShoulderCore::publishTF(const Pose& pose){
        // 将末端坐标系目标位姿转为本体坐标系目标位姿
        Eigen::Quaterniond q = pose.so3().unit_quaternion();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "shoulder_joint";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0;

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }
}