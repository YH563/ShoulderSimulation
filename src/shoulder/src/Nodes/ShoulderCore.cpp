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
        
    }
}