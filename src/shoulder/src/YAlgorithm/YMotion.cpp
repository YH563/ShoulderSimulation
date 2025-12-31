#include "YAlgorithm/YMotion.hpp"

namespace Motion{

    Pose TrajectoryGenerator::Update(double currentTime){
        assert(currentTime > 0 && "Current time is not in the range of the trajectory!");    
        // 更新时间
        time_.push_back(currentTime);

        // 更新位姿
        double s = GetS(currentTime);
        Pose relativeTransform = startPose_.inverse() * targetPose_;
        Sophus::SE3d::Tangent tangent = relativeTransform.log();
        Pose currentPose = startPose_ * Sophus::SE3d::exp(tangent * s);
        trajectory_.push_back(currentPose);

        // 更新速度
        double ds = GetS_dot(currentTime);
        velocity_.push_back(tangent * ds);

        // 更新加速度
        double dds = GetS_dot_double(currentTime);
        acceleration_.push_back(tangent * dds);

        return trajectory_.back();
    }
}