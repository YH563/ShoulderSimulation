#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>


namespace DataType{
    using Pose = Sophus::SE3d;
    using Rotation = Sophus::SO3d;
    using Twist = Sophus::SE3d::Tangent;

    // 将末端坐标系下的速度旋量转换为本体坐标系下的速度旋量
    inline Twist End2Body(const Twist& xi){
        Twist xiBody = Twist::Zero();
        xiBody.block(0, 0, 3, 1) = xi.block(3, 0, 3, 1);
        return xiBody;
    }
}