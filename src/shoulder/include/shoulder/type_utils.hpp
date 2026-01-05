#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <random>
#include <fstream>


namespace DataType{
    using Pose = Sophus::SE3d;
    using Rotation = Sophus::SO3d;
    using Twist = Sophus::SE3d::Tangent;

    // 将末端坐标系下的速度旋量转换为本体坐标系下的速度旋量
    inline Twist End2Body(const Twist& xi){
        Twist xiBody = Twist::Zero();
        xiBody.block(0, 0, 3, 1) = xi.block(3, 0, 3, 1);  // 方便动力学计算
        return xiBody;
    }

    // 随机生成在范围内的位姿矩阵
    inline Pose RandomPose(){
        std::uniform_real_distribution<double> dis(-1, 1);
        Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
        Rotation R(q);
        Eigen::Vector3d t0(0, 0, -0.3);
        Eigen::Vector3d t = t0 + R * t0;
        return Pose(q, t);
    }

    // 将肌肉力输出到csv文件，每行包含时间和力矩分量，力分量
    inline void MuscleForce2CSV(const std::string& filename, 
        const std::vector<double>& time, 
        const std::vector<Twist>& muscleForces,
        const std::vector<Pose>& trajectory){
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
            return;
        }

        file << "Time, End_X, End_Y, End_Z, Torque_X, Torque_Y, Torque_Z, Force_X, Force_Y, Force_Z\n";
        int n = muscleForces.size();
        for (int i = 0; i < n; ++i) {
            file << time[i] << ", "
                 << trajectory[i].translation()(0) << ", "
                 << trajectory[i].translation()(1) << ", "
                 << trajectory[i].translation()(2) << ", "
                 << muscleForces[i](0) << ", "
                 << muscleForces[i](1) << ", "
                 << muscleForces[i](2) << ", "
                 << muscleForces[i](3) << ", "
                 << muscleForces[i](4) << ", "
                 << muscleForces[i](5) << "\n";
        }
        file.close();
    }
}