#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <type_utils.hpp>
#include <cmath>
#include <cassert>

namespace Motion{
    using Pose = Sophus::SE3d;
    using Rotation = Sophus::SO3d;
    using Twist = Sophus::Vector6d;

    class TrajectoryGenerator{ 
    public:
        // 构造函数
        TrajectoryGenerator(const Pose& startPose, const Pose& targetPose, const double totalTime){
            startPose_ = startPose;
            targetPose_ = targetPose;
            totalTime_ = totalTime;
            trajectory_.push_back(startPose);
            time_.push_back(0.0);
            velocity_.push_back(Twist::Zero());
            acceleration_.push_back(Twist::Zero());
        }

        // 更新轨迹
        Pose Update (double currentTime);

        // 获取总时间
        double GetTotalTime() const { return totalTime_; }

        // 获取当前位姿
        const Pose& GetCurrentPose() const { return trajectory_.back(); }

        // 获取当前速度
        const Twist& GetCurrentVelocity() const { return velocity_.back(); }

        // 获取当前加速度
        const Twist& GetCurrentAcceleration() const { return acceleration_.back(); }

        // 获取轨迹
        const std::vector<Pose>& GetTrajectory() const { return trajectory_; }

        // 获取时间
        const std::vector<double>& GetTime() const { return time_; }

        // 获取速度
        const std::vector<Twist>& GetVelocity() const { return velocity_; }

        // 获取加速度
        const std::vector<Twist>& GetAcceleration() const { return acceleration_; }

    private:
        // 计算归一化参数s
        inline double GetS(double t) const{
            return 10 * (std::pow(t, 3) / std::pow(totalTime_, 3)) 
            - 15 * (std::pow(t, 4) / std::pow(totalTime_, 4)) 
            + 6 * (std::pow(t, 5) / std::pow(totalTime_, 5));
        }

        // 计算归一化参数s的一阶导
        inline double GetS_dot(double t) const{
            return 30 * (std::pow(t, 2) / std::pow(totalTime_, 3)) 
            - 60 * (std::pow(t, 3) / std::pow(totalTime_, 4)) 
            + 30 * (std::pow(t, 4) / std::pow(totalTime_, 5));
        }

        // 计算归一化参数s的二阶导
        inline double GetS_dot_double(double t) const{
            return 60 * (std::pow(t, 1) / std::pow(totalTime_, 3)) 
            - 180 * (std::pow(t, 2) / std::pow(totalTime_, 4)) 
            + 120 * (std::pow(t, 3) / std::pow(totalTime_, 5));
        }

        Pose startPose_;  // 初始位姿
        Pose targetPose_;  // 目标位姿
        double totalTime_;  // 总时间
        std::vector<Pose> trajectory_;  // 轨迹记录
        std::vector<double> time_;  // 时间记录
        std::vector<Twist> velocity_;  // 速度记录
        std::vector<Twist> acceleration_;  // 加速度记录
    };
}

