#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <type_utils.hpp>
#include <cmath>
#include "YMotion.hpp"
#include "type_utils.hpp"

namespace Dynamics{
    using Twist = Sophus::Vector6d;
    using MassMatrix = Eigen::Matrix<double, 6, 6>;
    using adMatrix = Eigen::Matrix<double, 6, 6>;
    using AdMatrix = Eigen::Matrix<double, 6, 6>;
    using Pose = Sophus::SE3d;
    using Rotation = Sophus::SO3d;
    #define g 9.8

    class DynamicsCalculator{
    public:
        // 构造函数
        DynamicsCalculator(double length, double r, double mass){
            length_ = length;
            r_ = r;
            mass_ = mass;
            CalMass();
            Twist gravityInitial = Twist::Zero();
            gravityInitial(5) = - mass * g;
            gravity_.push_back(gravityInitial);
            force_.push_back(Twist::Zero());
            muscleForce_.push_back(-gravityInitial);
        }
        
        // 更新力
        void Update(const Motion::TrajectoryGenerator &generator);

        // 获取合力
        const std::vector<Twist> &GetForce() const{ return force_; }
        
        // 获取肌肉力
        const std::vector<Twist> &GetMuscleForce() const{ return muscleForce_; }

    private:
        // 计算广义惯性矩阵
        inline void CalMass();

        // 计算惯性系下的重力旋量
        inline Twist CalGravity(const Motion::TrajectoryGenerator &generator);

        double length_;  // 大臂长度
        double r_;  // 大臂半径
        double mass_;  // 大臂质量
        MassMatrix massMatrix_;  // 广义惯性矩阵
        std::vector<Twist> gravity_;  // 重力旋量
        std::vector<Twist> force_;  // 合力旋量
        std::vector<Twist> muscleForce_;  // 肌肉力旋量
    };
}