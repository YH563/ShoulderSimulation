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
        inline void CalMass(){
            massMatrix_ = MassMatrix::Zero();
            massMatrix_(0, 0) = mass_ * (3 * r_ * r_ + 4 * length_ * length_) / 12;
            massMatrix_(1, 1) = mass_ * (3 * r_ * r_ + 4 * length_ * length_) / 12;
            massMatrix_(2, 2) = mass_ * r_ * r_ / 2;
            for(int i = 3; i < 6; i++)
                massMatrix_(i, i) = mass_;
            massMatrix_(0, 4) = mass_ * length_ / 2;
            massMatrix_(1, 3) = -mass_ * length_ / 2;
            massMatrix_(3, 1) = -mass_ * length_ / 2;
            massMatrix_(4, 0) = mass_ * length_ / 2;
        }

        // 计算惯性系下的重力旋量
        inline Twist CalGravity(const Motion::TrajectoryGenerator &generator){
            Eigen::Vector3d r_c (0, 0, - length_ / 2);
            Eigen::Vector3d mg (0, 0, - mass_ * g);
            Twist gravity = Twist::Zero();
            Rotation R = generator.GetCurrentPose().rotationMatrix();
            gravity.head<3>() = (R * r_c).cross(mg);
            gravity.tail<3>() = mg;
            return gravity;
        }

        double length_;  // 大臂长度
        double r_;  // 大臂半径
        double mass_;  // 大臂质量
        MassMatrix massMatrix_;  // 广义惯性矩阵
        std::vector<Twist> gravity_;  // 重力旋量
        std::vector<Twist> force_;  // 合力旋量
        std::vector<Twist> muscleForce_;  // 肌肉力旋量
    };
}