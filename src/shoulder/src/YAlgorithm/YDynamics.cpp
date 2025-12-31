#include "YAlgorithm/YDynamics.hpp"

namespace Dynamics{

    void DynamicsCalculator::Update(const Motion::TrajectoryGenerator &generator){
        Pose pose = generator.GetCurrentPose();
        Twist velocity = DataType::End2Body(generator.GetCurrentVelocity());
        Twist acceleration = DataType::End2Body(generator.GetCurrentAcceleration());

        // 带入动力学公式计算
        Eigen::Vector3d omega = velocity.head(3);
        adMatrix ad = adMatrix::Zero();
        ad.block(0 ,0, 3, 3) = Rotation::hat(omega);
        ad.block(3 ,3, 3, 3) = Rotation::hat(omega);
        Twist forceBody = mass_ * acceleration - ad.transpose() * mass_ * velocity;
        
        // 计算惯性系下的合力
        AdMatrix Ad = AdMatrix::Zero();
        Ad.block(0 ,0, 3, 3) = pose.rotationMatrix();
        Ad.block(3 ,3, 3, 3) = pose.rotationMatrix();
        Twist force = Ad * forceBody;
        force_.push_back(force);

        // 计算惯性系下的重力
        Twist grivity = CalGravity(generator);
        gravity_.push_back(grivity);

        // 计算惯性系下的肌肉力
        Twist muscleForce = force - grivity;
        muscleForce_.push_back(muscleForce);
    }

    inline void DynamicsCalculator::CalMass(){
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

    inline Twist DynamicsCalculator::CalGravity(const Motion::TrajectoryGenerator &generator){
        Eigen::Vector3d r_c (0, 0, - length_ / 2);
        Eigen::Vector3d mg (0, 0, - mass_ * g);
        Twist gravity = Twist::Zero();
        Rotation R = generator.GetCurrentPose().rotationMatrix();
        gravity.head<3>() = (R * r_c).cross(mg);
        gravity.tail<3>() = mg;
        return gravity;
    }
}