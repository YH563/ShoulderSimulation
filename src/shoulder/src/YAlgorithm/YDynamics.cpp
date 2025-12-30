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
}