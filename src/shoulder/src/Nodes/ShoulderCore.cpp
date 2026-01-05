#include "Nodes/ShoulderCore.hpp"

namespace Shoulder{

    static Pose startPose = Pose::exp(Sophus::Vector6d(0, 0, -0.3, 0, 0, 0));
    static Pose targetPose = DataType::RandomPose();
    static double mass = 1.0;
    static double length = 0.3;
    static double r = 0.025;
    
    void ShoulderCore::initialize(double totalTime){
        // 算法组件
        trajectoryGenerator_ = std::make_unique<TrajectoryGenerator>(startPose, targetPose, totalTime);
        dynamicsCalculator_ = std::make_unique<DynamicsCalculator>(length, r, mass);

         // ROS组件
        client_ = this->create_client<wrench_interfaces::srv::WrenchData>("wrench_data_service");
        // 检查服务是否可用
        RCLCPP_INFO(this->get_logger(), "等待 wrench_data_service 服务...");
        
        // 使用异步方式等待服务，避免阻塞
        if (!client_->wait_for_service(5s)) {
            RCLCPP_WARN(this->get_logger(), "服务 wrench_data_service 未就绪，将继续尝试");
        } else {
            RCLCPP_INFO(this->get_logger(), "服务 wrench_data_service 已连接");
        }
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        startTime_ = this->now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ShoulderCore::update, this)
        );
    }

    void ShoulderCore::update(){
        // 更新轨迹
        double totalTime = trajectoryGenerator_->GetTotalTime();
        double elapsed_time = (this->now() - startTime_).seconds();
        auto currentPose = trajectoryGenerator_->Update(elapsed_time);
        
        // 更新动力学
        dynamicsCalculator_->Update(*trajectoryGenerator_);

        publishTF(currentPose);
        if (elapsed_time >= totalTime){
            RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
            timer_->cancel();

            // 保存文件并发送请求
            sendRequest(
                trajectoryGenerator_->GetTime(),
                dynamicsCalculator_->GetMuscleForce(),
                trajectoryGenerator_->GetTrajectory()
            );
        }
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

        // 打印位姿
        RCLCPP_INFO(
            this->get_logger(), 
            "Current Pose: \n"
            "Rotation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f",
            q.x(), q.y(), q.z(), q.w()
        );
    }

    void ShoulderCore::sendRequest(
        const std::vector<double>& time, 
        const std::vector<Twist>& muscleForces,
        const std::vector<Pose>& trajectory){
        
        // 将动力学计算结果输出到csv文件
        std::string logDir = rclcpp::get_logging_directory().string();
        std::string csvPath = logDir + "/muscle_forces.csv";
        DataType::MuscleForce2CSV(csvPath, time, muscleForces, trajectory);
        
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "服务不可用，无法发送请求");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "csv文件保存路径为：%s", csvPath.c_str());

        // 创建并发送请求
        auto request = std::make_shared<wrench_interfaces::srv::WrenchData::Request>();
        request->csv_file_path = csvPath;
        
        RCLCPP_INFO(this->get_logger(), "发送请求到 wrench_data_service...");
        
        // 使用带回调的异步发送
        auto callback = [this](rclcpp::Client<wrench_interfaces::srv::WrenchData>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "收到反馈：%s\n",
                        response->message.c_str()
                    );
                } else {
                    RCLCPP_WARN(this->get_logger(), "请求处理失败: %s", response->message.c_str());
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "服务调用失败: %s", e.what());
            }
        };
        
        // 发送请求并设置回调
        auto future = client_->async_send_request(request, callback);
        RCLCPP_INFO(this->get_logger(), "请求已发送，等待异步响应...");
    }
}