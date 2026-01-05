#include "Nodes/ShoulderCore.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Shoulder::ShoulderCore>(5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}