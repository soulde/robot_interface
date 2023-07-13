//
// Created by soulde on 2023/6/14.
//

#include <robot_interface/RobotInterface.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotInterface>();


    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}