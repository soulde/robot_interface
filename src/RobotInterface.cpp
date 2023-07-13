//
// Created by soulde on 2023/6/14.
//

#include "robot_interface/RobotInterface.h"

RobotInterface::RobotInterface() : Node("robot") {
    this->declare_parameter("dev_name", "STM32");
    this->get_parameter<std::string>("dev_name", dev_name);
    serial = std::make_shared<Serial>(dev_name.c_str());

    twistSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 5,
                                                                           std::bind(&RobotInterface::twistCallback,
                                                                                     this,
                                                                                     std::placeholders::_1));

    GNSSSubscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("fix", 5,
                                                                            std::bind(&RobotInterface::GNSSCallback,
                                                                                      this,
                                                                                      std::placeholders::_1));

    goalPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 1);
    goal.header.frame_id = fixFrame;

    UWBPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/UWB", 1);
    uwb.header.frame_id = fixFrame;

    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
    odom.child_frame_id = robotFrame;
    odom.header.frame_id = fixFrame;

    transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    staticTransformBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);


    thread = std::make_unique<std::thread>([this]() { recvLoop(); });
}

void RobotInterface::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame.header), sizeof(Header));

    moveControlFrame.x = static_cast<float>(msg->linear.x);
    moveControlFrame.y = static_cast<float>(msg->linear.y);
    moveControlFrame.yaw = static_cast<float>(msg->angular.z);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));

    for (int i = 0; i < sizeof(MoveControlFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&moveControlFrame) + i));
    }
    std::cout << std::endl;
}

void RobotInterface::GNSSCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&GNSSFrame.header), sizeof(Header));
    std::cout << "GNSS Data Send" << std::endl;
    GNSSFrame.lon = static_cast<float>(msg->longitude);
    GNSSFrame.lat = static_cast<float>(msg->latitude);
    GNSSFrame.alt = static_cast<float>(msg->altitude);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&GNSSFrame), sizeof(GNSSFrame));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&GNSSFrame), sizeof(GNSSFrame));

    for (int i = 0; i < sizeof(GNSSFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&GNSSFrame) + i));
    }
    std::cout << std::endl;
}

void RobotInterface::recvLoop() {
    Header header;
    while (true) {
        // wait until head equals HEAD
        while (true) {
            serial->Recv(revBuf, 1);

            if (revBuf[0] == HEAD) {
                break;
            }
        }
//        std::cout << "rec" << std::endl;
        serial->Recv(revBuf + 1, sizeof(Header) - 1);

        if (Verify_CRC8_Check_Sum(revBuf, sizeof(Header))) {
            memcpy(&header, revBuf, sizeof(Header));
            auto p = revBuf + sizeof(Header);
                std::cout << std::hex << static_cast<int>(header.id) << std::endl;
            switch (static_cast<RecvPackageID>(header.id)) {
                case RecvPackageID::GIMBAL: {
                    serial->Recv(p, sizeof(GimbalFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(revBuf, sizeof(Header) + sizeof(GimbalFeedbackFrame))) {
                        memcpy(p, &gimbalFeedbackFrame, sizeof(GimbalFeedbackFrame));
                        base2gimbal = tf2::eigenToTransform(
                                base2gimbalTrans *
                                Eigen::AngleAxisd(gimbalFeedbackFrame.pitch / 180 * M_PI,
                                                  Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(gimbalFeedbackFrame.yaw / 180 * M_PI, Eigen::Vector3d::UnitZ())
                        );

                        base2gimbal.header.stamp = rclcpp::Clock().now();

                        transformBroadcaster->sendTransform(base2gimbal);
                    }
                }
                    break;
                case RecvPackageID::ODOM: {
                    static uint32_t count;

                    serial->Recv(p, sizeof(OdomFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(revBuf,
                                              sizeof(Header) + sizeof(OdomFeedbackFrame))) {
                        memcpy(p, &odomFeedbackFrame, sizeof(OdomFeedbackFrame));

                        odom.header.stamp = rclcpp::Clock().now();

                        odomPublisher->publish(odom);
                        Eigen::Affine3d affinePos =
                                Eigen::Translation3d(odomFeedbackFrame.x, odomFeedbackFrame.y, 0) *
                                Eigen::AngleAxisd(odomFeedbackFrame.yaw / 180 * M_PI,
                                                  Eigen::Vector3d::UnitZ());
                        world2base = tf2::eigenToTransform(affinePos);
                        world2base.header.stamp = rclcpp::Clock().now();

                        transformBroadcaster->sendTransform(world2base);
                    }

                }
                    break;
                case RecvPackageID::GOAL: {
                    serial->Recv(p, sizeof(GoalFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(revBuf,
                                              sizeof(Header) + sizeof(GoalFeedbackFrame))) {
                        memcpy(p, &goalFeedbackFrame, sizeof(GoalFeedbackFrame));
                        goal.pose.position.x = goalFeedbackFrame.x;
                        goal.pose.position.y = goalFeedbackFrame.y;
                        goalPublisher->publish(goal);
                    }
                }
                    break;
                case RecvPackageID::ENEMY: {
                    serial->Recv(p, sizeof(enemyFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(revBuf, sizeof(Header) + sizeof(enemyFeedbackFrame))) {
//                        std_srvs::SetBool enemy;
//                        enemy.request.data = enemyFeedbackFrame.isRed;

//                        enemyClient.call(enemy);
                    }

                }
                    break;
                case RecvPackageID::UWB: {

                    serial->Recv(p, sizeof(UWBFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(revBuf, sizeof(Header) + sizeof(UWBFeedbackFrame))) {
                        memcpy(p, &uwbFeedbackFrame, sizeof(UWBFeedbackFrame));
                        uwb.pose.position.x = uwbFeedbackFrame.x;
                        uwb.pose.position.y = uwbFeedbackFrame.y;
                        uwb.header.stamp = rclcpp::Clock().now();
                        UWBPublisher->publish(uwb);
                    }
                }
                    break;
                case RecvPackageID::BUFF: {
//                    std_srvs::SetBool buff;
//                    buff.request.data = true;
//                    buffClient.call(buff);
                }
                    break;
            }
        }
    }
}

