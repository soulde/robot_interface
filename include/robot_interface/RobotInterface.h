//
// Created by soulde on 2023/6/14.
//

#ifndef ROBOT_INTERFACE_ROBOTINTERFACE_H
#define ROBOT_INTERFACE_ROBOTINTERFACE_H
#define float32_t float
#define float64_t double

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>

#include "Serial.h"
#include "CRC/CRC.h"

class RobotInterface : public rclcpp::Node {

public:
    RobotInterface();

private:
    uint8_t revBuf[50];
    std::string fixFrame = "world", robotFrame = "base", gimbalFrame = "gimbal", odomFrame = "base_odom";
    Eigen::Translation3d base2gimbalTrans;

    std::string dev_name;
    std::shared_ptr<Serial> serial;

    std::unique_ptr<std::thread> thread;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPublisher;
    geometry_msgs::msg::PoseStamped goal;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    nav_msgs::msg::Odometry odom;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr UWBPublisher;
    geometry_msgs::msg::PoseStamped uwb;


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSubscriber;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr GNSSSubscriber;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticTransformBroadcaster;
    geometry_msgs::msg::TransformStamped base2gimbal, world2base;

    void twistCallback(geometry_msgs::msg::Twist::SharedPtr msg);

    void GNSSCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg);

    [[noreturn]] void recvLoop();
private:
    static constexpr unsigned char HEAD = 0xFE;

    enum class SendPackageID : unsigned char {
        GIMBAL = 0x01,
        MOVE = 0x02,
        GNSS = 0x03,
        HEARTBEAT = 0x41

    };

    enum class RecvPackageID : unsigned char {
        GIMBAL = 0x81,
        ODOM = 0x82,
        GOAL = 0x83,
        ENEMY = 0x84,
        UWB = 0x85,
        BUFF = 0xC1
    };

#pragma pack(1)
    struct Header {
        const uint8_t head = HEAD;
        uint8_t id = 0x00;
        uint8_t crc8 = 0x00;
    };

    struct MoveControlFrame {
        Header header;
        float32_t x = 0.;
        float32_t y = 0.;
        float32_t yaw = 0.;
        uint8_t crc8 = 0x0;

        MoveControlFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::MOVE);
        }

    } moveControlFrame;

    struct GimbalControlFrame {
        Header header;
        float32_t pitch = 0.;
        float32_t yaw = 0.;
        uint8_t fire = 0;
        uint8_t crc8 = 0x0;

        GimbalControlFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::GIMBAL);
        }
    } gimbalControlFrame;

    struct HeartBeatFrame {
        Header header;

        HeartBeatFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::HEARTBEAT);
        }
    } heartBeatFrame;

    struct GNSSFrame {
        Header header;
        float32_t lon;
        float32_t lat;
        float32_t alt;
        uint8_t crc8 = 0x0;

        GNSSFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::GNSS);
        }
    } GNSSFrame;

    struct GimbalFeedbackFrame {
        float32_t pitch = 0.;
        float32_t yaw = 0.;
        float32_t fire = 0.;
        uint8_t crc8 = 0x0;
    } gimbalFeedbackFrame;

    struct OdomFeedbackFrame {
        float32_t x = 0.;
        float32_t vx = 0.;
        float32_t y = 0.;
        float32_t vy = 0.;
        float32_t yaw = 0.;
        float32_t wy = 0.;
        uint8_t crc8 = 0x0;
    } odomFeedbackFrame;

    struct GoalFeedbackFrame {
        float32_t x;
        float32_t y;
    } goalFeedbackFrame;

    struct EnemyFeedbackFrame {
        uint8_t isRed;
    } enemyFeedbackFrame;

    struct UWBFeedbackFrame {
        float32_t x;
        float32_t y;
    } uwbFeedbackFrame;
#pragma pack()
};


#endif //ROBOT_INTERFACE_ROBOTINTERFACE_H
