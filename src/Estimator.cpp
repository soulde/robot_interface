//
// Created by soulde on 2023/6/15.
//

#include "robot_interface/Estimator.h"

Estimator::Estimator(double robotRadius, Eigen::Vector3d &x0, Eigen::Vector3d &v0, Eigen::Quaterniond &q0,
                     Eigen::Quaterniond &w0) :robotRadius(robotRadius),timeNNow(std::chrono::system_clock::now()),x(x0),v(v0),w(w0),q(q0){

}

void Estimator::fusion(Eigen::Vector4f speed, Eigen::Vector3f acc, Eigen::Vector3f gyro) {

}

void Estimator::getState(Eigen::Vector3d &x, Eigen::Vector3d &v, Eigen::Quaterniond &q, Eigen::Quaterniond &w) {

}
