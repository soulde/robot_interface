//
// Created by soulde on 2023/6/15.
//

#ifndef ROBOT_INTERFACE_ESTIMATOR_H
#define ROBOT_INTERFACE_ESTIMATOR_H

#include <eigen3/Eigen/Eigen>
#include <chrono>

class Estimator {
public:
    Estimator(double robotRadius, Eigen::Vector3d &x0, Eigen::Vector3d &v0, Eigen::Quaterniond &q0,
              Eigen::Quaterniond &w0);

    void fusion(Eigen::Vector4f speed, Eigen::Vector3f acc, Eigen::Vector3f gyro);

    void getState(Eigen::Vector3d &x, Eigen::Vector3d &v, Eigen::Quaterniond &q, Eigen::Quaterniond &w);

private:

    double robotRadius;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> timeNNow;
    Eigen::Vector3d x, v;
    Eigen::Quaterniond w, q;
};

template<uint stateDim, uint measurementDim>
class KF {
public:
    KF(Eigen::Matrix<double, stateDim, 1> state0, Eigen::Matrix<double, stateDim, stateDim> P0,
       Eigen::Matrix<double, measurementDim, measurementDim> Q0,
       Eigen::Matrix<double, measurementDim, measurementDim> R0) : P(P0), Q(Q0), estimateState(state0) {};

    virtual void
    predict(Eigen::Matrix<double, measurementDim, 1> input, Eigen::Matrix<double, stateDim, 1> observe, double dt) {
        observeState = observe;
        state = F * estimateState;
        P = F * P * F.transpose() + Q;
    }

    void update() {
        K = P * H.transpose() * Eigen::inverse(H * P * H.transpose() + R);
        estimateState = state + K * (observeState - H * state);
        P = (Eigen::MatrixXd::Identity() - K * H) * P;
    }

    void setF(Eigen::Matrix<double, stateDim, stateDim> F_in) {
        F = F_in;
    }

    void setH(Eigen::Matrix<double, measurementDim, stateDim> H_in) {
        H = H_in;
    }

private:
    Eigen::Matrix<double, stateDim, 1> state, estimateState, observeState;
    Eigen::Matrix<double, stateDim, stateDim> F; //transfer matrix
    Eigen::Matrix<double, stateDim, stateDim> P; // state estimate Covariance
    Eigen::Matrix<double, measurementDim, stateDim> H; //measurement matrix
    Eigen::Matrix<double, measurementDim, measurementDim> R; //noise covariance
    Eigen::Matrix<double, stateDim, stateDim> Q; //noise covariance
    Eigen::Matrix<double, stateDim, measurementDim> K; //kalman gain
};


template<uint stateDim, uint measurementDim>
class EKF : public KF<stateDim, measurementDim> {
public:
    EKF(Eigen::Matrix<double, stateDim, 1> state0, Eigen::Matrix<double, stateDim, stateDim> P0,
        Eigen::Matrix<double, measurementDim, measurementDim> Q0,
        Eigen::Matrix<double, measurementDim, measurementDim> R0) : KF<stateDim, measurementDim>(state0, P0, Q0, R0) {}

    std::function<Eigen::Matrix<double, stateDim, stateDim>(Eigen::Matrix<double, stateDim, 1>, double)> calcF;
    std::function<Eigen::Matrix<double, measurementDim, stateDim>(Eigen::Matrix<double, stateDim, 1>, double)> calcH;

    void predict(Eigen::Matrix<double, measurementDim, 1> input, Eigen::Matrix<double, stateDim, 1> observe, double dt) {
        assert(calcH != nullptr && calcF != nullptr);
        auto tmpF = calcF(observe, dt);
        auto tmpH = calcH(observe, dt);
        this->setF(tmpF);
        this->setH(tmpH);
        KF<stateDim, measurementDim>::predict(input, observe, dt);
    }

};
//Eigen::Matrix<double,

#endif //ROBOT_INTERFACE_ESTIMATOR_H
