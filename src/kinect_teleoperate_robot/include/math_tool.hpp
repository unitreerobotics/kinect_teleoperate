#pragma once

#include <k4a/k4a.h>
#include <k4abt.h>

#include <Eigen/Dense>

#include <cmath>
#include <deque>
#include <iostream>


class MovingAverageFilter {
public:
    MovingAverageFilter(size_t max_size = 8, double epsilon = 1e-9) 
        : max_size_(max_size), epsilon_(epsilon), sum_(0.0) {
        if (max_size == 0) {
            throw std::invalid_argument("max_size must be greater than 0");
        }
    }

    double update(double new_value) {
        if (std::isinf(new_value) || std::isnan(new_value)) {
            std::cerr << "Warning: new value is inf or nan, ignoring this value." << std::endl;
            return average_;
        }
        // The Control_loop thread runs at a much higher frequency than the Main_loop. 
        // When the Main_loop has not updated the data, the Control_loop will input the old value into the filter. 
        // The old value should be skipped here.
        if (!history_.empty() && std::fabs(history_.back() - new_value) < epsilon_) {
            return average_;
        }

        if (history_.size() >= max_size_) {
            double oldest_value = history_.front();
            history_.pop_front();
            sum_ -= oldest_value;
        }

        history_.push_back(new_value);
        sum_ += new_value;

        average_ = sum_ / history_.size();

        return average_;
    }


private:
    std::deque<double> history_;
    size_t max_size_;
    double epsilon_;
    double sum_;
    double average_;
};

// Test to get the rotation order, because the coordinate system definition of the Kinect camera is still a mystery
void quaternion2Euler(k4a_quaternion_t q, float& roll, float& pitch, float& yaw)
{
    float w = q.wxyz.w;
    float x = q.wxyz.x;
    float y = q.wxyz.y;
    float z = q.wxyz.z;

    double sinp_cosp = 2 * (w * y - z * x);
    double cosp_cosp = 1 - 2 * (x * x + y * y);
    pitch = std::atan2(sinp_cosp, cosp_cosp);

    double sinr = 2 * (w * x + y * z);
    if (std::abs(sinr) >= 1)
        roll = std::copysign(M_PI / 2.0, sinr);
    else
        roll = std::asin(sinr);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}


Eigen::Quaternionf k4aToEigenQuaternion(const k4a_quaternion_t& k4a_q) {
    return Eigen::Quaternionf(k4a_q.wxyz.w, k4a_q.wxyz.x, k4a_q.wxyz.y, k4a_q.wxyz.z);
}

k4a_quaternion_t calculateRelativeQuaternion(const k4a_quaternion_t& q, const k4a_quaternion_t& reference) {
    Eigen::Quaternionf eigen_q = k4aToEigenQuaternion(q);
    Eigen::Quaternionf eigen_reference = k4aToEigenQuaternion(reference);

    Eigen::Quaternionf relative_q = eigen_reference.conjugate() * eigen_q;

    k4a_quaternion_t result;
    result.wxyz.w = relative_q.w();
    result.wxyz.x = relative_q.x();
    result.wxyz.y = relative_q.y();
    result.wxyz.z = relative_q.z();

    return result;
}

float calculateRelativeAngle(const k4a_quaternion_t &q1, const k4a_quaternion_t &q2)
{
    float dot = q1.wxyz.w * q2.wxyz.w + q1.wxyz.x * q2.wxyz.x + q1.wxyz.y * q2.wxyz.y + q1.wxyz.z * q2.wxyz.z;
    dot = fmin(dot < 0.0f ? -dot : dot, 1.0f);
    float angle = 2.0f * acos(dot) * (180.0f / M_PI);
    return angle;
}