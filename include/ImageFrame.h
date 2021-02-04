//
// Created by qzj on 2021/2/3.
//

#ifndef imu_preintegration_IMAGEFRAME_H
#define imu_preintegration_IMAGEFRAME_H


#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <queue>
#include "IntegrationBase.h"

class ImageFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageFrame() {};

    ImageFrame(double _t) : t{_t}, is_key_frame{false} {

    };
    double t;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
};


#endif //imu_preintegration_IMAGEFRAME_H
