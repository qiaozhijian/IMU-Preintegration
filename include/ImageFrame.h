//
// Created by qzj on 2021/2/3.
//

#ifndef IMU_PREINTEGRATION_IMAGEFRAME_H
#define IMU_PREINTEGRATION_IMAGEFRAME_H


#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <queue>
#include "IntegrationBase.h"

class ImageFrame {
public:
    ImageFrame() {};

    ImageFrame(double _t) : t{_t}, is_key_frame{false} {

    };
    double t;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
};


#endif //IMU_PREINTEGRATION_IMAGEFRAME_H
