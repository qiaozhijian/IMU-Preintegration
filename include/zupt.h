//
// Created by qzj on 2021/2/3.
//

#ifndef imu_preintegration_ZUPT_H
#define imu_preintegration_ZUPT_H

#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <queue>

class ZUPT {

public:
    ZUPT(double STATIC_THRESHOLD = 0.3, int STATIC_JUDGE_NUM = 20, int STATIC_BUFFER_NUM = 500) : STATIC_THRESHOLD(
            STATIC_THRESHOLD), STATIC_JUDGE_NUM(STATIC_JUDGE_NUM), STATIC_BUFFER_NUM(STATIC_BUFFER_NUM) {

        //for(int i=0;i<STATIC_BUFFER_NUM;i++)
        //    wRawSlide.push(Eigen::Vector3d::Zero());
        staticCount = 0;
        mbBiasUpdate = false;
        wBiasSum.setZero();
        wBias.setZero();
        staticCountLast = 0;
        staticNum = 0;
    };

    void estimateBias(const Eigen::Vector3d &wRaw);

    bool CheckStatic(double wz);

    void setBias(const Eigen::Vector3d &bias) {
        std::unique_lock<std::mutex> lock(mutexBias);
        wBias = bias;
    }

    Eigen::Vector3d getBias() {
        std::unique_lock<std::mutex> lock(mutexBias);
        return wBias;
    }

    bool mbBiasUpdate;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const double RAD_TO_ANGLE = 57.295779515;
    int STATIC_BUFFER_NUM;
    int STATIC_JUDGE_NUM;
    double STATIC_THRESHOLD;
    std::queue<Eigen::Vector3d> wRawSlide;
    unsigned int staticCount;
    unsigned int staticNum = 0;
    Eigen::Vector3d wBiasSum;
    unsigned int staticCountLast;

    std::mutex mutexBias;
    Eigen::Vector3d wBias;
};


#endif //imu_preintegration_ZUPT_H
