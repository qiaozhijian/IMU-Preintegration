//
// Created by qzj on 2021/2/3.
//

#include "zupt.h"
#include <iostream>

using namespace std;

bool ZUPT::CheckStatic(double wz) {
    wz = fabs(wz * RAD_TO_ANGLE);
    //ROS_INFO("%f",wz);
    if (wz < STATIC_THRESHOLD) {
        staticNum++;
        //cout<<"ZUPT staticNum "<<staticNum<<endl;
    } else
        staticNum = 0;
    if (staticNum > STATIC_JUDGE_NUM) {
        staticNum = STATIC_JUDGE_NUM + 1;
        return true;
    } else
        return false;
}

// 输入原始角速度wRaw，输出去零飘的角速度wNew
void ZUPT::estimateBias(const Eigen::Vector3d &wRaw) {
    Eigen::Vector3d wNew;
    if (staticCount >= STATIC_BUFFER_NUM) {
        mbBiasUpdate = true;
        setBias(wBiasSum / double(STATIC_BUFFER_NUM));
        wNew = wRaw - getBias();
    } else
        wNew = wRaw;

    if (CheckStatic(wNew[2])) {
        staticCount++;
        wRawSlide.push(wRaw);
        wBiasSum = wBiasSum + wRawSlide.back();
        if (staticCount > STATIC_BUFFER_NUM) {
            wBiasSum = wBiasSum - wRawSlide.front();
            wRawSlide.pop();
            staticCount = STATIC_BUFFER_NUM;
        }
    } else
        //    staticCount = 0;

    if (staticCount > 0 && staticCountLast == 0)
        cout << "ZUPT robot is static" << endl;
    else if (staticCount == 0 && staticCountLast != 0)
        cout << "ZUPT robot start to move" << endl;
    if (staticCount == STATIC_BUFFER_NUM && staticCountLast != STATIC_BUFFER_NUM)
        cout << "ZUPT imu bais update" << endl;
    staticCountLast = staticCount;

    //for (unsigned char axis = 0; axis < 3; axis++)
    //    if (fabs(wNew[axis] * RAD_TO_ANGLE) < STATIC_THRESHOLD)
    //        wNew[axis] = 0.0;
}
