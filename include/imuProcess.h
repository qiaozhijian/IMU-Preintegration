//
// Created by qzj on 2020/9/18.
//

#ifndef SRC_IMUPROCESS_H
#define SRC_IMUPROCESS_H

#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "parameters.h"
#include "zupt.h"
#include "ImageFrame.h"

namespace ORB_SLAM3 {
    enum MarginalizationFlag {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    enum SolverFlag {
        INITIAL,// 还未成功初始化
        NON_LINEAR // 已成功初始化，正处于紧耦合优化状态
    };

    class IMUProcess {
    public:
        IMUProcess();

        bool getIMUInterval(double t0, double t1, std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
                            std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector);

        void inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity);

        void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);

        void preIntegrateIMU(double img_t);

        bool IMUAvailable(double t);

        void initFirstIMUPose(std::vector<std::pair<double, Eigen::Vector3d>> &accVector);

        void setParameter();

        void processImage(const double header);

        void slideWindow();

        void updateIMUBias();

        void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d *Bgs);

        void updatePoseFromORB3(Eigen::Vector3d tic[], Eigen::Matrix3d ric[]);

        void processIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration,
                        const Eigen::Vector3d &angular_velocity);

        Eigen::Matrix4d getPrediction();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::queue<std::pair<double, Eigen::Vector3d>> accBuf;
        std::queue<std::pair<double, Eigen::Vector3d>> gyrBuf;

        IntegrationBase *tmp_pre_integration;//这个是输入到图像中的预积分值
        Eigen::Vector3d acc_0, gyr_0;

        /***********滑动窗口*************/
        std::map<double, ImageFrame> all_image_frame;
        //窗口内所有帧的时间
        double Headers[(WINDOW_SIZE + 1)];
        //窗口中的dt,a,v
        vector<double> dt_buf[(WINDOW_SIZE + 1)];
        vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
        vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
        IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];//里边放的是imu的预积分
        Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];//划窗内所有的p
        Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];//划窗内所有的速度
        Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];//划窗内所有的R
        Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];//划窗内所有的bias of a
        Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];//划窗内所有的bias of g

        Eigen::Matrix3d R_init;

        Eigen::Matrix3d ric[2];
        Eigen::Vector3d tic[2];

        int frame_count; //窗口内的第几帧,最大值为WINDOW_SIZE + 1
        double curTime, prevTime;
        double latest_time;
        Eigen::Vector3d g;
        Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
        Eigen::Quaterniond latest_Q;

        bool estBiasFirst;
        bool first_imu;//该图像之后的第一个imu
        bool initFirstPoseFlag;//IMU初始位姿的flag
        SolverFlag solver_flag;
        MarginalizationFlag marginalization_flag;

        //检测静止估计IMU零漂
        //Zero-velocity Update,ZUPT
        ZUPT *mZUPT;

    public:
        bool getIsUpdateBias();

        void setIsUpdateBias(bool isUpdateBias);

    private:
        std::mutex mIsUpdateBias;
        bool mbIsUpdateBias;

        std::mutex mProcess;
        std::mutex mBuf;
        std::mutex mPropagate;
        std::mutex mBiasUpdate;
    };

}

#endif //SRC_IMUPROCESS_H
