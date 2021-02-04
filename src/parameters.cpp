/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double TD;
int NUM_OF_CAM;
int USE_IMU;
int FREQ_IMU;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
std::vector<std::string> CAM_NAMES;

using namespace std;

void readParameters(std::string config_file) {
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL) {
        cout << "config_file dosen't exist; wrong config_file path" << endl;
        //ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    FREQ_IMU = fsSettings["IMU.Frequency"];
    double sf = sqrt(FREQ_IMU);
    ACC_N = fsSettings["IMU.NoiseAcc"].real() * sf;
    ACC_W = fsSettings["IMU.AccWalk"].real() / sf;
    GYR_N = fsSettings["IMU.NoiseGyro"].real() * sf;
    GYR_W = fsSettings["IMU.GyroWalk"].real() / sf;
    G.z() = fsSettings["g_norm"];

    cout << endl;
    cout << "IMU frequency: " << FREQ_IMU << " Hz" << endl;
    cout << "IMU discrete gyro noise: " << GYR_N << " rad/s" << endl;
    cout << "IMU discrete gyro walk: " << GYR_W << " rad/s" << endl;
    cout << "IMU discrete accelerometer noise: " << ACC_N << " m/s^2" << endl;
    cout << "IMU discrete accelerometer walk: " << ACC_W << " m/s^2" << endl;

    ESTIMATE_EXTRINSIC = 0;
    cv::Mat cv_T;
    fsSettings["Tbc"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));

    CAM_NAMES.push_back("");
    NUM_OF_CAM = 1;
    TD = 0.f;
    ESTIMATE_TD = 0;

    fsSettings.release();
}
