/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<ros/ros.h>
#include <ros/time.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<opencv2/core/core.hpp>
#include"ImuTypes.h"
#include "parameters.h"
#include "imuProcess.h"
#include "global_defination.h"
#include "nav_msgs/Odometry.h"

std::shared_ptr<ORB_SLAM3::IMUProcess> imuProcessor;

static uint32_t imuCnt=0;
static bool IMUReady = false;

using namespace std;
class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){
    }

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void SyncWithImu();
    void publishPose(Eigen::Matrix4d &pose, double t);

public:
    queue<sensor_msgs::ImageConstPtr> imgLeftBuf;
    std::mutex mBufMutexLeft;

    ImuGrabber *mpImuGb;

    ros::Publisher pubIMUPrediction;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_odo_node3");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    imuProcessor.reset(new ORB_SLAM3::IMUProcess());
    readParameters(WORK_SPACE_PATH+"/Examples/parameter.yaml");
    imuProcessor->setParameter();

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/imu0", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = n.subscribe("/cam0/image_raw", 10, &ImageGrabber::GrabImageLeft,&igb);

    igb.pubIMUPrediction = n.advertise<nav_msgs::Odometry>("/imu_prediction", 5);
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(!IMUReady)
        return;
    mBufMutexLeft.lock();
//    if (!imgLeftBuf.empty())
//        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::publishPose(Eigen::Matrix4d &pose, double t) {

    nav_msgs::Odometry poseRos;
    
    Eigen::Vector3d transV(pose(0, 3), pose(1, 3), pose(2, 3));
    Eigen::Matrix3d rotMat = pose.block(0, 0, 3, 3);
    Eigen::Quaterniond qua(rotMat);

    poseRos.header.stamp = ros::Time().fromSec(t);
    poseRos.pose.pose.position.x = transV(0);
    poseRos.pose.pose.position.y = transV(1);
    poseRos.pose.pose.position.z = transV(2);
    poseRos.pose.pose.orientation.x = qua.x();
    poseRos.pose.pose.orientation.y = qua.y();
    poseRos.pose.pose.orientation.z = qua.z();
    poseRos.pose.pose.orientation.w = qua.w();

    pubIMUPrediction.publish(poseRos);

}

void ImageGrabber::SyncWithImu()
{
    cout<<"SyncWithImu start"<<endl;
    while(1)
    {
        double tImLeft = 0;
        if (!imgLeftBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();

            // IMU值太少了,需要在相机之后依然有
            if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            //载入IMU数据
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=(tImLeft+0.01))
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            imuProcessor->preIntegrateIMU(tImLeft);
            Eigen::Matrix4d Tprev_cur = imuProcessor->getPrediction();
            publishPose(Tprev_cur, tImLeft);
            imuProcessor->updateIMUBias();
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(rx, ry, rz);
    imuProcessor->inputIMU(t, acc, gyr);
    imuCnt++;
    if(imuCnt>5)
        IMUReady = true;
    return;
}


