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
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<opencv2/core/core.hpp>
#include"ImuTypes.h"
#include "parameters.h"
#include "imuProcess.h"

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
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf;
    std::mutex mBufMutexLeft;

    ImuGrabber *mpImuGb;

};

vector<float> vTimesTrack;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_odo_node3");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    imuProcessor.reset(new ORB_SLAM3::IMUProcess());
    imuProcessor->setParameter();

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/imu0", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 1, &ImageGrabber::GrabImageLeft,&igb);
    std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

    ros::spin();

    return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(!IMUReady)
        return;
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    //img_msg->header.stamp = ;
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    if(img_msg->encoding == sensor_msgs::image_encodings::BGR8)
    {
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if(cv_ptr->image.type()==16)
        {
            return cv_ptr->image.clone();
        }
        else
        {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }else if(img_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if(cv_ptr->image.type()==0)
        {
            return cv_ptr->image.clone();
        }
        else
        {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }
}

void ImageGrabber::SyncWithImu()
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    while(1)
    {
        cv::Mat imLeft;
        double tImLeft = 0;
        if (!imgLeftBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();

            // IMU值太少了,需要在相机之后依然有
            if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            t1 = std::chrono::steady_clock::now();
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
            t3 = std::chrono::steady_clock::now();
            imuProcessor->updateIMUBias();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
            vTimesTrack.push_back(ttrack);
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
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
    cv::Point2f encoder(imu_msg->angular_velocity_covariance[4],
                        imu_msg->angular_velocity_covariance[5]);
    imuProcessor->inputIMU(t, acc, gyr);
    imuCnt++;
    if(imuCnt>5)
        IMUReady = true;
    return;
}


