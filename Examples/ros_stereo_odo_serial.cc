#include<algorithm>
#include<chrono>
#include<vector>
#include<queue>
#include<mutex>
#include<ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include<sensor_msgs/Imu.h>
#include"ImuTypes.h"
#include "imuProcess.h"
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>

using namespace std;

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;

    std::mutex mBufMutex;
};

std::shared_ptr<ORB_SLAM3::IMUProcess> imuProcessor;

int main(int argc, char **argv) {

    // Location of the ROS bag we want to read in
    std::string path_to_bag = "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/01/2020-07-26-19-47-34.bag";
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

    imuProcessor.reset(new ORB_SLAM3::IMUProcess());
    imuProcessor->setParameter();

    // Our camera topics (left and right stereo)
    std::string topic_imu = "/imu0";
    ImuGrabber imugb;

    double bag_start = 0, bag_durr = -1;
    rosbag::Bag bag;
    rosbag::View view;
    bag.open(path_to_bag, rosbag::bagmode::Read);
    view.addQuery(bag);

    ROS_INFO("bag start: %.1f", bag_start);
    ROS_INFO("bag duration: %.1f", bag_durr);
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    // Step through the rosbag
    for (const rosbag::MessageInstance &m : view) {
        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != nullptr && m.getTopic() == topic_imu) {
            imugb.GrabImu(s2);
        }
    }
    return 0;
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
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
    imuProcessor->preIntegrateIMU(t);
    return;
}


