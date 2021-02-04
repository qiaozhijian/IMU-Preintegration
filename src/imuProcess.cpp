//
// Created by qzj on 2020/9/18.
//
#include "imuProcess.h"
#include "RotUtil.h"
#include <list>
#include<iostream>

using namespace std;
using namespace Eigen;

namespace ORB_SLAM3 {

    IMUProcess::IMUProcess() : initFirstPoseFlag(false), first_imu(false),
                               prevTime(-1), curTime(0), estBiasFirst(false), mbIsUpdateBias(false),
                               frame_count(0), solver_flag(INITIAL) {
        mProcess.lock();
        while (!accBuf.empty())
            accBuf.pop();
        while (!gyrBuf.empty())
            gyrBuf.pop();

        for (int i = 0; i < WINDOW_SIZE + 1; i++) {
            Rs[i].setIdentity();
            Ps[i].setZero();
            Vs[i].setZero();
            Bas[i].setZero();
            Bgs[i].setZero();
            dt_buf[i].clear();
            linear_acceleration_buf[i].clear();
            angular_velocity_buf[i].clear();
            pre_integrations[i] = nullptr;
        }

        mZUPT = new ZUPT(0.3, 20, 500);

        R_init.setIdentity();

        for (int i = 0; i < 2; i++) {
            tic[i] = Vector3d::Zero();
            ric[i] = Matrix3d::Identity();
        }

        all_image_frame.clear();

        tmp_pre_integration = nullptr;

        mProcess.unlock();
        cout << "IMUProcess Create." << endl;
    }

    // 设置参数，并开启processMeasur ements线程
    void IMUProcess::setParameter() {
        mProcess.lock();//涉及到多线程，暂时不了解
        // 讲相机参数传入
        for (int i = 0; i < 2; i++) {
            tic[i] = TIC[0];
            ric[i] = RIC[0];
        }
        g = G;//理想的中立加速度
        cout << "set g " << g.transpose() << endl;
        mProcess.unlock();
    }

    //预测当前相机的PVQ
    void IMUProcess::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity) {
        double dt = t - latest_time;
        latest_time = t;
        Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g; //上一个世界坐标系下的加速度
        Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg; //机体坐标系下的角速度
        latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt); //更新
        Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
        latest_V = latest_V + dt * un_acc;
        latest_acc_0 = linear_acceleration;
        latest_gyr_0 = angular_velocity;
    }

    // 输入一个imu的量测
// 填充了accBuf和gyrBuf
    void
    IMUProcess::inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity) {
        mBuf.lock();
        accBuf.push(make_pair(t, linearAcceleration));
        gyrBuf.push(make_pair(t, angularVelocity));
        mBuf.unlock();
        mZUPT->estimateBias(angularVelocity);
    }


    bool IMUProcess::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                                    vector<pair<double, Eigen::Vector3d>> &gyrVector) {
        if (accBuf.empty()) {
            printf("not receive imu\n");
            return false;
        }
        //printf("get imu from %f %f\n", t0, t1);
        //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);

        // 有足够的imu值
        if (t1 <= accBuf.back().first) {
            //如果队列里第一个数据的时间小于起始时间，则删除第一个元素
            while (accBuf.front().first <= t0) {
                accBuf.pop();//.pop删除栈顶元素
                gyrBuf.pop();
            }
            // 讲队列里所有的acc和gyr输入到accvector个gyrvector中
            while (accBuf.front().first < t1) {
                accVector.push_back(accBuf.front());
                accBuf.pop();
                gyrVector.push_back(gyrBuf.front());
                gyrBuf.pop();
            }
            //fixme 这里不需要判断accBuf是否还有值吗
            //再多加一个超过时间t1的
            accVector.push_back(accBuf.front());
            gyrVector.push_back(gyrBuf.front());
        } else {
            printf("wait for imu\n");
            return false;
        }
        return true;
    }

    // 判断输入的时间t时候的imu是否可用
    bool IMUProcess::IMUAvailable(double t) {
        if (!accBuf.empty() && t <= accBuf.back().first)
            return true;
        else
            return false;
    }

    //初始第一个imu位姿
    void IMUProcess::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector) {
        printf("init first imu pose\n");
        initFirstPoseFlag = true;
        //return;
        //计算加速度的均值
        Eigen::Vector3d averAcc(0, 0, 0);
        int n = (int) accVector.size();
        for (size_t i = 0; i < accVector.size(); i++) {
            averAcc = averAcc + accVector[i].second;
        }
        averAcc = averAcc / n;
        printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());

        Eigen::Matrix3d R0 = Utility::g2R(averAcc);
        double yaw = Utility::R2ypr(R0).x();
        R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;//另初始的航向为0
        Rs[0] = R0;
        R_init = R0;
        cout << "init R0 " << endl << Rs[0] << endl;
        //Vs[0] = Vector3d(5, 0, 0);
    }

    Eigen::Matrix4d IMUProcess::getPrediction()
    {
//        R[frame_count] Tw_cur
        cout<<"frame_count: "<<frame_count<<endl;
        Eigen::Matrix4d Tw_cur = Eigen::Matrix4d::Identity();
        Tw_cur.block(0,0,3,3) = Rs[frame_count];
        Tw_cur.block(0,3,3,1) = Ps[frame_count];
        Eigen::Matrix4d Tprev_w = Eigen::Matrix4d::Identity();
        if(frame_count!=0)
        {
            Tprev_w.block(0,0,3,3) = Rs[frame_count-1].transpose();
            Tprev_w.block(0,3,3,1) = -Rs[frame_count-1].transpose()*Ps[frame_count-1];
        }
        Eigen::Matrix4d Tprev_cur = Tprev_w * Tw_cur;

        Eigen::Matrix4d Tic = Eigen::Matrix4d::Identity();
        Tic.block(0,0,3,3) = ric[0];
        Tic.block(0,3,3,1) = tic[0];
        Eigen::Matrix4d Tci = Eigen::Matrix4d::Identity();
        Tci.block(0,0,3,3) = ric[0].transpose();
        Tci.block(0,3,3,1) = -ric[0].transpose()*tic[0];

        Tprev_cur = Tci * Tci * Tic;

        return Tprev_cur;
    }

    void IMUProcess::preIntegrateIMU(double img_t) {
        //printf("process measurments\n");
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        curTime = img_t;
        //判断是否有可用的IMU
        while (1) {
            if ((IMUAvailable(curTime)))
                break;
            else {
                printf("wait for imu ... \n");
                std::chrono::milliseconds dura(5);//定义5ms的延迟
                std::this_thread::sleep_for(dura);//这个线程延迟5ms
            }
        }
        mBuf.lock();
        // 对imu的时间进行判断，讲队列里的imu数据放入到accVector和gyrVector中
        getIMUInterval(prevTime, curTime, accVector, gyrVector);
        mBuf.unlock();

        //初始化旋转
        if (!initFirstPoseFlag)
            initFirstIMUPose(accVector);
        for (size_t i = 0; i < accVector.size(); i++) {
            // 对于前n-1个，accVector[i].first 大于 prevTime 小于 curTime， 第n个 大于 curTime
            double dt;//计算每次imu量测之间的dt
            if (i == 0)
                dt = accVector[i].first - prevTime;
            else if (i == accVector.size() - 1) {
                // 因为accVector[i].first大于curTime，这个时候用curTime - accVector[i - 1].first
                dt = curTime - accVector[i - 1].first;
                //printf("time: %f.\n",dt);
            } else
                dt = accVector[i].first - accVector[i - 1].first;
            //进行了预积分，改变了Rs[frame_count]，Ps[frame_count]，Vs[frame_count]
            processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
        }
    }

    /**
 * @brief   处理IMU数据
 * @Description IMU预积分，中值积分得到当前PQV作为优化初值
 * @param[in]   dt 时间间隔
 * @param[in]   linear_acceleration 线加速度
 * @param[in]   angular_velocity 角速度
 * @return  void
*/
    void IMUProcess::processIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration,
                                const Eigen::Vector3d &angular_velocity) {
        double yaw = 0.0;
        // 第一个imu处理
        if (!first_imu) {
            first_imu = true;
            acc_0 = linear_acceleration;
            gyr_0 = angular_velocity;
        }

        // 如果是新的一帧,则新建一个预积分项目
        if (!pre_integrations[frame_count]) {
            //初始化协方差矩阵
            pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
        }

        //f rame_count是窗内图像帧的计数
        // 一个窗内有是个相机帧，每个相机帧之间又有多个IMU数据
        if (frame_count != 0) {
            //更新两帧间pvq变换量，雅克比，协方差，偏置
            pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);//跟上面一样的输入，一样的操作

            //dt,linear_acceleration,angular_velocity经过上面两步并未改变
            dt_buf[frame_count].push_back(dt);
            linear_acceleration_buf[frame_count].push_back(linear_acceleration);
            angular_velocity_buf[frame_count].push_back(angular_velocity);

            // 得到当前帧的PVQ，与fastPredictIMU中的操作类似
            // Rs Ps Vs是frame_count这一个图像帧开始的预积分值,是在世界坐标系下的.
            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;//移除了偏执的加速度
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];//移除了偏执的gyro
            Rs[j] = Rs[j] * Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] = Ps[j] + dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] = Vs[j] + dt * un_acc;
        }
        // 让此时刻的值等于上一时刻的值，为下一次计算做准备
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

// image 里边放的就是该图像的特征点 header 时间
    void IMUProcess::processImage(const double header) {
        //cout<<std::fixed<<"frame_count "<<frame_count<<" header: "<<header<<endl;
        //他俩应该相等的
        Headers[frame_count] = header;
        //将图像数据、时间、临时预积分值存到图像帧类中
        ImageFrame imageframe(header);
        imageframe.pre_integration = tmp_pre_integration;
        // 检测关键帧
        if (true) {
            imageframe.is_key_frame = true;
            marginalization_flag = MARGIN_OLD;//新一阵将被作为关键帧!
        } else {
            marginalization_flag = MARGIN_SECOND_NEW;
        }
        all_image_frame.insert(make_pair(header, imageframe));
        //更新临时预积分初始值
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

        //frameCnt 为第一帧时不进行计算
        // 将结果放入到队列当中
        if (frame_count == WINDOW_SIZE ) {
//            是否用相机的位姿对偏置进行估计
//            updatePoseFromORB3(tic, ric);
//            if(getIsUpdateBias())
//                solveGyroscopeBias(all_image_frame, Bgs);

            // 对之前预积分得到的结果进行更新。
            // 预积分的好处查看就在于你得到新的Bgs，不需要又重新再积分一遍，可以通过Bgs对位姿，速度的一阶导数，进行线性近似，得到新的Bgs求解出MU的最终结果。
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
            }
            //updateLatestStates();
            //solver_flag = NON_LINEAR;
            slideWindow();
        }

        // 如果划窗内的没有放满,进行状态更新
        if (frame_count < WINDOW_SIZE) {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    }

    void IMUProcess::updateIMUBias()
    {
        mProcess.lock();
        processImage(curTime);
        prevTime = curTime;
        mProcess.unlock();
    }

    // 滑动窗口法
    void IMUProcess::slideWindow() {
        if (marginalization_flag == MARGIN_OLD) {
            double t_0 = Headers[0];
            //滑动窗口左移一个单位
            //cout<<"all_image size 1 "<<all_image_frame.size()<<endl;
            if (frame_count == WINDOW_SIZE) {
                for (int i = 0; i < WINDOW_SIZE; i++) {
                    Headers[i] = Headers[i + 1];
                    Rs[i].swap(Rs[i + 1]);//交换
                    Ps[i].swap(Ps[i + 1]);
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);//交换预积分值

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }

                // 滑动窗口最后一个与倒数第二个一样
                Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
                Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
                Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
                delete pre_integrations[WINDOW_SIZE];//讲预积分的最后一个值删除
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                    Bgs[WINDOW_SIZE]};//在构造一个新的
                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();


                map<double, ImageFrame>::iterator it_0;
                //cout<<"t_0 "<<t_0<<endl;
                //it_0 = all_image_frame.find(t_0);//找到第一个
                it_0 = all_image_frame.begin();//找到第一个
                delete it_0->second.pre_integration;
                all_image_frame.erase(it_0);
                //all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            //cout<<"all_image size 2 "<<all_image_frame.size()<<endl;
        }
            // 把滑动窗口第二新的去掉（第一新的因为是最新的所以就挪到第二新的位置上去）
        else {
            if (frame_count == WINDOW_SIZE) {
                Headers[frame_count - 1] = Headers[frame_count];
                Ps[frame_count - 1] = Ps[frame_count];
                Rs[frame_count - 1] = Rs[frame_count];

                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration,
                                                                 tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                    Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
        }
    }

    bool IMUProcess::getIsUpdateBias() {
        unique_lock<mutex> lock(mIsUpdateBias);
        return mbIsUpdateBias;
    }

    void IMUProcess::setIsUpdateBias(bool isUpdateBias) {
        unique_lock<mutex> lock(mIsUpdateBias);
        mbIsUpdateBias = isUpdateBias;
    }
    void IMUProcess::solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs) {
        Matrix3d A;
        Vector3d b;
        Vector3d delta_bg;
        A.setZero();
        b.setZero();
        map<double, ImageFrame>::iterator frame_i;
        map<double, ImageFrame>::iterator frame_j;
        //cout<<"all_image size "<<all_image_frame.size()<<endl;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++) {
            frame_j = next(frame_i);
            if (frame_i->first >= frame_j->first) {
                cout << "map order error" << endl;
            }
            MatrixXd tmp_A(3, 3);
            tmp_A.setZero();
            VectorXd tmp_b(3);
            tmp_b.setZero();
            Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
            //cout<<"cam: "<<Utility::R2ypr(q_ij.toRotationMatrix()).transpose();
            //cout<<" imu: "<<Utility::R2ypr(frame_j->second.pre_integration->delta_q.toRotationMatrix()).transpose()<<endl;
            tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
            tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
            A += tmp_A.transpose() * tmp_A;
            b += tmp_A.transpose() * tmp_b;
        }
        delta_bg = A.ldlt().solve(b);
        //ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

        //    静止初始化，始终相信
        {
            unique_lock<mutex> lock(mBiasUpdate);

            if (!estBiasFirst) {
                estBiasFirst = true;
                for (int i = 0; i <= WINDOW_SIZE; i++)
                    Bgs[i] = Bgs[i] + delta_bg;
            } else {
                //cout<<"delta_bg.norm(): "<< delta_bg.norm()<<endl;
                //控制偏置估计不能浮动太大
                if (delta_bg.norm() < 0.01) {
                    for (int i = 0; i <= WINDOW_SIZE; i++) {
                        float alpha = 0.5;
                        Vector3d BgsNew = Bgs[i] + delta_bg;
                        Bgs[i] = alpha * Bgs[i] + (1 - alpha) * BgsNew;
                    }
                }
            }
            float alphaForZUPT = 0.2;
            Vector3d bias = mZUPT->getBias();
            if (mZUPT->mbBiasUpdate)
                for (int i = 0; i <= WINDOW_SIZE; i++)
                    Bgs[i] = alphaForZUPT * bias + (1 - alphaForZUPT) * Bgs[i];
        }

        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++) {
            frame_j = next(frame_i);
            frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
        }
    }

}
