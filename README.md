## 1.环境配置
本代码在Ubuntu 18.04上测试。
1.1依赖包
```js
sudo apt-get install libcgal-dev
#我们使用的是较高的1.65.0版本
sudo apt-get install libboost-all-dev
#根据你的ros版本替换melodic,这个包用于串口读取机器人的数据,如果使用数据集可以不安装
sudo apt install ros-melodic-serial
sudo apt-get install libcanberra-gtk*
```
##### 1.2第三方库安装(catkin_ws/src/Thirdparty文件夹)
+ opencv
	+ 我们替换了lsd.cpp文件
	+ 进入opencv文件夹,运行build.sh(可以选择性注释最后一句安装)
+ [PCL](https://pcl-tutorials.readthedocs.io/en/latest/compiling_pcl_posix.html)
+ [ceres](http://ceres-solver.org/installation.html)
+ Pangolin
##### 1.3可选安装(catkin_ws/src/ORB-SLAM3/Thirdparty)
+ EDLines/EDTest
	+ 用于半稠密建图
	+ 如果不需要则应该在catkin_ws/src/ORB-SLAM3/CMakeLists.txt去掉该部分,以及在catkin_ws/src/ORB-SLAM3/build.sh里去掉该部分的编译
+ Thirdparty/SLAM++ 
	+ 如果使用good graph,则需要这个库
	+ 如果不需要则应该在catkin_ws/src/ORB-SLAM3/CMakeLists.txt去掉该部分,以及在catkin_ws/src/ORB-SLAM3/build.sh里去掉该部分的编译
	+ 如果需要
		+ 1，安装armadillo之前所需要的库：
        	+ sudo apt-get install liblapack-dev
        	+ sudo apt-get install libblas-dev
        	+ sudo apt-get install libboost-dev
   		+ 2，安装armadillo：
    		+ sudo apt-get install libarmadillo-dev
+ Thirdparty/eigen3
	+ 如果你们的eigen版本有问题,可以使用这一版


## 2.demo运行
修改launch/sweeper.launch中的文件夹地址为本地存放数据集和参数文件的地址
```
roslaunch launch/sweeper.launch
```
按下空格开始播放数据集



## 3.定位与建图
+ cd catkin_ws/src/ORB-SLAM3/ && sudo ./build.sh
+ catkin_make or catkin build
+ 执行定位与建图程序
	+ roslaunch orb_slam3 robot_odo_stereo.launch
	+ 位姿保存在/home/user/.ros/里,名字为FrameTrajectory_TUM_Format.txt
##### 可选的一些功能,可以通过宏定义去选择
+ 双目匹配加速(ALTER_STEREO_MATCHING / selfDefine.h)
	+ 默认开启
	+ 单位秒
	+ 结果
	    + ALTER_STEREO_MATCHING 
            + min tracking time: 0.0188
            + max tracking time: 0.1133
            + median tracking time: 0.0265
            + mean tracking time: 0.0285
	    + NO ALTER_STEREO_MATCHING
            + min tracking time: 0.0280
            + max tracking time: 0.1555
            + median tracking time: 0.0379
            + mean tracking time: 0.0382
+ 2D地图保存和显示(SAVE_MAP_PCD / selfDefine.h)
	+ 默认开启
	+ 需要安装
		+ sudo apt install ros-melodic-octomap-*
	+ 默认开启,如果使用launch,地图默认保存在/home/user/.ros/里,名字为slam_map.pcd
	+ 保存完毕后,修改publish_pointcloud里的demo.launch,把pcd文件修改为刚刚生成的这个
	    + roslaunch publish_pointcloud demo.launch
+ 词典(USE_FBOW,USE_DBOW3,USE_DBOW2 / selfDefine.h)
	+ 默认DBoW3
	+ 精度上,DBoW3,DBoW2大于fbow;速度上,fbow略大于DBoW3,大于DBoW2
	+ 使用关键帧图片离线构建辞典(CEATE_VOC / selfDefine.h),默认不开启
	+ 构建词典过程可能会超过ros限定的时间15s(escalating to SIGTERM)，导致词典建立失败,百度有解决方案
	+ 需要在launch文件里调整对应的词典
        + fbow:ORB-SLAM3/Vocabulary/ORBvoc.fbow
        + DBow3:ORB-SLAM3/Vocabulary/ORBvoc.txt
        + DBow2:ORB-SLAM3/Vocabulary/ORBvoc.txt
        + 该文件夹其他词典是我们利用实验室场景生成的词典,已验证可以用于特征匹配和回环检测,但我们更相信ORB-SLAM提供的,因为他场景丰富,所以默认用他们的
+ Good Graph(ENABLE_GOOD_GRAPH / selfDefine.h)
    + 使用hash树优化地图中的关键帧
	+ 因为效果不明显(不会变差,但也没多好),且不好维护,默认不开启
+ 半稠密建图(USE_SEMI_DENSE_MAP / selfDefine.h)
    + 借用线特征进行理想半稠密建图
	+ 测试发现对于大地图和纹理缺失场景不鲁棒,且时间较长,默认不开启
	+ 我们提供了一个较好的结果在dataset/01里,semi_pointcloud.obj,可以用meshlab打开
    + 可能出现时间过长而自动退出的情况(escalating to SIGTERM)，百度有解决方案,这种情况也可以运行bin里的ros_stereo_odo_serial_node3，例如./ros_stereo_odo_serial_node3 /home/qzj/code/catkin_ws/src/ORB-SLAM3/Vocabulary/ORBvoc.txt
/home/qzj/code/catkin_ws/src/ORB-SLAM3/config/robot/robot_orb_stereo_new.yaml
/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/10/2020-09-25-13-30-56.bag
##### 评估轨迹误差
   + [evo](https://github.com/MichaelGrupp/evo.git)
        + numpy,scipy,matplotlib
       + 仅用于可视化,不用于计算精确度,可以不安装
       + cd catkin_ws/src/ORB-SLAM3/evaluation/liegroups && pip install . && cd ..
       + python eval_self.py --est_path YOUR_PATH/FrameTrajectory_TUM_Format.txt --gt_path YOUR_PATH/dataset/10/vicon_10.txt --end_time 1601012055.929288 --scaleAlign
       + example: python eval_self.py --est_path /home/qzj/code/catkin_ws/src/args/SLAM/FrameTrajectory_TUM_Format.txt --gt_path /home/qzj/code/catkin_ws/src/args/SLAM/vicon_10.txt --end_time 1601012055.929288 --scaleAlign
       + 因为后期超过了vicon的范围,所以不能评估全部轨迹
       +  轨迹评估标准是当前误差比上以走路程,单位百分比
       + trans error mean:  0.6453431039831751
       + trans error median:  0.5469408115732155
       + rot error mean:  0.0027372962622565326
       + rot error median:  0.0010774216933725118
## 3.重定位
+ 在定位建图编译时,已经编译了相关程序,相关程序在catkin_ws/src/ORB-SLAM3/relocal文件夹里
+ 利用一个序列进行词典训练(训练点线词典)
	+ cd catkin_ws/src/ORB-SLAM3/bin
	+ ./feature_training YOUR_PATH/dataset/
	+ 在当前目录生成点词典 robot_vocab_pt.yml.gz
	+ 在当前目录生成线词典 robot_vocab_line.yml.gz
+ 利用另外两个序列进行重定位,用于测试成功率
	+ cd catkin_ws/src/ORB-SLAM3/bin
	+ ./loop_closure YOUR_PATH/dataset/
	+ 在当前目录生成结果 result_dbow_pl.txt, 每行第一个是当前图片序号,后面是按相似程度排序的重定位图片序号结果
	+ 这一步运行很长,因为指标要求百分之95,所以用了大量的图片,仅仅使用轨迹中的回环是算不出这个指标的.
	+ 所以,我们提供了result_dbow_pl.txt的结果
+ 测试成功率
	+ cd catkin_ws/src/ORB-SLAM3/evaluation
	+ python generate_dataset.py --path YOUR_PATH/dataset/
	    + 用vicon数据生成重定位真值,认为旋转小于15度,平移小于0.2m是在相同地点
	    + 生成真值文件test_positive_15_0.2.txt
	+ 请把之前生成的result_dbow_pl.txt复制到当前文件夹
	+ python generate_dataset.py --path YOUR_PAH/dataset/ --test
	    + 每一行,例如50 97.60132995566815,代表,对于数据集中的一张图片,我们的算法找出的前50个最相似的结果都是对的,则记该张图片成功，计算整个数据集上的成功率.
    + 成功率影响因素:
        + 可以适当调小树深度;
        + 训练词典时尽量挑选有代表性图片,例如关键帧,而不是所有图片

## 4.主方向识别
主方向识别只在初期运行,得到主方向之后就停止运行。主方向识别利用房间中和房间垂直的直线信息,要求运行期间机器人运动尽量平稳,可以有一些旋转运动.
运行sweeper.launch后，结果保存在catkin_ws/src/args/all_dataset/main_direction_result/main_direction_result.txt文件中。
各列表示：

(1) 用于主方向识别的50帧中第一帧的时间戳； 

(2) 房间主方向相对于相机的角度值； 

(3) 置信度； 

(4) 程序开始运行的全局第一帧时间戳； 

(5) 全局第一帧房间主方向相对于相机的角度

真实值通过vicon测量得到，保存在catkin_ws/src/args/all_dataset/main_direction_result/mian_direction_gt.txt文件中

通过时间戳将检测结果和真实值对应和比较

## 5. 障碍物识别和四类物体检测
由于机器人在移动过程中不容易对物体的位置进行准确测量,因此单独录制了数据集进行障碍物和四类物体检测的精度测试,通过以下命令运行:

```
roslaunch launch/object_detection.launch
```
按下空格键开始播放数据集


结果存放于catkin_ws/src/args/obj_det_dataset/object_det_result/object_det_matlab_sgbm.txt， 一行表示一个识别到的物体，各列分别表示：

(1) 帧序号

(2) 该帧yolo检测到的物体数量

(3) 该物体类别序号

(4) 该物体世界坐标系下x、y、z坐标(m)

```
# 修改data_to_catkin_ws到自己的目录
# 计算准确率
python object_detection/scripts/eval/eval.py  data_to_catkin_ws/src/args/obj_det_dataset/ object_det_matlab_sgbm.txt
```
位置估计误差输出如下,其中z表示z轴方向(深度方向)的误差,x表示x轴方向(横向)的误差,序号0-3分别表示羽毛球,线,麻将,狗屎这四种类型的物体,仅对0.5m-1.4m范围内的物体进行精度评估
```
////////////////obsatcle result//////////////////
e_z_0: 0.014215000000000021
e_z_1: 0.024795425000000003
e_z_2: 0.012935264705882354
e_z_3: 0.01760174999999999
e_x_0: 0.032323202894736835
e_x_1: 0.035623471250000004
e_x_2: 0.03481014411764706
e_x_3: 0.031825130000000014
```
识别准确率为输入如下,前四行分别输出四种物体各自的识别成功个数,总个数,识别成功率,最后一行输出总的识别成功率
```
count0: 38 38 1.0
count1: 40 40 1.0
count2: 34 40 0.85
count3: 40 41 0.975609756097561
all:0.9559748427672956
```

## 6.毛毯识别
Carpet Detection For Sweeper

carpet_detection文件夹为ros下创建的程序包，主要实现毛毯识别功能;

carpet_detection/src/目录下的carpet_process.cpp为源代码，节点名称为carpet_process;

carpet_detection/sweeper_launch/目录下的sweeper.launch用于同时启动三个节点，分别为发布视频数据集节点、障碍物检测节点以及上述提及的毛毯识别节点（需结合其他文件和程序包执行）;

carpet_process节点共接收畸变矫正后的图像，障碍物掩膜以及yolo检测到物体的掩膜三个消息，共发布两个消息, 话题名称分别为“/carpet_region”和“/carpet_region_rec”，分别对应于毛毯区域及其包络框的图像和毛毯区域所在的外接矩形区域图像。

关于毛毯识别部分的相关参数可打开carpet_process.cpp文件中的rectifiedCallback函数进行调节：

1.visualize_process_flag: true为可视化, false为省略所有的处理图像;

2.canny_threshold为边缘检测阈值，阈值越大，能够识别出的直线越多，反之则越少;

3.close_operation_size为闭操作核的大小， 若该闭操作核过大，可能会使诸多不同类的物体相连构成一个连通区域，影响识别效果;

4.open_operation_size为开操作核的大小，主要用于剔除较小的干扰区域;

5.carpet_area_threshold为毛毯区域面积的阈值，阈值越小，所呈现出毛毯面积越大，同时可能引入其他无毛毯的区域;

6.carpet_width_threshold为毛毯长度的阈值，作用类似于carpet_area_threshold;

7.detected_times_threshold为连续检测到地毯的帧数阈值，主要用于排除偶然误差。

备注：当visualize_process_flag设置为true时，将显示10个窗口。每个窗口及其对应关系为：

A.窗口名称：yolo_mask_resize， 图像为裁剪后的yolo_mask;

B.窗口名称：obstacle_mask_resize， 图像为裁剪后的obstacle_mask;

C.窗口名称：object_detection_mask， 图像为合并两个mask后的掩膜;

D.窗口名称：edge_roi， 图像为边缘检测图像;

E.窗口名称：close_operation_mask， 图像为边缘图像经过闭操作处理后的掩膜;

F.窗口名称：merged_mask， 图像为上述所有掩膜合并后的mask;

G.窗口名称：origin_left_roi， 图像为畸变矫正后的感兴趣区域;

H.窗口名称：open_operation_merged_mask， 图像为对mask进行开操作后处理的图像;

I.窗口名称：Carpet Region， 图像为地毯区域及其包络框， 发布在“/carpet_region”话题上;

J.窗口名称：display_carpet_region_in_rec， 图像为地毯区域对应的外接矩形区域图像， 发布在“/carpet_region_rec”话题上。  

成功识别毛毯的标准：所识别到毛毯区域的面积至少占图像中毛毯实际面积的50%以上（通过肉眼判断）。