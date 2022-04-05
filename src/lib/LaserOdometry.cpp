// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <pcl/filters/filter.h>

#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/common.h"
#include "math_utils.h"

namespace loam
{

  using std::sin;
  using std::cos;
  using std::asin;
  using std::atan2;
  using std::sqrt;
  using std::fabs;
  using std::pow;


  LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations):
    BasicLaserOdometry(scanPeriod, maxIterations),
    _ioRatio(ioRatio)
  {
    // initialize odometry and odometry tf messages 初始化(里程计message)和(里程计tf message)：设置帧ID和child帧ID
    _laserOdometryMsg.header.frame_id = "camera_init";
    _laserOdometryMsg.child_frame_id  = "laser_odom";

    _laserOdometryTrans.frame_id_       = "camera_init";
    _laserOdometryTrans.child_frame_id_ = "laser_odom";
  }


  bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
  {
    // fetch laser odometry params
    float fParam;
    int iParam;

    // 以下ROS_ERROR错误提示信息基本全是错的

    if (privateNode.getParam("scanPeriod", fParam))
    { // 扫描周期必须为正数
      if (fParam <= 0)
      {
        ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
        return false;
      }
      else
      {
        setScanPeriod(fParam);
        ROS_INFO("Set scanPeriod: %g", fParam);
      }
    }

    if (privateNode.getParam("ioRatio", iParam))
    { // 输入输出帧数比(整数)必须>=1(输入帧数>=输出帧数)
      if (iParam < 1)
      {
        ROS_ERROR("Invalid ioRatio parameter: %d (expected > 0)", iParam);
        return false;
      }
      else
      {
        _ioRatio = iParam;
        ROS_INFO("Set ioRatio: %d", iParam);
      }
    }

    if (privateNode.getParam("maxIterations", iParam))
    { // 最大迭代次数必须>=1
      if (iParam < 1)
      {
        ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
        return false;
      }
      else
      {
        setMaxIterations(iParam);
        ROS_INFO("Set maxIterations: %d", iParam);
      }
    }

    if (privateNode.getParam("deltaTAbort", fParam))
    { // 优化中deltaT的迭代终止阈值必须为正数
      if (fParam <= 0)
      {
        ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
        return false;
      }
      else
      {
        setDeltaTAbort(fParam);
        ROS_INFO("Set deltaTAbort: %g", fParam);
      }
    }

    if (privateNode.getParam("deltaRAbort", fParam))
    { // 优化中deltaR的迭代终止阈值必须为正数
      if (fParam <= 0)
      {
        ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
        return false;
      }
      else
      {
        setDeltaRAbort(fParam);
        ROS_INFO("Set deltaRAbort: %g", fParam);
      }
    }

    // advertise laser odometry topics 发布里程计处理结果topic
    _pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
    _pubLaserCloudSurfLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
    _pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);
    _pubLaserOdometry = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);

    // subscribe to scan registration topics 订阅：registration模块发布的6个topic，每个topic用1个单独的回调函数处理
    _subCornerPointsSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_sharp", 2, &LaserOdometry::laserCloudSharpHandler, this);

    _subCornerPointsLessSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_sharp", 2, &LaserOdometry::laserCloudLessSharpHandler, this);

    _subSurfPointsFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_flat", 2, &LaserOdometry::laserCloudFlatHandler, this);

    _subSurfPointsLessFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_flat", 2, &LaserOdometry::laserCloudLessFlatHandler, this);

    _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_2", 2, &LaserOdometry::laserCloudFullResHandler, this);

    _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>
      ("/imu_trans", 5, &LaserOdometry::imuTransHandler, this);

    return true;
  }

  void LaserOdometry::reset()
  {
    _newCornerPointsSharp = false;
    _newCornerPointsLessSharp = false;
    _newSurfPointsFlat = false;
    _newSurfPointsLessFlat = false;
    _newLaserCloudFullRes = false;
    _newImuTrans = false;
  }

  void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg)
  {
    _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp;  // 记录时间戳

    cornerPointsSharp()->clear(); // 清除之前的点云
    pcl::fromROSMsg(*cornerPointsSharpMsg, *cornerPointsSharp()); // 把输入从ros点云message转为pcl点云，存入成员变量
    std::vector<int> indices; // 清除NaN的方法使用的辅助容器：记录非NaN点的索引，以便构造输出
    pcl::removeNaNFromPointCloud(*cornerPointsSharp(), *cornerPointsSharp(), indices);  // 从点云中移除NaN的无效点
    _newCornerPointsSharp = true; // 标记“接收到新的锋锐角点点云”
  }



  void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg)
  {
    _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;  // 记录时间戳

    cornerPointsLessSharp()->clear(); // 清除之前的点云
    pcl::fromROSMsg(*cornerPointsLessSharpMsg, *cornerPointsLessSharp()); // 把输入从ros点云message转为pcl点云，存入成员变量
    std::vector<int> indices; // 清除NaN的方法使用的辅助容器：记录非NaN点的索引，以便构造输出
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp(), *cornerPointsLessSharp(), indices);  // 从点云中移除NaN的无效点
    _newCornerPointsLessSharp = true; // 标记“接收到新的角点点云”
  }



  void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg)
  {
    _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;  // 记录时间戳

    surfPointsFlat()->clear(); // 清除之前的点云
    pcl::fromROSMsg(*surfPointsFlatMsg, *surfPointsFlat()); // 把输入从ros点云message转为pcl点云，存入成员变量
    std::vector<int> indices; // 清除NaN的方法使用的辅助容器：记录非NaN点的索引，以便构造输出
    pcl::removeNaNFromPointCloud(*surfPointsFlat(), *surfPointsFlat(), indices);  // 从点云中移除NaN的无效点
    _newSurfPointsFlat = true; // 标记“接收到新的平坦平面点点云”
  }



  void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg)
  {
    _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;  // 记录时间戳

    surfPointsLessFlat()->clear(); // 清除之前的点云
    pcl::fromROSMsg(*surfPointsLessFlatMsg, *surfPointsLessFlat()); // 把输入从ros点云message转为pcl点云，存入成员变量
    std::vector<int> indices; // 清除NaN的方法使用的辅助容器：记录非NaN点的索引，以便构造输出
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat(), *surfPointsLessFlat(), indices);  // 从点云中移除NaN的无效点
    _newSurfPointsLessFlat = true; // 标记“接收到新的平面点点云”
  }



  void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
  {
    _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;  // 记录时间戳

    laserCloud()->clear(); // 清除之前的点云
    pcl::fromROSMsg(*laserCloudFullResMsg, *laserCloud()); // 把输入从ros点云message转为pcl点云，存入成员变量
    std::vector<int> indices; // 清除NaN的方法使用的辅助容器：记录非NaN点的索引，以便构造输出
    pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);  // 从点云中移除NaN的无效点
    _newLaserCloudFullRes = true; // 标记“接收到新的全分辨率点云”
  }



  void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
  {
    _timeImuTrans = imuTransMsg->header.stamp;  // 记录时间戳

    pcl::PointCloud<pcl::PointXYZ> imuTrans;  // 根据scan registration模块的定义，sweep中的IMU位姿变换信息以一定格式存放在一个点云中
    pcl::fromROSMsg(*imuTransMsg, imuTrans);  // 把输入从ros点云message转为pcl点云，存入新建的对象
    updateIMU(imuTrans);  // 更新IMU信息
    _newImuTrans = true;  // 标记“接收到新的IMU信息”
  }


  void LaserOdometry::spin()
  {
    ros::Rate rate(100);  // ros::Rate帮助循环以指定的速率运行（原则上应当比数据产生频率更快？）
    bool status = ros::ok(); // ros是否还在继续运行；一旦ros::shutdown()被调用，变为false

    // loop until shutdown 循环直到关闭
    while (status)
    {
      ros::spinOnce();  // 执行一次回调(此后真正收到并可以处理订阅的message)

      // try processing new data 检查是否接收到新数据，并尝试处理
      process();

      status = ros::ok(); // 检查ros是否还在继续运行
      rate.sleep(); // 本次循环剩余时间里休眠(sleep还有一个判断循环速度是否满足设定的返回，这里没用到)
    }
  }


  bool LaserOdometry::hasNewData()
  {
    /** 只有下列条件全部满足才会返回true(开始处理)：
     * 1. 全分辨率点云、每种类型的特征点云、IMU位姿都有接收到的新数据(里程计算法需要数据种类齐全)；
     * 2. 锋锐角点云、角点云、平坦平面点云、全分辨率点云、IMU位姿五者和平面点云的时间戳间的差分别都在0.005秒内(尽可能保证所处理的数据属于同一时刻)。
     */ 
    return _newCornerPointsSharp && _newCornerPointsLessSharp && _newSurfPointsFlat &&
      _newSurfPointsLessFlat && _newLaserCloudFullRes && _newImuTrans &&
      fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeImuTrans - _timeSurfPointsLessFlat).toSec()) < 0.005;
  }



  void LaserOdometry::process()
  {
    // 若没有收到新数据，则本次process什么也不做
    if (!hasNewData())
      return;// waiting for new data to arrive...

    // 若有新数据
    reset();  // reset flags, etc. 重置接收新数据flag以便处理完后等待时再次使用
    BasicLaserOdometry::process();  // 调用基础方法处理新数据
    publishResult();  // 把处理结果发布出去
  }


  void LaserOdometry::publishResult()
  {
    // publish odometry transformations 里程计位姿变换
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(),
                                                                               -transformSum().rot_x.rad(),
                                                                               -transformSum().rot_y.rad());  // 从RPY欧拉角创建四元数message

    // 发布_laserOdometryMsg
    _laserOdometryMsg.header.stamp            = _timeSurfPointsLessFlat;  // 时间戳 = 平面点云的时间戳(hasNewData中的判断参照)
    _laserOdometryMsg.pose.pose.orientation.x = -geoQuat.y;
    _laserOdometryMsg.pose.pose.orientation.y = -geoQuat.z;
    _laserOdometryMsg.pose.pose.orientation.z = geoQuat.x;
    _laserOdometryMsg.pose.pose.orientation.w = geoQuat.w;                // 四元数的实部
    _laserOdometryMsg.pose.pose.position.x    = transformSum().pos.x();   // 累积位移：X
    _laserOdometryMsg.pose.pose.position.y    = transformSum().pos.y();   // 累积位移：Y
    _laserOdometryMsg.pose.pose.position.z    = transformSum().pos.z();   // 累积位移：Z
    _pubLaserOdometry.publish(_laserOdometryMsg);

    // 广播_laserOdometryTrans
    _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat; // 时间戳
    _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));  // 从四元数设置位姿变换的旋转部分
    _laserOdometryTrans.setOrigin(tf::Vector3(transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z())); // 设置位姿变换的平移部分
    _tfBroadcaster.sendTransform(_laserOdometryTrans);

    // publish cloud results according to the input output ratio 只有输入输出帧数比<2 或 已处理帧数%输入输出帧数比=1(即有1帧初始化帧未处理) 才视为处理帧处理正常，可以发布
    if (_ioRatio < 2 || frameCount() % _ioRatio == 1)
    {
      ros::Time sweepTime = _timeSurfPointsLessFlat; // 时间戳
      publishCloudMsg(_pubLaserCloudCornerLast, *lastCornerCloud(), sweepTime, "/camera");  // 本次里程计使用的角点：发布给建图使用
      publishCloudMsg(_pubLaserCloudSurfLast, *lastSurfaceCloud(), sweepTime, "/camera");  // 本次里程计使用的平面点：发布给建图使用

      transformToEnd(laserCloud());  // transform full resolution cloud to sweep end before sending it 全分辨率点云发出去前先投影到sweep结束时(下个sweep开始时)
      publishCloudMsg(_pubLaserCloudFullRes, *laserCloud(), sweepTime, "/camera");  // 本次里程计总结出的全分辨率点云：发布给建图使用
    }
  }

} // end namespace loam
