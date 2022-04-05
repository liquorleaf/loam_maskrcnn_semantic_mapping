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

#include "loam_velodyne/ScanRegistration.h"
#include "math_utils.h"

#include <tf/transform_datatypes.h>


namespace loam {



bool ScanRegistration::parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out) 
{
  bool success = true;  // 是否通过的标记
  int iParam = 0;       // 临时存放整型参数
  float fParam = 0;     // 临时存放浮点型参数

  if (nh.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {  // 扫描周期不得小于或等于0
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      success = false;
    } else {
        config_out.scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (nh.getParam("imuHistorySize", iParam)) {
    if (iParam < 1) { // IMU历史缓存器的尺寸必须为正整数
      ROS_ERROR("Invalid imuHistorySize parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.imuHistorySize = iParam;
      ROS_INFO("Set imuHistorySize: %d", iParam);
    }
  }

  if (nh.getParam("featureRegions", iParam)) {
    if (iParam < 1) { // 划分特征region的数目必须为正整数
      ROS_ERROR("Invalid featureRegions parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.nFeatureRegions = iParam;
      ROS_INFO("Set nFeatureRegions: %d", iParam);
    }
  }

  if (nh.getParam("curvatureRegion", iParam)) {
    if (iParam < 1) { // 曲率计算邻域每侧的大小必须为正整数
      ROS_ERROR("Invalid curvatureRegion parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.curvatureRegion = iParam;
      ROS_INFO("Set curvatureRegion: +/- %d", iParam);
    }
  }

  if (nh.getParam("maxCornerSharp", iParam)) {
    if (iParam < 1) { // 每region最大锋锐角点数必须为正整数
      ROS_ERROR("Invalid maxCornerSharp parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.maxCornerSharp = iParam;
        config_out.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("Set maxCornerSharp / less sharp: %d / %d", iParam, config_out.maxCornerLessSharp);
    }
  }

  if (nh.getParam("maxCornerLessSharp", iParam)) {
    if (iParam < config_out.maxCornerSharp) { // 每region最大角点数必须至少为最大锋锐角点数的十倍
      ROS_ERROR("Invalid maxCornerLessSharp parameter: %d (expected >= %d)", iParam, config_out.maxCornerSharp);
      success = false;
    } else {
        config_out.maxCornerLessSharp = iParam;
      ROS_INFO("Set maxCornerLessSharp: %d", iParam);
    }
  }

  if (nh.getParam("maxSurfaceFlat", iParam)) {
    if (iParam < 1) { // 每region最大平坦平面点数必须为正整数
      ROS_ERROR("Invalid maxSurfaceFlat parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.maxSurfaceFlat = iParam;
      ROS_INFO("Set maxSurfaceFlat: %d", iParam);
    }
  }

  if (nh.getParam("surfaceCurvatureThreshold", fParam)) {
    if (fParam < 0.001) { // 划分角点和平面点的曲率阈值必须至少为0.001
      ROS_ERROR("Invalid surfaceCurvatureThreshold parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
        config_out.surfaceCurvatureThreshold = fParam;
      ROS_INFO("Set surfaceCurvatureThreshold: %g", fParam);
    }
  }

  if (nh.getParam("lessFlatFilterSize", fParam)) {
    if (fParam < 0.001) { // 体素栅格滤波器尺寸参数必须至少为0.001
      ROS_ERROR("Invalid lessFlatFilterSize parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
        config_out.lessFlatFilterSize = fParam;
      ROS_INFO("Set lessFlatFilterSize: %g", fParam);
    }
  }

  return success;
}

bool ScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out)
{
  // 如果参数不合法，则设置失败
  if (!parseParams(privateNode, config_out))
    return false;

  // subscribe to IMU topic 订阅：IMU数据，队列最多排队message数=50(超出的舍弃)，使用ScanRegistration::handleIMUMessage处理本对象的message
  //_subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data1", 50, &ScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics 发布：完整点云、四种特征点云、IMU位姿变换；点云队列最多2个，IMU信息队列最多5个
  _pubLaserCloud            = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp     = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat        = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat    = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans              = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  return true;
}



void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation); // sensor_msgs::Imu用四元数存储方向(坐标系旋转)信息，将其提取出来转换成tf::Quaternion
  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);  // 使用tf的方法把四元数变为旋转矩阵，然后使用旋转矩阵计算三个欧拉角

  /** 获取IMU参考系下的运动加速度
   * 首先，减去重力加速度的影响：(旋转顺序：yaw,pitch,roll)(质量块受重力初始指向-Z方向，使传感器报告沿Z方向的加速度，要抵消的话需要加上一个重力加速度)
   * (1) 对X分量：yaw后X轴仍水平(与-Z夹角不变)，pitch时X轴动，roll时X轴是转轴保持不动——只考虑pitch即可(sin(pitch)投影到X轴上)
   * 同理，由于roll时X轴不动，故最终X轴与重力加速度夹角一定是是(90度-pitch)，cos(pitch)投影到与X轴垂直方向，即YZ平面上，
   * 且无roll时重力加速度的YZ平面分量与Y轴垂直，沿-Z方向，roll使得Z偏离roll前+Z的角度是roll，因此：
   * (2) 对Y分量：yaw后Y轴仍水平(与-Z夹角不变)，pitch时Y轴是转轴保持不动，roll时Y轴动——cos(pitch)后sin(roll)得到Y分量
   * (3) 对Z分量：yaw时Z轴是转轴保持不动，pitch和roll时Z轴动——cos(pitch)后cos(roll)得到Z分量
   * 另一种更系统的理解:(https://blog.csdn.net/u012700322/article/details/103615287)，用3个三维绕轴旋转矩阵推导总旋转矩阵
   * 
   * 随后，轴的顺序变化，适应loam_velodyne中定义的标准：
   * 此处原本(按KITTI标准的话)：
   * X前 绕X是滚转角roll  0代表水平 左边向上转为正 范围正负pi
   * Y左 绕Y是俯仰角pitch 0代表水平 前边向下转为正 范围正负pi/2
   * Z上 绕Z是偏航角yaw   0代表朝向正东 逆时针转为正 范围正负pi
   * 之后：
   * Z前 滚转角
   * X左 俯仰角
   * Y上 偏航角
   * (参考：https://blog.csdn.net/zhuogoulu4520/article/details/102254664)
   */
  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);  // 因为滚转角以左边(+Y)向上为正，故为减
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);  // 因为滚转角以左边(+Y)向上为正，故为减
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch)             * 9.81);  // 因为俯仰角以前边(+X)向下为正，故为加

  IMUState newState;
  newState.stamp = fromROSTime( imuIn->header.stamp); // 时间戳
  newState.roll = roll; // 欧拉角1
  newState.pitch = pitch; // 欧拉角2
  newState.yaw = yaw; //欧拉角3
  newState.acceleration = acc;  // 加速度

  updateIMUData(acc, newState); // 转到世界坐标系；估算位置、速度；随后将6数据+时间戳构成的IMU状态对象加入缓冲器
}


void ScanRegistration::publishResult()
{
  auto sweepStartTime = toROSTime(sweepStart());  // 获取当前sweep开始时刻的时间戳
  // publish full resolution and feature point clouds 用public成员函数获取各private点云成员，并发布各点云
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, "/camera");

  // publish corresponding IMU transformation information 发布对应的IMU位姿变换信息
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, "/camera");
}

} // end namespace loam
