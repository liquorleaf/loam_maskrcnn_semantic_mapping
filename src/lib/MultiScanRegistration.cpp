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

#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound); // 线性插值因子 = (扫描环个数-1) / (竖直角范围大小) = 扫描环间隔数 / 竖直角范围大小 = 1 / 相邻两条扫描线间夹角
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}






MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper)
    : _scanMapper(scanMapper)
{};



bool MultiScanRegistration::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
  RegistrationParams config;  // 用于存放从参数服务器接收的扫描参数
  if (!setupROS(node, privateNode, config)) // 若有异常则启动失败
    return false;

  configure(config);  // 配置扫描参数
  return true;
}

bool MultiScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out)
{
  // 接收扫描参数，若扫描参数有不合法则启动失败；配置订阅/发布
  if (!ScanRegistration::setupROS(node, privateNode, config_out))
    return false;

  // fetch scan mapping params
  std::string lidarName;  // 存储字符串参数(lidar型号)

  // 根据“lidar型号”或“直接指定的参数”配置MultiScanMapper；若lidar型号不支持，或指定参数中“下限>=上限”或“扫描环数目<2”，则启动失败
  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
    // 若无扫描周期参数输入，则默认设置成0.1秒，并通告
    if (!privateNode.hasParam("scanPeriod")) {
      config_out.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", config_out.scanPeriod);
    }
  } else {
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }

  // subscribe to input cloud topic 订阅点云输入topic，用handleCloudMessage处理
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_points", 2, &MultiScanRegistration::handleCloudMessage, this);
//  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
//        ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);
  return true;
}



void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  // 根据设置的系统启动延迟，延迟启动；丢弃_systemDelay个点云数据
  if (_systemDelay > 0) 
  {
    --_systemDelay;
    return;
  }

  // fetch new input cloud 把新输入的点云从ros message转换为pcl点云处理
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));  // 点云时间戳从ros message获取
}



void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size(); // 点云总点数

  // determine scan start and end orientations 扫描起始和终止的方向 = arctan(y/x)，加负号是因为velodyne激光雷达顺时针旋转，而处理程序以逆时针为正方向
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  /** 标准库atan2：结果范围[-pi,+pi]，y和x的符号决定了结果的符号
   * 终止方向+2pi，使得范围变为[+pi,+3pi]，从而终止方向角度>=起始方向角度，便于做差
   * 此后，判断终止方向角度-起始方向角度：
   * 若大于3pi，则终止方向-2pi
   * 若小于pi，则终止方向+2pi
   * 原本差值范围是[0,+4pi]，此举可控制差值范围在[+pi,+3pi]之间
   */
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZI point;
  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());  // 点云向量长度变为和扫面线数相同
  // clear all scanline points 清除原点云，准备存储新点云
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto&&v) {v.clear(); }); // for_each：对前两个参数决定的范围内的每个元素执行第三个参数指向的函数

  // extract valid points from input cloud 提取输入点云中的有效点
  for (int i = 0; i < cloudSize; i++) { // i：对点云中每个点各一次
    // 原本velodyne：X前Y左Z上
    // 修改为项目内部统一的X左Y上Z前
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points 若XYZ坐标中有一个INF，则视为无效点
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points 若点和原点几乎重合，则视为无效点
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.6) {
      continue;
    }

    // calculate vertical point angle and scan ID 计算点的竖直角和扫描环ID
    // 竖直角：根据lidar文档计算
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));  // 这个公式是根据Y上X左Z前的坐标系计算的
    // 扫描ID：使用scanMapper从竖直角计算；若结果scanID小于0或大于(扫描环数-1)，则视为无效点
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle 计算点的水平角
    float ori = -std::atan2(point.x, point.z);  // 这个公式是根据Y上X左Z前的坐标系计算的；加负号是因为velodyne激光雷达顺时针旋转，而处理程序以逆时针为正方向
    // 各种范围条件：保证角度在[startOri,endOri]范围内或超出该范围但较小，使得relTime可以用(距起始方向夹角)/(终止和起始方向夹角)的比例式计算
    if (!halfPassed) {
      // 确保前半圈 -pi/2 <= ori - startOri <= +(3/2)pi
      if (ori < startOri - M_PI / 2) {
        // 若当前点水平角和起始方向比太小，则+2pi
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) { // 根据https://zhuanlan.zhihu.com/p/263090394，这里可能有错
        // 若当前点水平角和起始方向比太大，则-2pi
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;  // 前半圈和起始方向对比修正；超过半圈后，剩下的和终止方向对比修正
      }
    } else {
      ori += 2 * M_PI;
      // 确保后半圈 -(3/2)pi <= ori - endOri <= +pi/2
      if (ori < endOri - M_PI * 3 / 2) {
        // 若当前点水平角和终止方向比太小，则+2pi
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        // 若当前点水平角和终止方向比太大，则-2pi
        ori -= 2 * M_PI;
      }
    }

    /** calculate relative scan time based on point orientation 计算相对本次scan开始时刻的相对时间
     * 相对时间 = 扫描周期 * (方向角-起始方向角)/(终止方向角-起始方向角)
     * 这里相对时间的算法限制了一个扫描周期内lidar不应该转动超过1圈，否则(endOri - startOri)不再能表示一个扫描周期对应的角度
     */
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri); // scanPeriod = 0.1，再乘[-0.5,1.5]的数，结果[-0.05,0.15]
    point.intensity = scanID + relTime; // 整数部分是scanID，小数部分是相对时间relTime，这样intensity可以按照scan先后和scan内时间先后给点排序

    projectPointToStartOfSweep(point, relTime); // 若有IMU数据可用，使用IMU数据消除非线性运动位置漂移畸变，并把点投影到当前sweep开始时的IMU坐标系内；若无IMU可用则不执行，强行按照匀速运动使用里程计做估计

    _laserCloudScans[scanID].push_back(point);  // 按scanID把点加入相应扫描环的点云中
  } // END for (points in the input cloud)

  // 处理这个scan：提取特征点，更新IMU位姿信息；并把结果全分辨率点云、特征点云、IMU位姿publish出去
  processScanlines(scanTime, _laserCloudScans);
  publishResult();
}

} // end namespace loam
