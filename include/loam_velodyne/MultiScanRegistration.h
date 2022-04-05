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

#ifndef LOAM_MULTISCANREGISTRATION_H
#define LOAM_MULTISCANREGISTRATION_H


#include "loam_velodyne/ScanRegistration.h"

#include <sensor_msgs/PointCloud2.h>


namespace loam {



/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 * 一个类：多线竖直角-扫描环ID映射器。实现从“点的竖直方向夹角”到对应的“扫描环(扫描平面)”间的线性映射
 */
class MultiScanMapper {
public:
  /** \brief Construct a new multi scan mapper instance.
   * 构造函数：创建一个新的多线扫描(multi scan)映射器实例，默认相邻扫描环夹角2度(对应线性插值因子0.5)
   * @param lowerBound - the lower vertical bound (degrees) 竖直角下限(度)，默认-15度
   * @param upperBound - the upper vertical bound (degrees) 竖直角上限(度)，默认15度
   * @param nScanRings - the number of scan rings 扫描环个数，默认16
   */
  MultiScanMapper(const float& lowerBound = -15,
                  const float& upperBound = 15,
                  const uint16_t& nScanRings = 16);

  //*****取得映射器初始化参数的方法*****

  const float& getLowerBound() { return _lowerBound; }
  const float& getUpperBound() { return _upperBound; }
  const uint16_t& getNumberOfScanRings() { return _nScanRings; }

  /** \brief Set mapping parameters.
   * 使用输入参数设置3个映射参数
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float& lowerBound,
           const float& upperBound,
           const uint16_t& nScanRings);

  /** \brief Map the specified vertical point angle to its ring ID.
   * 给出一个点的竖直角，返回其对应的扫描环编号。算法：输入弧度转换为角度，减去下限，乘线性插值因子(相当于除以相邻两环夹角，以乘代除运算更快)，四舍五入为整数。
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  int getRingForAngle(const float& angle);

  //*****创建适用于特定传感器的MultiScanMapper的方法*****

  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };


private:
  float _lowerBound;      ///< the vertical angle of the first scan ring 竖直角下限：第一个扫描环的竖直角
  float _upperBound;      ///< the vertical angle of the last scan ring 竖直角上限：最后一个扫描环的竖直角
  uint16_t _nScanRings;   ///< number of scan rings 扫描环的数目
  float _factor;          ///< linear interpolation factor 线性插值因子 = 1 / 相邻两扫描线间夹角
};



/** \brief Class for registering point clouds received from multi-laser lidars.
 * 一个类：配准来自多线激光雷达的点云。从ScanRegistration虚继承而来。
 */
class MultiScanRegistration : virtual public ScanRegistration {
public:
  /** 构造函数：创建时指定某个(或使用默认)MultiScanMapper，以适用于特定传感器 */
  MultiScanRegistration(const MultiScanMapper& scanMapper = MultiScanMapper());

  /** 启动结点：调用setupROS和configure */
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Handler method for input cloud messages.
   * 用作回调函数。系统启动延迟计数；把输入的ros点云msg转为pcl点云，调用process处理
   * @param laserCloudMsg the new input cloud message to process
   */
  void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

private:
  /** \brief Setup component in active mode.
   * 给结点配置各种运行参数
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out) override;

  /** \brief Process a new input cloud.
   * 处理一个新的输入点云(以sweep为单位，实现中 一次sweep = 一次scan)：剔除无效点，计算相对时间，IMU修正，提取特征点并发布，更新IMU位姿并发布(无IMU可用时发布全0，无实际作用)
   * @param laserCloudIn the new input cloud to process
   * @param scanTime the scan (message) timestamp
   */
  void process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime);

private:
  int _systemDelay = 20;             ///< system startup delay counter 系统启动延迟计数器，倒数默认从20开始
  MultiScanMapper _scanMapper;  ///< mapper for mapping vertical point angles to scan ring IDs 多线竖直角-扫描环ID映射器
  std::vector<pcl::PointCloud<pcl::PointXYZI> > _laserCloudScans; // 存储接收的点云的向量，元素为单个扫描环点云
  ros::Subscriber _subLaserCloud;   ///< input cloud message subscriber 订阅：点云message

};

} // end namespace loam


#endif //LOAM_MULTISCANREGISTRATION_H
