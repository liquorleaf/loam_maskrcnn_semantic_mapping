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

#ifndef LOAM_SCANREGISTRATION_H
#define LOAM_SCANREGISTRATION_H


#include "common.h"

#include <stdint.h>

#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

#include "BasicScanRegistration.h"


namespace loam
{
  /** \brief Base class for LOAM scan registration implementations.
   *
   * As there exist various sensor devices, producing differently formatted point clouds,
   * specific implementations are needed for each group of sensor devices to achieve an accurate registration.
   * This class provides common configurations, buffering and processing logic.
   */
  class ScanRegistration : protected BasicScanRegistration
  {
  public:

    /** \brief Setup component.
     * 设置ROS结点，配置扫描参数，配置订阅/发布；是虚函数
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
    virtual bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);

    /** \brief Handler method for IMU messages.
     * 用作回调函数。处理IMU传来的message：计算欧拉角；加速度去除重力影响；转到世界坐标系；估算位置、速度；随后将(3欧拉角+位置/速度/加速度+时间戳)构成的IMU状态对象加入缓冲器；是虚函数
     * @param imuIn the new IMU message
     */
    virtual void handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn);

  protected:
    /** \brief Publish the current result via the respective topics. 
     * 通过相应topic发布当前结果。
     * （如果没有IMU数据可用，则发布
     */
    void publishResult();

  private:

    /** \brief Parse node parameter.
    * 从参数服务器获取参数，分析结点用该参数是否合法，若合法则赋给config_out中相应参数变量
    * @param nh the ROS node handle
    * @return true, if all specified parameters are valid, false if at least one specified parameter is invalid
    */
    bool parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out);

  private:
    ros::Subscriber _subImu;                    ///< IMU message subscriber                     订阅：IMU数据
    ros::Publisher _pubLaserCloud;              ///< full resolution cloud message publisher    发布：全分辨率点云
    ros::Publisher _pubCornerPointsSharp;       ///< sharp corner cloud message publisher       发布：锋锐角点云
    ros::Publisher _pubCornerPointsLessSharp;   ///< less sharp corner cloud message publisher  发布：角点云
    ros::Publisher _pubSurfPointsFlat;          ///< flat surface cloud message publisher       发布：平坦平面点云
    ros::Publisher _pubSurfPointsLessFlat;      ///< less flat surface cloud message publisher  发布：平面点云
    ros::Publisher _pubImuTrans;                ///< IMU transformation message publisher       发布：IMU位姿变换
  };

} // end namespace loam


#endif //LOAM_SCANREGISTRATION_H
