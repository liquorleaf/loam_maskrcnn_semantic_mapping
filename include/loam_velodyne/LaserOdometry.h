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

#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H


#include "Twist.h"
#include "nanoflann_pcl.h"

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "BasicLaserOdometry.h"

namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   * 类：LOAM激光里程计组件的实现
   */
  class LaserOdometry : public BasicLaserOdometry
  {
  public:
    /** 构造函数：默认扫描周期0.1，输入输出帧数比2，最大迭代次数25。 */
    explicit LaserOdometry(float scanPeriod = 0.1, uint16_t ioRatio = 2, size_t maxIterations = 25);

    /** \brief Setup component.
     * 启动里程计组件，判断里程计参数是否合法；是虚函数
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
    virtual bool setup(ros::NodeHandle& node,
      ros::NodeHandle& privateNode);

    /** \brief Handler method for a new sharp corner cloud.
     * 用作回调函数。处理锋锐角点云：清除NaN无效点，标记“接收到新的该类型点云”
     * @param cornerPointsSharpMsg the new sharp corner cloud message
     */
    void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg);

    /** \brief Handler method for a new less sharp corner cloud.
     * 用作回调函数。处理角点云：清除NaN无效点，标记“接收到新的该类型点云”
     * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
     */
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg);

    /** \brief Handler method for a new flat surface cloud.
     * 用作回调函数。处理平坦平面点云：清除NaN无效点，标记“接收到新的该类型点云”
     * @param surfPointsFlatMsg the new flat surface cloud message
     */
    void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg);

    /** \brief Handler method for a new less flat surface cloud.
     * 用作回调函数。处理平面点云：清除NaN无效点，标记“接收到新的该类型点云”
     * @param surfPointsLessFlatMsg the new less flat surface cloud message
     */
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg);

    /** \brief Handler method for a new full resolution cloud.
     * 用作回调函数。处理全分辨率点云：清除NaN无效点，标记“接收到新的该类型点云”
     * @param laserCloudFullResMsg the new full resolution cloud message
     */
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

    /** \brief Handler method for a new IMU transformation information.
     * 用作回调函数。处理IMU位姿变换信息：更新一个sweep的IMU信息(起始时欧拉角、结束时欧拉角、sweep内非线性运动位置漂移、sweep内速度变化)；
     * 之后IMU信息将被用于修正非线性(即非匀速直线)运动产生的点云运动失真；
     * 若无IMU可用，此处接收到的是全0的message，参与各种运算时无影响
     * @param laserCloudFullResMsg the new IMU transformation information message
     */
    void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg);


    /** \brief Process incoming messages in a loop until shutdown (used in active mode).
     * (loam自编spin函数)活跃模式下，循环尝试接收并处理message，直到程序关闭
     */
    void spin();

    /** \brief Try to process buffered data.
     * 尝试用里程计算法处理接收到的新数据；若没有收到新数据，则本次process什么也不做
     */
    void process();

  protected:
    /** \brief Reset flags, etc.
     * 重置是否接收到新的点云/某种特征点云/IMU位姿的flag为false
     */
    void reset();

    /** \brief Check if all required information for a new processing step is available.
     * 检查执行一次里程计算法所需的数据是否齐全并满足要求(时间戳基本相同)
     */
    bool hasNewData();

    /** \brief Publish the current result via the respective topics.
     * 
     */
    void publishResult();

  private:
    uint16_t _ioRatio;       ///< ratio of input to output frames 输入输出帧数比

    /** 当前点云/某种特征点云/IMU位姿的时间点 */
    ros::Time _timeCornerPointsSharp;      ///< time of current sharp corner cloud
    ros::Time _timeCornerPointsLessSharp;  ///< time of current less sharp corner cloud
    ros::Time _timeSurfPointsFlat;         ///< time of current flat surface cloud
    ros::Time _timeSurfPointsLessFlat;     ///< time of current less flat surface cloud
    ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
    ros::Time _timeImuTrans;               ///< time of current IMU transformation information

    /** 是否接收到新的点云/某种特征点云/IMU位姿的flag */
    bool _newCornerPointsSharp;       ///< flag if a new sharp corner cloud has been received
    bool _newCornerPointsLessSharp;   ///< flag if a new less sharp corner cloud has been received
    bool _newSurfPointsFlat;          ///< flag if a new flat surface cloud has been received
    bool _newSurfPointsLessFlat;      ///< flag if a new less flat surface cloud has been received
    bool _newLaserCloudFullRes;       ///< flag if a new full resolution cloud has been received
    bool _newImuTrans;                ///< flag if a new IMU transformation information cloud has been received

    nav_msgs::Odometry _laserOdometryMsg;       ///< laser odometry message 激光里程计位姿message(ros的nav_msgs::Odometry)
    tf::StampedTransform _laserOdometryTrans;   ///< laser odometry transformation 激光里程计位姿变换(ros的tf::StampedTransform)

    /** 发布里程计处理结果点云/特征点云/激光里程计位姿变换的publisher或broadcaster */
    ros::Publisher _pubLaserCloudCornerLast;  ///< last corner cloud message publisher 上一次的角点云publisher
    ros::Publisher _pubLaserCloudSurfLast;    ///< last surface cloud message publisher 上一次的平面点云publisher
    ros::Publisher _pubLaserCloudFullRes;     ///< full resolution cloud message publisher 全分辨率点云publisher
    ros::Publisher _pubLaserOdometry;         ///< laser odometry publisher 激光里程计publisher
    tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster 激光里程计位姿变换broadcaster

    /** 订阅scan registration模块发布的点云、特征点云、IMU信息的subscriber */
    ros::Subscriber _subCornerPointsSharp;      ///< sharp corner cloud message subscriber
    ros::Subscriber _subCornerPointsLessSharp;  ///< less sharp corner cloud message subscriber
    ros::Subscriber _subSurfPointsFlat;         ///< flat surface cloud message subscriber
    ros::Subscriber _subSurfPointsLessFlat;     ///< less flat surface cloud message subscriber
    ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud message subscriber
    ros::Subscriber _subImuTrans;               ///< IMU transformation information message subscriber
  };

} // end namespace loam

#endif //LOAM_LASERODOMETRY_H
