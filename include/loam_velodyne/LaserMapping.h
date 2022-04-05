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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H


#include "BasicLaserMapping.h"
#include "common.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>



namespace loam
{
/** \brief Implementation of the LOAM laser mapping component.
 * 类：LOAM激光建图组件的实现；LaserMapping类的结构和运作方式与LaserOdometry完全类似，
 * 除了一些细节(比如处理特征点时不再区分锋锐和较不锋锐、平坦和较不平坦)、Basic类的内容，和laserOdometryHandler方法的存在
 */
class LaserMapping : public BasicLaserMapping
{
public:
   explicit LaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief Setup component in active mode.
    *
    * @param node the ROS node handle
    * @param privateNode the private ROS node handle
    */
   virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

   /** \brief Handler method for a new last corner cloud.
    *
    * @param cornerPointsLastMsg the new last corner cloud message
    */
   void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg);

   /** \brief Handler method for a new last surface cloud.
    *
    * @param surfacePointsLastMsg the new last surface cloud message
    */
   void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg);

   /** \brief Handler method for a new full resolution cloud.
    *
    * @param laserCloudFullResMsg the new full resolution cloud message
    */
   void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

   /** \brief Handler method for a new laser odometry.
    * 处理激光里程计数据：取得本次里程计估计的全局累积位姿。
    * @param laserOdometry the new laser odometry message
    */
   void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

   /** \brief Handler method for IMU messages.
    * 建图组件和scan registration组件一样，直接订阅IMU原始数据，并存放在一个CircularBuffer中
    * @param imuIn the new IMU message
    */
   void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);

   /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
   void spin();

   /** \brief Try to process buffered data. */
   void process();


protected:
   /** \brief Reset flags, etc. */
   void reset();

   /** \brief Check if all required information for a new processing step is available. */
   bool hasNewData();

   /** \brief Publish the current result via the respective topics.
    * 发布结果。其中全局优化前的里程计位姿也一并发布了。
    */
   void publishResult();

private:
   ros::Time _timeLaserCloudCornerLast;   ///< time of current last corner cloud
   ros::Time _timeLaserCloudSurfLast;     ///< time of current last surface cloud
   ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
   ros::Time _timeLaserOdometry;          ///< time of current laser odometry

   bool _newLaserCloudCornerLast;  ///< flag if a new last corner cloud has been received
   bool _newLaserCloudSurfLast;    ///< flag if a new last surface cloud has been received
   bool _newLaserCloudFullRes;     ///< flag if a new full resolution cloud has been received
   bool _newLaserOdometry;         ///< flag if a new laser odometry has been received


   nav_msgs::Odometry _odomAftMapped;      ///< mapping odometry message 建图后里程计位姿
   tf::StampedTransform _aftMappedTrans;   ///< mapping odometry transformation 建图后里程计位姿变换

    /** 发布建图结果周围点云/全分辨率点云/建图后优化得到的激光里程计在世界坐标系中位姿的publisher或broadcaster */
   ros::Publisher _pubLaserCloudSurround;    ///< map cloud message publisher
   ros::Publisher _pubLaserCloudFullRes;     ///< current full resolution cloud message publisher
   ros::Publisher _pubOdomAftMapped;         ///< mapping odometry publisher
   tf::TransformBroadcaster _tfBroadcaster;  ///< mapping odometry transform broadcaster

   /** 订阅odometry模块发布的点云、特征点云、IMU信息的subscriber */
   ros::Subscriber _subLaserCloudCornerLast;   ///< last corner cloud message subscriber
   ros::Subscriber _subLaserCloudSurfLast;     ///< last surface cloud message subscriber
   ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud message subscriber
   ros::Subscriber _subLaserOdometry;          ///< laser odometry message subscriber
   ros::Subscriber _subImu;                    ///< IMU message subscriber
};

} // end namespace loam

#endif //LOAM_LASERMAPPING_H
