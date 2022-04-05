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

#ifndef LOAM_COMMON_H
#define LOAM_COMMON_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "time_utils.h"

// 3个inline的函数：publishCloudMsg用于把点云变为message传给publisher，fromROSTime和toROSTime用于来自标准库的时间和ros::Time相互转换

namespace loam {

//**********publishCloudMsg**********

/** \brief Construct a new point cloud message from the specified information and publish it via the given publisher.
 * 把特定点类型的pcl点云变为ros点云message并送给publisher
 * @tparam PointT the point type
 * @param publisher the publisher instance 通过哪个publisher发布
 * @param cloud the cloud to publish 发布的点云
 * @param stamp the time stamp of the cloud message 发布的点云message的时间戳
 * @param frameID the message frame ID 发布的message的帧ID
 */
template <typename PointT>
inline void publishCloudMsg(ros::Publisher& publisher,
                            const pcl::PointCloud<PointT>& cloud,
                            const ros::Time& stamp,
                            std::string frameID) {
  sensor_msgs::PointCloud2 msg;   // 新ros点云msg对象
  pcl::toROSMsg(cloud, msg);      // 用pcl自带方法把pcl点云变为ros点云message
  msg.header.stamp = stamp;       // 点云的时间戳
  msg.header.frame_id = frameID;  // 生成message的帧ID
  publisher.publish(msg);         // 引用的publisher把message发布出去
}

//**********ros::Time和Time(std::chrono::system_clock::time_point的别名指定)相互转换**********

// ROS time adapters
// 从ros::Time到Time：用默认构造函数创建一个新Time对象；由于ros::Time有 秒 和 纳秒 两部分，故分别提取并转为标准秒和纳秒，相加后加到刚刚的新Time对象上，得到结果
inline Time fromROSTime(ros::Time const& rosTime)
{
  auto epoch = std::chrono::system_clock::time_point();                                           // epoch：纪元; 时代; 时期
  auto since_epoch = std::chrono::seconds(rosTime.sec) + std::chrono::nanoseconds(rosTime.nsec);  // epoch后需要计多少时间
  return epoch + since_epoch;
}

// ROS time adapters
// 从Time到ros::Time：提取时间(第一个函数得到duration值，第二个变秒数值)，然后转换为纳秒，再用ros::Time的从纳秒构造的方法转为ros::Time
inline ros::Time toROSTime(Time const& time_point)
{
  return ros::Time().fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(time_point.time_since_epoch()).count());
}

} // end namespace loam

#endif // LOAM_COMMON_H
