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

#ifndef LOAM_TRANSFORMMAINTENANCE_H
#define LOAM_TRANSFORMMAINTENANCE_H


#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "loam_velodyne/BasicTransformMaintenance.h"

namespace loam {

/** \brief Implementation of the LOAM transformation maintenance component.
 * 
 * LOAM变换维护组件的实现：
 * 1.使用laserOdometry输出的odom（高频率）对laserMapping输出的odom（低频率）进行插值。
 * 2.由于mapping结果位姿的频率低于odometry输出的位姿，所以两次mapping结果之间的odometry位姿不能在mapping中使用全局优化，
 * 只能在TransformMaintenance中使用上一次mapping“建图前后旋转偏差”进行修正，用其结果补全两次mapping之间的空缺，使最终位姿输出也达到和odometry一样的高频率。
 * 3.相当于做只有初始猜测，且不更新“上一次全局优化前/后位姿”的mapping。
 * 4.如果应用场景运算性能完全够，可以一帧(一里程计)一次建图优化。如果这样, transformMaintenance就没有存在意义了。
 */
class TransformMaintenance: public BasicTransformMaintenance {
public:
  /** 构造函数：设置帧id和child帧id */
  TransformMaintenance();

  /** \brief Setup component.
   * 启动结点：设置订阅/发布
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Handler method for laser odometry messages.
   * 处理里程计结果：更新_transformSum，使用上一次mapping“建图前后旋转偏差”修正_transformSum，得到_transformMapped并发布
   * @param laserOdometry the new laser odometry
   */
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

  /** \brief Handler method for mapping odometry messages.
   * 处理建图组件全局优化结果：更新_transformAftMapped和_transformBefMapped
   * @param odomAftMapped the new mapping odometry
   */
  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);

private:
  nav_msgs::Odometry _laserOdometry2;         ///< latest integrated laser odometry message 
  tf::StampedTransform _laserOdometryTrans2;  ///< latest integrated laser odometry transformation 

  ros::Publisher _pubLaserOdometry2;          ///< integrated laser odometry publisher
  tf::TransformBroadcaster _tfBroadcaster2;   ///< integrated laser odometry transformation broadcaster

  ros::Subscriber _subLaserOdometry;    ///< (high frequency) laser odometry subscriber
  ros::Subscriber _subOdomAftMapped;    ///< (low frequency) mapping odometry subscriber
};

} // end namespace loam


#endif //LOAM_TRANSFORMMAINTENANCE_H
