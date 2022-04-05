#pragma once
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

#include "Twist.h"

namespace loam
{

/** \brief Implementation of the LOAM transformation maintenance component.
 *
 */
class BasicTransformMaintenance
{
public:
   /** 使用从odometry接收到的message更新_transformSum */
   void updateOdometry(double pitch, double yaw, double roll, double x, double y, double z);
   /** 使用2个Twist对象更新_transformAftMapped和_transformBefMapped */
   void updateMappingTransform(Twist const& transformAftMapped, Twist const& transformBefMapped);
   /** 使用6+6个数值更新_transformAftMapped和_transformBefMapped */
   void updateMappingTransform(double pitch, double yaw, double roll,
      double x, double y, double z,
      double twist_rot_x, double twist_rot_y, double twist_rot_z,
      double twist_pos_x, double twist_pos_y, double twist_pos_z);

   /** 使用“建图算法全局优化前后的位姿差”修正里程计的_transformSum，结果存放在_transformMapped中；
    * 计算参见BasicLaserMapping::transformAssociateToMap
    */
   void transformAssociateToMap();

   // result accessor 提取_transformMapped
   auto const& transformMapped() const { return _transformMapped; }

private:
   // 各种累积位姿，元素顺序是[0]pitch(rx),[1]yaw(ry),[2]roll(rz),[3]x,[4]y,[5]z；使局部坐标转换为世界坐标需要roll,pitch,yaw(即ZXY)，然后平移(x,y,z)
   float _transformSum[6]{};        ///< 某次里程计累积位姿
   float _transformIncre[6]{};      ///< 建图算法全局优化前后位姿的平移差
   float _transformMapped[6]{};     ///< 使用“建图算法全局优化前后的位姿差”修正里程计的_transformSum后得到的结果
   float _transformBefMapped[6]{};  ///< 某次全局优化和建图前mapping组件从里程计处收到的位姿变换
   float _transformAftMapped[6]{};  ///< 某次全局优化和建图后mapping组件得到的已优化位姿变换
};

} // end namespace loam

