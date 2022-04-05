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
#include "CircularBuffer.h"
#include "time_utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace loam
{

/** IMU state data. 相比IMUState，没有偏航角yaw和三个平移状态(位置、速度、加速度) */
typedef struct IMUState2
{
   /** The time of the measurement leading to this state (in seconds). */
   Time stamp;

   /** The current roll angle. */
   Angle roll;

   /** The current pitch angle. */
   Angle pitch;

   /** \brief Interpolate between two IMU states.
    * 两IMU状态间线性插值；离start状态的值和end状态的值的距离比是ratio:(1-ratio)
    * @param start the first IMU state
    * @param end the second IMU state
    * @param ratio the interpolation ratio
    * @param result the target IMU state for storing the interpolation result
    */
   static void interpolate(const IMUState2& start,
                           const IMUState2& end,
                           const float& ratio,
                           IMUState2& result)
   {
      float invRatio = 1 - ratio;
      // 滚转角和俯仰角：直接线性插值，离前一状态的值和后一状态的值的距离比是ratio:(1-ratio)
      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
   };
} IMUState2;

class BasicLaserMapping
{
public:
   /** 构造函数。默认：扫描周期=0.1，优化最大迭代次数=10。其余默认详见BasicLaserMapping类的声明；初始化帧计数器、建立包含_laserCloudNum个子cube点云的点云向量，设置下采样滤波器尺寸参数。 */
   explicit BasicLaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief Try to process buffered data. */
   bool process(Time const& laserOdometryTime);
   /** 把一个新状态添加到_imuHistory的buffer中 */
   void updateIMU(IMUState2 const& newState);
   /** 更新_transformSum */
   void updateOdometry(double pitch, double yaw, double roll, double x, double y, double z);
   /** 更新_transformSum */
   void updateOdometry(Twist const& twist);

   /** 取得或设置各private成员的基本函数 */
   
   auto& laserCloud() { return *_laserCloudFullRes; }
   auto& laserCloudCornerLast() { return *_laserCloudCornerLast; }
   auto& laserCloudSurfLast() { return *_laserCloudSurfLast; }

   void setScanPeriod(float val) { _scanPeriod = val; }
   void setMaxIterations(size_t val) { _maxIterations = val; }
   void setDeltaTAbort(float val) { _deltaTAbort = val; }
   void setDeltaRAbort(float val) { _deltaRAbort = val; }

   auto& downSizeFilterCorner() { return _downSizeFilterCorner; }
   auto& downSizeFilterSurf() { return _downSizeFilterSurf; }
   auto& downSizeFilterMap() { return _downSizeFilterMap; }

   auto frameCount()    const { return _frameCount; }
   auto scanPeriod()    const { return _scanPeriod; }
   auto maxIterations() const { return _maxIterations; }
   auto deltaTAbort()   const { return _deltaTAbort; }
   auto deltaRAbort()   const { return _deltaRAbort; }

   auto const& transformAftMapped()   const { return _transformAftMapped; }
   auto const& transformBefMapped()   const { return _transformBefMapped; }
   auto const& laserCloudSurroundDS() const { return *_laserCloudSurroundDS; }

   bool hasFreshMap() const { return _downsizedMapCreated; }

private:
   /** Run an optimization. 配准特征，优化位姿_transformTobeMapped */
   void optimizeTransformTobeMapped();

   /** 根据上次位姿在建图前后被优化导致的变化，对本次建图前里程计位姿做相同变化，作为全局优化后位姿的初始猜测 */
   void transformAssociateToMap();
   /** 用本次里程计和全局优化结果更新_transformBefMapped和_transformAftMapped，供下次全局优化计算初始猜测使用；若有IMU数据，则用其略微修正pitch和roll，再更新 */
   void transformUpdate();
   /** 用_transformTobeMapped把pi变换到世界坐标系，存放在po中 */
   void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
   /** 用_transformTobeMapped把pi从世界坐标系变换到lidar坐标系，存放在po中 */
   void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
   /** 用优化后的_transformTobeMapped把本次sweep提供的全分辨率点云转换到世界坐标系中 */
   void transformFullResToMap();
   /** 建立并下采样周围特征点云，得到新的_laserCloudSurroundDS，若执行了建立则返回true */
   bool createDownsizedMap();

   // private:
   /** 把三维索引转化成一维线性索引：i是宽度(X)，j是高度(Y)，k是深度(Z) */
   size_t toIndex(int i, int j, int k) const
   { return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k; }

private:
   Time _laserOdometryTime;

   float _scanPeriod;          ///< time per scan 扫描周期，默认0.1
   const int _stackFrameNum;  ///< 默认1，即每帧都处理；每隔(_stackFrameNum-1)帧处理1帧，中间(_stackFrameNum-1)帧被跳过，但第1帧一定处理
   const int _mapFrameNum;  ///< 默认5；每隔(_mapFrameNum-1)帧建图1帧周围点云，中间(_mapFrameNum-1)帧被跳过，但第1帧一定建图周围点云
   long _frameCount;  ///< 帧计数，初始化为_stackFrameNum-1
   long _mapFrameCount; ///< 地图帧计数，初始化为_mapFrameNum-1

   size_t _maxIterations;  ///< maximum number of iterations 优化最大迭代次数，默认10
   float _deltaTAbort;     ///< optimization abort threshold for deltaT
   float _deltaRAbort;     ///< optimization abort threshold for deltaR

   int _laserCloudCenWidth;   ///< 地图点云大立方体中，中心所属的子cube的宽(X)方向索引，默认10
   int _laserCloudCenHeight;  ///< 地图点云大立方体中，中心所属的子cube的宽(X)方向索引，默认5
   int _laserCloudCenDepth;   ///< 地图点云大立方体中，中心所属的子cube的宽(X)方向索引，默认10
   const size_t _laserCloudWidth;   ///< 地图点云大立方体中，分割成的子cube沿宽(X)方向个数，默认21个
   const size_t _laserCloudHeight;  ///< 地图点云大立方体中，分割成的子cube沿高(Y)方向个数，默认11个
   const size_t _laserCloudDepth;   ///< 地图点云大立方体中，分割成的子cube沿深(Z)方向个数，默认21个
   const size_t _laserCloudNum;     ///< 地图点云大立方体中，分割成的子cube总个数，初始化为_laserCloudWidth * _laserCloudHeight * _laserCloudDepth

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;   ///< last corner points cloud  激光里程计在本次sweep内使用的角点点云(sweep结束时的lidar坐标系)
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;     ///< last surface points cloud  激光里程计在本次sweep内使用的平面点点云(sweep结束时的lidar坐标系)
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud  本次sweep的全分辨率点云(sweep结束时的lidar坐标系)，最后转换到世界坐标系发布；所有发布的拼接起来即可得到完整地图

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;    ///< 激光里程计在本次sweep内使用的角点点云(lidar坐标系)
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;      ///< 激光里程计在本次sweep内使用的平面点点云(lidar坐标系)
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled 下采样后的激光里程计在本次sweep内使用的角点点云(lidar坐标系)(被配准)
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled 下采样后的激光里程计在本次sweep内使用的平面点点云(lidar坐标系)(被配准)

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround;         ///< 周围所有子cube的特征点云
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;       ///< down sampled 下采样后的周围所有子cube的特征点云
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap;    ///< 周围的视野内子cube的角点点云(配准用特征地图)
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap;      ///< 周围的视野内子cube的平面点点云(配准用特征地图)

   pcl::PointCloud<pcl::PointXYZI> _laserCloudOri;    // LM算法中已匹配特征点
   pcl::PointCloud<pcl::PointXYZI> _coeffSel;         // LM算法中已匹配特征点的偏导数和距离

   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;    ///< 角点点云向量(特征地图)
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;      ///< 平面点点云向量(特征地图)
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled 下采样后的角点点云向量(特征地图)
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled 下采样后的平面点点云向量(特征地图)

   std::vector<size_t> _laserCloudValidInd;     ///< 当前以lidar为中心的周围5*5*5个子cube中在视野内的cube的全局一维索引
   std::vector<size_t> _laserCloudSurroundInd;  ///< 当前以lidar为中心的周围5*5*5个子cube的全局一维索引

   ///< 各种位姿(原代码中是一行，即一个Twist)
   Twist _transformSum;          ///< 建图前里程计给出的累积位姿
   Twist _transformIncre;        ///< 建图前里程计位移增量(的负值) = 里程计上次建图前位置 - 里程计这次建图前位置
   Twist _transformTobeMapped;   ///< 本次建图中要优化的位姿
   Twist _transformBefMapped;    ///< 上次全局优化前里程计给出的累积位姿
   Twist _transformAftMapped;    ///< 上次全局优化后的累积位姿

   CircularBuffer<IMUState2> _imuHistory;    ///< history of IMU states 建图组件和scan registration组件一样，直接订阅IMU原始数据，并存放在一个CircularBuffer中

   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;   ///< voxel filter for down sizing corner clouds   下采样角点点云的体素栅格滤波器，默认leaf size = (0.2,0.2,0.2)
   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;     ///< voxel filter for down sizing surface clouds  下采样平面点点云的体素栅格滤波器，默认leaf size = (0.4,0.4,0.4)
   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMap;      ///< voxel filter for down sizing accumulated map 下采样累积地图的体素栅格滤波器（没有用到）

   bool _downsizedMapCreated = false;  ///< 是否创建了下采样版周围点云地图的flag
};

} // end namespace loam





