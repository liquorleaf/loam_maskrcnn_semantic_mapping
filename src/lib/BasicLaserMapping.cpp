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


#include "loam_velodyne/BasicLaserMapping.h"
#include "loam_velodyne/nanoflann_pcl.h"
#include "math_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam
{

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;


BasicLaserMapping::BasicLaserMapping(const float& scanPeriod, const size_t& maxIterations) :
   _scanPeriod(scanPeriod),
   _stackFrameNum(1),
   _mapFrameNum(5),
   _frameCount(0),
   _mapFrameCount(0),
   _maxIterations(maxIterations),
   _deltaTAbort(0.05),
   _deltaRAbort(0.05),
   _laserCloudCenWidth(10),
   _laserCloudCenHeight(5),
   _laserCloudCenDepth(10),
   _laserCloudWidth(21),
   _laserCloudHeight(11),
   _laserCloudDepth(21),
   _laserCloudNum(_laserCloudWidth * _laserCloudHeight * _laserCloudDepth),
   _laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>())
{
   // initialize frame counter
   _frameCount = _stackFrameNum - 1;
   _mapFrameCount = _mapFrameNum - 1;

   // setup cloud vectors
   _laserCloudCornerArray.resize(_laserCloudNum);
   _laserCloudSurfArray.resize(_laserCloudNum);
   _laserCloudCornerDSArray.resize(_laserCloudNum);
   _laserCloudSurfDSArray.resize(_laserCloudNum);

   for (size_t i = 0; i < _laserCloudNum; i++)
   {
      _laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudCornerDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudSurfDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
   }

   // setup down size filters
   _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
   _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
}


void BasicLaserMapping::transformAssociateToMap()
{
   _transformIncre.pos = _transformBefMapped.pos - _transformSum.pos;   // 全局优化前里程计位移增量(的负值) = 里程计上次全局优化前位置 - 里程计这次全局优化前位置
   rotateYXZ(_transformIncre.pos, -(_transformSum.rot_y), -(_transformSum.rot_x), -(_transformSum.rot_z));  // 位移增量用这次里程计全局优化前旋转角旋转

   // 这次位姿旋转变换的初始猜测：里程计这次欧拉角_transformSum，之后用_transformBefMapped旋转，再用_transformAftMapped转回来
   // 后两项的总旋转量，即上次全局优化后旋转和上次全局优化前旋转的差值，也就是说给“这次的全局优化前旋转”补上了“上次全局优化前后旋转偏差”，得到“这次全局优化后旋转的初始猜测”

   float sbcx = _transformSum.rot_x.sin();
   float cbcx = _transformSum.rot_x.cos();
   float sbcy = _transformSum.rot_y.sin();
   float cbcy = _transformSum.rot_y.cos();
   float sbcz = _transformSum.rot_z.sin();
   float cbcz = _transformSum.rot_z.cos();

   float sblx = _transformBefMapped.rot_x.sin();
   float cblx = _transformBefMapped.rot_x.cos();
   float sbly = _transformBefMapped.rot_y.sin();
   float cbly = _transformBefMapped.rot_y.cos();
   float sblz = _transformBefMapped.rot_z.sin();
   float cblz = _transformBefMapped.rot_z.cos();

   float salx = _transformAftMapped.rot_x.sin();
   float calx = _transformAftMapped.rot_x.cos();
   float saly = _transformAftMapped.rot_y.sin();
   float caly = _transformAftMapped.rot_y.cos();
   float salz = _transformAftMapped.rot_z.sin();
   float calz = _transformAftMapped.rot_z.cos();

   float srx = -sbcx * (salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz)
      - cbcx * sbcy*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                     - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      - cbcx * cbcy*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                     - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx);
   _transformTobeMapped.rot_x = -asin(srx);

   float srycrx = sbcx * (cblx*cblz*(caly*salz - calz * salx*saly)
                          - cblx * sblz*(caly*calz + salx * saly*salz) + calx * saly*sblx)
      - cbcx * cbcy*((caly*calz + salx * saly*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(sbly*sblz + cbly * cblz*sblx) - calx * cblx*cbly*saly)
      + cbcx * sbcy*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly) + calx * cblx*saly*sbly);
   float crycrx = sbcx * (cblx*sblz*(calz*saly - caly * salx*salz)
                          - cblx * cblz*(saly*salz + caly * calz*salx) + calx * caly*sblx)
      + cbcx * cbcy*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz) + calx * caly*cblx*cbly)
      - cbcx * sbcy*((saly*salz + caly * calz*salx)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(cbly*cblz + sblx * sbly*sblz) - calx * caly*cblx*sbly);
   _transformTobeMapped.rot_y = atan2(srycrx / _transformTobeMapped.rot_x.cos(),
                                      crycrx / _transformTobeMapped.rot_x.cos());

   float srzcrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      + cbcx * sbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   float crzcrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
      + cbcx * cbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   _transformTobeMapped.rot_z = atan2(srzcrx / _transformTobeMapped.rot_x.cos(),
                                      crzcrx / _transformTobeMapped.rot_x.cos());

   Vector3 v = _transformIncre.pos;
   // 两次“全局优化前里程计位置间的位移”之前已用这次里程计旋转角旋转，现在再用初始猜测_transformTobeMapped转回来，其中“初始猜测”比“这次里程计旋转角”多了“上次全局优化前后旋转偏差”
   // 相当于给位移增量补上了“上次全局优化前后旋转偏差”，得到“这次全局优化后位移增量的初始猜测”
   rotateZXY(v, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);
   // 待优化位置初始值 = 上次全局优化后的位置 - 这次全局优化后位移增量(上次-这次)的初始猜测 = 这次全局优化后位置的初始猜测
   _transformTobeMapped.pos = _transformAftMapped.pos - v;
}



void BasicLaserMapping::transformUpdate()
{
   if (0 < _imuHistory.size())   // 若有IMU数据可用
   {
      size_t imuIdx = 0;   // 从队首的IMU数据开始查找

      // 计算 timeDiff = 距上次IMU数据的时间间隔 = sweep开始时间 + 扫描周期 - 上次IMU数据时间 = sweep结束时间 - 上次IMU数据时间
      // 插值需要未来IMU状态，故最新IMU数据需要比sweep结束时间超前，即：需要timeDiff<=0
      // IMU数据用尽前，循环寻找更加新鲜的IMU数据，若找到timeDiff<=0，即超前或同时刻的数据，则停止
      while (imuIdx < _imuHistory.size() - 1 && toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0)
      {
         imuIdx++;
      }

      IMUState2 imuCur;

      // 若只有一个IMU数据，或所有数据均滞后，则无法插值，直接将最后一个IMU状态当作输出
      if (imuIdx == 0 || toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0)
      {
         // scan time newer then newest or older than oldest IMU message
         imuCur = _imuHistory[imuIdx];
      }
      else  // 若存在更超前的IMU数据，则计算距离超前数据的时间间隔占两IMU数据时间间隔的比例，并按该比例插值获得输出
      {
         float ratio = (toSec(_imuHistory[imuIdx].stamp - _laserOdometryTime) - _scanPeriod)
            / toSec(_imuHistory[imuIdx].stamp - _imuHistory[imuIdx - 1].stamp);

         IMUState2::interpolate(_imuHistory[imuIdx], _imuHistory[imuIdx - 1], ratio, imuCur);
      }

      // 建图中IMU只补偿pitch和roll，并且所占比例是一个固定的非常小的值（推测是因为之前ScanRegistration和Odometry的IMU补偿已经够多，并且lidar建图算法的准确度还算可以）
      _transformTobeMapped.rot_x = 0.998 * _transformTobeMapped.rot_x.rad() + 0.002 * imuCur.pitch.rad();
      _transformTobeMapped.rot_z = 0.998 * _transformTobeMapped.rot_z.rad() + 0.002 * imuCur.roll.rad();
   }

   _transformBefMapped = _transformSum;         // 全局优化前里程计给出的位姿记录为“上次建图前的”，供下次建图计算初始猜测使用
   _transformAftMapped = _transformTobeMapped;  // 全局优化后优化得到的位姿记录为“上次建图前的”，供下次建图计算初始猜测使用
}



void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   po.x = pi.x;
   po.y = pi.y;
   po.z = pi.z;
   po.intensity = pi.intensity;

   rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

   po.x += _transformTobeMapped.pos.x();
   po.y += _transformTobeMapped.pos.y();
   po.z += _transformTobeMapped.pos.z();
}



void BasicLaserMapping::pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   po.x = pi.x - _transformTobeMapped.pos.x();
   po.y = pi.y - _transformTobeMapped.pos.y();
   po.z = pi.z - _transformTobeMapped.pos.z();
   po.intensity = pi.intensity;

   rotateYXZ(po, -_transformTobeMapped.rot_y, -_transformTobeMapped.rot_x, -_transformTobeMapped.rot_z);
}



void BasicLaserMapping::transformFullResToMap()
{
   // transform full resolution input cloud to map
   for (auto& pt : *_laserCloudFullRes)
      pointAssociateToMap(pt, pt);
}

bool BasicLaserMapping::createDownsizedMap()
{
   // create new map cloud according to the input output ratio 根据输入输出帧数比建立新的周围特征点云地图
   // 每隔(_mapFrameNum-1)帧建图1帧周围特征点云，中间(_mapFrameNum-1)帧被跳过，但第1帧一定建图周围点云
   _mapFrameCount++;
   if (_mapFrameCount < _mapFrameNum)
      return false;

   _mapFrameCount = 0;

   // accumulate map cloud 累积地图点云
   _laserCloudSurround->clear(); // 周围点云清空
   for (auto ind : _laserCloudSurroundInd)   // 周围子cube点云中的角点和平面点构成“周围点云”
   {
      *_laserCloudSurround += *_laserCloudCornerArray[ind];
      *_laserCloudSurround += *_laserCloudSurfArray[ind];
   }

   // down size map cloud 下采样周围点云，滤波器尺寸和下采样角点用的体素栅格滤波器相同(小于平面点的滤波器尺寸)
   _laserCloudSurroundDS->clear();
   _downSizeFilterCorner.setInputCloud(_laserCloudSurround);
   _downSizeFilterCorner.filter(*_laserCloudSurroundDS);
   return true;
}

bool BasicLaserMapping::process(Time const& laserOdometryTime)
{
   // skip some frames?!?
   _frameCount++;
   if (_frameCount < _stackFrameNum)   // 若_stackFrameNum>1，则会跳过一些帧，即每隔一些帧处理一帧
   {
      return false;
   }
   _frameCount = 0;
   _laserOdometryTime = laserOdometryTime;   // 时间戳 = 本次里程计sweep开始时刻

   pcl::PointXYZI pointSel;   // 临时存放一个特定的点的变量

   // relate incoming data to map 获得这次全局优化后位姿的初始猜测
   transformAssociateToMap();

   // 这次里程计中的特征点原本在sweep结束时的lidar坐标系中，全部用_transformTobeMapped(初始猜测)变换到世界坐标系；
   // 然后分类存放在_laserCloudCornerStack或_laserCloudSurfStack中
   for (auto const& pt : _laserCloudCornerLast->points)
   {
      pointAssociateToMap(pt, pointSel);
      _laserCloudCornerStack->push_back(pointSel);
   }

   for (auto const& pt : _laserCloudSurfLast->points)
   {
      pointAssociateToMap(pt, pointSel);
      _laserCloudSurfStack->push_back(pointSel);
   }

   // 判断视野时余弦定理辅助点：lidar的Y轴(向上为正)正向上(0,10m,0)处的点，变换到世界坐标系中
   pcl::PointXYZI pointOnYAxis;
   pointOnYAxis.x = 0.0;
   pointOnYAxis.y = 10.0;
   pointOnYAxis.z = 0.0;
   pointAssociateToMap(pointOnYAxis, pointOnYAxis);

   // 整个地图范围限制在一个大立方体中，这个大cube划分为_laserCloudNum个子cube，每个子cube中的点云是点云向量的一个元素
   // 保存地图点云的大立方体中子cube的尺寸：边长50m
   auto const CUBE_SIZE = 50.0;
   auto const CUBE_HALF = CUBE_SIZE / 2;

   /** 大立方体中lidar所在子cube的索引
    * 默认XYZ方向各分割为21、11、21个子cube；lidar所在索引 = 中心(10,5,10) + 加数
    * 加数(三个方向分别算) = 浮点运算(该方向上lidar坐标/CUBE_SIZE)的四舍五入
    * 注意：该算式表明，原点cube(即索引0,0,0的cube)的中心点和lidar的位姿世界坐标原点重合
    */
   int centerCubeI = int((_transformTobeMapped.pos.x() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;  // 大立方体中lidar所在子cube的X索引
   int centerCubeJ = int((_transformTobeMapped.pos.y() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight; // 大立方体中lidar所在子cube的Y索引
   int centerCubeK = int((_transformTobeMapped.pos.z() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;  // 大立方体中lidar所在子cube的Z索引
   // 当计算机的int()采用“向零取整”时，如果任一方向上lidar坐标<-一个子cube的一半，即当上面算出的索引应当在向下取整后得到一个负数时，则计算机会向上取整，所以该方向上索引应该再-1
   if (_transformTobeMapped.pos.x() + CUBE_HALF < 0) centerCubeI--;
   if (_transformTobeMapped.pos.y() + CUBE_HALF < 0) centerCubeJ--;
   if (_transformTobeMapped.pos.z() + CUBE_HALF < 0) centerCubeK--;

   // 如果lidar的坐标偏离中心太远，则使点云带着lidar和中心整体移动，直到3<=lidar的XZ索引<=17，且3<=lidar的Y索引<=7
   while (centerCubeI < 3) // 如果lidar的X索引太小，则点云向X正向移动，直到lidar的X索引>=3
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            // 不断交换，使各子cube点云向X正向移动1个索引，原X索引最大的子cube的点云变到X索引为0的位置，并且清除
            for (int i = _laserCloudWidth - 1; i >= 1; i--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i - 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(0, j, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeI++; // lidar所在cube的X索引+1
      _laserCloudCenWidth++;  // 大cube中心子cube的X索引+1
   }

   while (centerCubeI >= _laserCloudWidth - 3) // 如果lidar的X索引太大，则点云向X负向移动，直到lidar的X索引<18
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int i = 0; i < _laserCloudWidth - 1; i++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i + 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(_laserCloudWidth - 1, j, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeI--;
      _laserCloudCenWidth--;
   }

   while (centerCubeJ < 3) // 如果lidar的Y索引太小，则点云向Y正向移动，直到lidar的Y索引>=3
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = _laserCloudHeight - 1; j >= 1; j--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j - 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, 0, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeJ++;
      _laserCloudCenHeight++;
   }

   while (centerCubeJ >= _laserCloudHeight - 3) // 如果lidar的Y索引太大，则点云向Y负向移动，直到lidar的Y索引<8
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = 0; j < _laserCloudHeight - 1; j++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j + 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, _laserCloudHeight - 1, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeJ--;
      _laserCloudCenHeight--;
   }

   while (centerCubeK < 3) // 如果lidar的Z索引太小，则点云向Z正向移动，直到lidar的Z索引>=3
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = _laserCloudDepth - 1; k >= 1; k--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k - 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, j, 0);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeK++;
      _laserCloudCenDepth++;
   }

   while (centerCubeK >= _laserCloudDepth - 3) // 如果lidar的Z索引太大，则点云向Z负向移动，直到lidar的Z索引<18
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = 0; k < _laserCloudDepth - 1; k++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k + 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, j, _laserCloudDepth - 1);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeK--;
      _laserCloudCenDepth--;
   }

   // 从以lidar所在为中心的周围5*5*5个(包括lidar所在)相邻子cube中筛选出在lidar视野内的子cube
   _laserCloudValidInd.clear();  // 清空周围视野内子cube索引buffer
   _laserCloudSurroundInd.clear();  // 清空周围子cube索引buffer
   for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
   {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
         for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
         {
            if (i >= 0 && i < _laserCloudWidth &&
                j >= 0 && j < _laserCloudHeight &&
                k >= 0 && k < _laserCloudDepth) // 鲁棒性：如果索引合法(实际上前面平移已经限制范围了)
            {
               // “大立方体中心cube的中心”到“lidar所在cube中心”的向量(centerX,centerY,centerZ)
               float centerX = 50.0f * (i - _laserCloudCenWidth);
               float centerY = 50.0f * (j - _laserCloudCenHeight);
               float centerZ = 50.0f * (k - _laserCloudCenDepth);

               pcl::PointXYZI transform_pos = (pcl::PointXYZI) _transformTobeMapped.pos;  // (初始猜测)lidar的位置

               bool isInLaserFOV = false; // 默认不在视野内
               // 对lidar所在cube的8个顶点遍历，若其中任意一个顶点满足条件，判定为在视野内，则判定该cube在视野内
               for (int ii = -1; ii <= 1; ii += 2)
               {
                  for (int jj = -1; jj <= 1; jj += 2)
                  {
                     for (int kk = -1; kk <= 1; kk += 2)
                     {
                        pcl::PointXYZI corner;  // lidar所在cube的顶点，暂时记为C
                        corner.x = centerX + 25.0f * ii;
                        corner.y = centerY + 25.0f * jj;
                        corner.z = centerZ + 25.0f * kk;

                        float squaredSide1 = calcSquaredDiff(transform_pos, corner);   // 以“大立方体中心cube的中心”为原点，lidar位置T到该顶点C的平方距离
                        float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner); // lidar坐标系(0,10m,0)点Py转到世界坐标系后，到该顶点C的平方距离；P到T固定是10m

                        // 余弦定理：TPy^2 + TC^2 - PyC^2 = 2·TPy·TC·cos<TPy,TC>
                        float check1 = 100.0f + squaredSide1 - squaredSide2
                           - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                        float check2 = 100.0f + squaredSide1 - squaredSide2
                           + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                        // 这个条件意味着 -sqrt(3) < 2cos<TPy,TC> < sqrt(3)，即lidar到C和lidar正上方的夹角在(30度,150度)内，
                        // 即lidar到C和lidar水平面的夹角在(-60度,60度)内（考虑到还没优化，此处应为“大致满足”）
                        if (check1 < 0 && check2 > 0)
                        {
                           isInLaserFOV = true;
                        }
                     }
                  }
               }

               size_t cubeIdx = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;  // 当前考虑的子cube的一维索引
               if (isInLaserFOV)
               {
                  _laserCloudValidInd.push_back(cubeIdx);
               }
               _laserCloudSurroundInd.push_back(cubeIdx);
            }
         }
      }
   }

   // prepare valid map corner and surface cloud for pose optimization 从总体点云向量中提取周围视野内子cube的角点和平面点点云，构造本次优化配准要用的特征点云
   _laserCloudCornerFromMap->clear();
   _laserCloudSurfFromMap->clear();
   for (auto const& ind : _laserCloudValidInd)
   {
      *_laserCloudCornerFromMap += *_laserCloudCornerArray[ind];
      *_laserCloudSurfFromMap += *_laserCloudSurfArray[ind];
   }

   // prepare feature stack clouds for pose optimization 本次里程计sweep中用的将被配准的特征点用_transformTobeMapped(初始猜测)转换到lidar坐标系（中间没用，为什么要转过去再转回来？）
   for (auto& pt : *_laserCloudCornerStack)
      pointAssociateTobeMapped(pt, pt);

   for (auto& pt : *_laserCloudSurfStack)
      pointAssociateTobeMapped(pt, pt);

   // down sample feature stack clouds 下采样本次里程计sweep中用的特征点云，并记录下采样后其中特征点数目
   _laserCloudCornerStackDS->clear();
   _downSizeFilterCorner.setInputCloud(_laserCloudCornerStack);
   _downSizeFilterCorner.filter(*_laserCloudCornerStackDS);
   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();  // 本次里程计sweep中角点云下采样后点数

   _laserCloudSurfStackDS->clear();
   _downSizeFilterSurf.setInputCloud(_laserCloudSurfStack);
   _downSizeFilterSurf.filter(*_laserCloudSurfStackDS);
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();  // 本次里程计sweep中平面点云下采样后点数

   // 未下采样的特征点云不再使用，及时清理
   _laserCloudCornerStack->clear();
   _laserCloudSurfStack->clear();

   // run pose optimization 配准特征，优化位姿_transformTobeMapped
   optimizeTransformTobeMapped();

   // store down sized corner stack points in corresponding cube clouds 把下采样后的角点存放到对应的子cube中(即更新特征地图)
   for (int i = 0; i < laserCloudCornerStackNum; i++)
   {
      pointAssociateToMap(_laserCloudCornerStackDS->points[i], pointSel);  // 角点从lidar坐标系用全局优化后位姿转换到世界坐标系

      /** 大立方体中特征点所在子cube的索引
       * 默认XYZ方向各分割为21、11、21个子cube；特征点所在索引 = 中心(10,5,10) + 加数
       * 加数(三个方向分别算) = 浮点运算(该方向上特征点坐标/CUBE_SIZE)的四舍五入
       * 注意：该算式表明，原点cube(即索引0,0,0的cube)的中心点和特征点的世界坐标原点重合
       */
      
      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      // 当计算机的int()采用“向零取整”时，如果任一方向上特征点坐标<-一个子cube的一半，即当上面算出的索引应当在向下取整后得到一个负数时，则计算机会向上取整，所以该方向上索引应该再-1
      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      // 如果索引合法，加入对应子cube的角点点云
      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;  // 三维索引对应的一维索引
         _laserCloudCornerArray[cubeInd]->push_back(pointSel);
      }
   }

   // store down sized surface stack points in corresponding cube clouds 把下采样后的平面点存放到对应的子cube中(即更新特征地图)
   for (int i = 0; i < laserCloudSurfStackNum; i++)
   {
      pointAssociateToMap(_laserCloudSurfStackDS->points[i], pointSel);

      /** 大立方体中特征点所在子cube的索引，算法同lidar位置和角点位置对应索引 */
      
      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      // 当计算机的int()采用“向零取整”时，如果任一方向上特征点坐标<-一个子cube的一半，即当上面算出的索引应当在向下取整后得到一个负数时，则计算机会向上取整，所以该方向上索引应该再-1
      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      // 如果索引合法，加入对应子cube的平面点点云
      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;  // 三维索引对应的一维索引
         _laserCloudSurfArray[cubeInd]->push_back(pointSel);
      }
   }

   // down size all valid (within field of view) feature cube clouds 对所有有效(即视野内)子cube特征点云做一次下采样精简，防止地图膨胀过快
   for (auto const& ind : _laserCloudValidInd)
   {
      _laserCloudCornerDSArray[ind]->clear();   // 下采样后地图清零
      _downSizeFilterCorner.setInputCloud(_laserCloudCornerArray[ind]);
      _downSizeFilterCorner.filter(*_laserCloudCornerDSArray[ind]);

      _laserCloudSurfDSArray[ind]->clear();   // 下采样后地图清零
      _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
      _downSizeFilterSurf.filter(*_laserCloudSurfDSArray[ind]);

      // swap cube clouds for next processing 把下采样前后地图交换，这样下次全局优化使用的是下采样后地图；DS里存放的下采样前地图不再使用
      _laserCloudCornerArray[ind].swap(_laserCloudCornerDSArray[ind]);
      _laserCloudSurfArray[ind].swap(_laserCloudSurfDSArray[ind]);
   }

   transformFullResToMap();   // 准备发布1：本次sweep的全分辨率点云用全局优化后的位姿从lidar坐标系转换到世界坐标系
   _downsizedMapCreated = createDownsizedMap();    // 准备发布2：建立并下采样周围点云

   return true;
}


void BasicLaserMapping::updateIMU(IMUState2 const& newState)
{
   _imuHistory.push(newState);
}

void BasicLaserMapping::updateOdometry(double pitch, double yaw, double roll, double x, double y, double z)
{
   _transformSum.rot_x = pitch;
   _transformSum.rot_y = yaw;
   _transformSum.rot_z = roll;

   _transformSum.pos.x() = float(x);
   _transformSum.pos.y() = float(y);
   _transformSum.pos.z() = float(z);
}

void BasicLaserMapping::updateOdometry(Twist const& twist)
{
   _transformSum = twist;
}

nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromMap;

void BasicLaserMapping::optimizeTransformTobeMapped()
{
   // 只有特征点数目足够多(角点>10且平面点>100)才进行
   if (_laserCloudCornerFromMap->size() <= 10 || _laserCloudSurfFromMap->size() <= 100)
      return;

   pcl::PointXYZI pointSel, pointOri, /*pointProj, */coeff; // pointSel存放一个本次sweep的特征点转换到世界坐标系后的点；pointOri存放一个本次sweep的lidar坐标系下的特征点；coeff临时存放已加权偏导数

   std::vector<int> pointSearchInd(5, 0);       // 查找到的最邻近点的索引
   std::vector<float> pointSearchSqDis(5, 0);   // “查找到的最邻近点”到“被查找点”的距离的平方

   // 特征点云构建KD树，便于查找其中点
   kdtreeCornerFromMap.setInputCloud(_laserCloudCornerFromMap);
   kdtreeSurfFromMap.setInputCloud(_laserCloudSurfFromMap);

   // 声明一系列矩阵运算中间变量
   
   Eigen::Matrix<float, 5, 3> matA0;  // 平面点配准中：找到的5个最邻近平面点的坐标，每行一个点(x,y,z)
   Eigen::Matrix<float, 5, 1> matB0;  // 平面点配准中：计算偏导数过程中解方程用到的全-1右侧向量
   Eigen::Vector3f matX0;             // 平面点配准中：计算偏导数过程中解方程得到的解
   Eigen::Matrix3f matA1;             // 角点配准中：找到的5个最邻近角点的协方差矩阵
   Eigen::Matrix<float, 1, 3> matD1;  // 角点配准中：协方差矩阵特征值
   Eigen::Matrix3f matV1;             // 角点配准中：协方差矩阵特征向量（每列是一个特征向量）

   matA0.setZero();
   matB0.setConstant(-1);
   matX0.setZero();

   matA1.setZero();
   matD1.setZero();
   matV1.setZero();

   bool isDegenerate = false;          // L-M算法第一次迭代中(J^T)J是否退化的flag
   Eigen::Matrix<float, 6, 6> matP;    // L-M算法第一次迭代中(J^T)J退化时使用

   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();  // 本次里程计sweep中角点云下采样后点数
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();      // 本次里程计sweep中平面点云下采样后点数

   /** Levenberg-Marquardt(L-M)算法：非线性最小二乘优化。最大迭代次数=_maxIterations。由于lambda=0，实际上是高斯牛顿法 */
   for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
   {
      _laserCloudOri.clear();   // 上一次迭代中已匹配特征点清空
      _coeffSel.clear();        // 上一次迭代中已匹配特征点的偏导数和距离清空

      /** 特征对应配准：每次迭代都要搜索对应特征，相比里程计5次迭代1次搜索，计算更慢但结果更准确 */
      for (int i = 0; i < laserCloudCornerStackNum; i++)  // i：对本次sweep每个角点
      {
         pointOri = _laserCloudCornerStackDS->points[i];
         pointAssociateToMap(pointOri, pointSel);  // 本次sweep特征点转换到世界坐标系
         kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); // 找5个当前角点的K最邻近角点，记录索引和距离平方

         if (pointSearchSqDis[4] < 1.0)   // 若角点和找到的最邻近的1个点距离平方<1，即相距<1m，才认为真的很邻近并采用（精度要求比里程计的5m高很多）
         {
            Vector3 vc(0, 0, 0); // 找到的5个最邻近点的平均坐标（质心，或重心）

            for (int j = 0; j < 5; j++)
               vc += Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]);
            vc /= 5.0;

            Eigen::Matrix3f mat_a;  // 协方差矩阵：找到的5个最邻近点离它们质心的均方误差：各坐标轴上的均方误差以及不同轴坐标间的协方差；注意只用计算一半，它只有下三角部分有非零值
            mat_a.setZero();

            for (int j = 0; j < 5; j++)
            {
               Vector3 a = Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]) - vc;

               mat_a(0, 0) += a.x() * a.x();
               mat_a(1, 0) += a.x() * a.y();
               mat_a(2, 0) += a.x() * a.z();
               mat_a(1, 1) += a.y() * a.y();
               mat_a(2, 1) += a.y() * a.z();
               mat_a(2, 2) += a.z() * a.z();
            }
            matA1 = mat_a / 5.0;
            // This solver only looks at the lower-triangular part of matA1.
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
            matD1 = esolver.eigenvalues().real();
            matV1 = esolver.eigenvectors().real();

            /** 对协方差矩阵进行主成分分析：
             * (1)若这五个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，与该特征值相关的特征向量表示所处直线的方向；
             * (2)若这五个点分布在平面上，协方差矩阵的特征值存在一个显著小的元素，与该特征值相关的特征向量表示所处平面的法线方向
             */
            if (matD1(0, 2) > 3 * matD1(0, 1))  // 最大特征值>第二大特征值的3倍，则认为主成分分析结果是情况(1)，配准成功
            {

               // 求距离的三个点：特征点转换到世界坐标系的点；质心+0.1*(特征值最大点对应的特征向量)；质心-0.1*(特征值最大点对应的特征向量)
               // 后两点即过质心的对应特征线段

               float x0 = pointSel.x;
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               float x1 = vc.x() + 0.1 * matV1(0, 2);
               float y1 = vc.y() + 0.1 * matV1(1, 2);
               float z1 = vc.z() + 0.1 * matV1(2, 2);
               float x2 = vc.x() - 0.1 * matV1(0, 2);
               float y2 = vc.y() - 0.1 * matV1(1, 2);
               float z2 = vc.z() - 0.1 * matV1(2, 2);

               // 算式意义类似里程计角点配准
               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                 * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float ld2 = a012 / l12;

//                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
//                pointProj = pointSel;
//                pointProj.x -= la * ld2;
//                pointProj.y -= lb * ld2;
//                pointProj.z -= lc * ld2;

               /** 角点配准
                * 原则：点离对应配准特征线段距离越远，就认为这次配准越不可靠，权重越低
                * 系数的来源：应当使权重为正数。0.9是工程经验？
                * 注意：这里没有用论文上的复杂二次公式，而用了简单线性的表达式
                */
               float s = 1 - 0.9f * fabs(ld2);

               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.intensity = s * ld2;

               if (s > 0.1)   // 权重>0.1的特征点才能视作配准成功，记录点的lidar坐标系坐标和已加权偏导数
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }  // END if (协方差矩阵主成分分析表明5个点基本分布在同一条直线上，构成一条特征线段)
         }  // END if (找到的5个最邻近点离当前角点确实很近)
      }  // END for (本次sweep每个角点)

      for (int i = 0; i < laserCloudSurfStackNum; i++)  // i：对本次sweep每个平面点
      {
         pointOri = _laserCloudSurfStackDS->points[i];
         pointAssociateToMap(pointOri, pointSel);  // 本次sweep特征点转换到世界坐标系
         kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);  // 找5个当前角点的K最邻近平面点，记录索引和距离平方

         if (pointSearchSqDis[4] < 1.0)   // 若平面点和找到的最邻近的1个点距离平方<1，即相距<1m，才认为真的很邻近并采用（精度要求比里程计的5m高很多）
         {
            for (int j = 0; j < 5; j++)   // 提取找到的5个最邻近平面点的坐标，每行一个点(x,y,z)
            {
               matA0(j, 0) = _laserCloudSurfFromMap->points[pointSearchInd[j]].x;
               matA0(j, 1) = _laserCloudSurfFromMap->points[pointSearchInd[j]].y;
               matA0(j, 2) = _laserCloudSurfFromMap->points[pointSearchInd[j]].z;
            }
            // 构造方程：A0·X=B0；其中A0每行是一个邻近点的坐标向量，和X内积得到等号右边的一个-1
            // 这个方程实际上是假设5个点共面，平面方程是ax+by+cz+1=0；根据5个点坐标得到超定方程组，解得最小二乘解(a,b,c)
            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            // 平面方程的系数，也是法向量的分量
            float pa = matX0(0, 0); // 平面方程x前系数a(法向量X)
            float pb = matX0(1, 0); // 平面方程y前系数b(法向量Y)
            float pc = matX0(2, 0); // 平面方程z前系数c(法向量Z)
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);   // 法向量模长
            // 法向量长度归一化
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            /** 判断这5个点是否真的基本共面：每个点都代入解出的平面方程(已做法向量长度归一化处理后的方程)，结果即点到平面距离，越接近0越好；
             * 若任意一个点到平面距离>0.2m，则认为共面性不好，配准平面片失败
             */
            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
               if (fabs(pa * _laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        pb * _laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        pc * _laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)   // 点到平面距离公式 = 点代入直线方程求模，再除以法向量模长
               {
                  planeValid = false;
                  break;
               }
            }

            if (planeValid)   // 若共面性好，配准成功
            {
               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

               //                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
               //                pointProj = pointSel;
               //                pointProj.x -= pa * pd2;
               //                pointProj.y -= pb * pd2;
               //                pointProj.z -= pc * pd2;

               /** 平面点配准
                * 原则1：点离对应配准特征距离越远，就认为这次配准越不可靠，权重越低
                * 原则2：到原点距离越近，平面随lidar运动的位姿变化越剧烈，配准越不可靠，权重越低
                * 系数的来源：应当使权重为正数。0.9和原则2是工程经验？
                * 注意：这里没有用论文上的复杂二次公式，而用了简单的(特征点离对应配准平面片距离/特征点距原点距离的平方根)
                */
               float s = 1 - 0.9f * fabs(pd2) / sqrt(calcPointDistance(pointSel));

               // 根据点到平面距离公式，d对特征点变换到世界坐标系后的坐标x0,y0,z0的偏导数分别就是pa,pb,pc
               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.intensity = s * pd2;

               if (s > 0.1)  // 权重>0.1的特征点才能视作配准成功，记录点的lidar坐标系坐标和已加权偏导数
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }  // END if (协方差矩阵主成分分析表明5个点基本分布在同平面上，构成一个特征平面片)
         }  // END if (找到的5个最邻近点离当前平面点确实很近)
      }  // END for (本次sweep每个平面点)

      /** 优化求解位姿 */

      float srx = _transformTobeMapped.rot_x.sin(); // 待优化位姿的pitch的sin值
      float crx = _transformTobeMapped.rot_x.cos(); // 待优化位姿的pitch的cos值
      float sry = _transformTobeMapped.rot_y.sin(); // 待优化位姿的yaw的sin值
      float cry = _transformTobeMapped.rot_y.cos(); // 待优化位姿的yaw的cos值
      float srz = _transformTobeMapped.rot_z.sin(); // 待优化位姿的roll的sin值
      float crz = _transformTobeMapped.rot_z.cos(); // 待优化位姿的roll的cos值

      // 只有成功配准的点数>=50才进行位姿估计的优化计算；可用点太少时，优化视为不可靠，跳过此轮，开始下一轮迭代
      size_t laserCloudSelNum = _laserCloudOri.size();
      if (laserCloudSelNum < 50)
         continue;

      // 声明一系列矩阵运算中间变量
      
      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);  // 雅可比矩阵J
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum); // 雅可比矩阵转置J^T
      Eigen::Matrix<float, 6, 6> matAtA;        // J^T·J
      Eigen::VectorXf matB(laserCloudSelNum);   // 距离向量d
      Eigen::VectorXf matAtB;                   // J^T·d
      Eigen::VectorXf matX;                     // 位姿的每次迭代更新量

      for (int i = 0; i < laserCloudSelNum; i++)  // 对每个已配准特征点，有一个方程，对应雅可比矩阵的一行(对3个变量待优化变量rx ry rz的偏导数)
      {
         pointOri = _laserCloudOri.points[i]; // 当前特征点
         coeff = _coeffSel.points[i];         // 当前特征点的偏导数和到对应配准特征的距离

         // 使用求导的链式法则：d对位姿的导数 = d对特征点坐标(世界坐标系)的导数 · 特征点坐标(世界坐标系)对位姿的导数
         // 对rx的偏导数
         float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
            + (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
            + (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

         // 对ry的偏导数
         float ary = ((cry*srx*srz - crz * sry)*pointOri.x
                      + (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
            + ((-cry * crz - srx * sry*srz)*pointOri.x
               + (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

         // 对rz的偏导数
         float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
            + (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
            + ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

         matA(i, 0) = arx;
         matA(i, 1) = ary;
         matA(i, 2) = arz;
         matA(i, 3) = coeff.x;
         matA(i, 4) = coeff.y;
         matA(i, 5) = coeff.z;
         matB(i, 0) = -coeff.intensity;   // 为了求解结果准确，这里距离没有像odometry里一样为了加快收敛而直接设到很小
      }  // END for (每个已配准特征点)

      // 目标是d->0，L-M方法是用( T - (J^T·J+lambda·diag(J^T·J))^(-1)·(J^T)·d )去更新T；每次迭代更新量x = - (J^T·J+lambda·diag(J^T·J))^(-1)·(J^T)·d
      // 实现中lambda=0，实际是高斯牛顿法，此时x = - (J^T·J)^(-1)(J^T)d，即方程(J^T·J)·x = (J^T)(-d)的解
      // 使用列选主元的Householder QR分解求解线性方程组 (J^T)J·x = (J^T)(-d)
      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      // 关于防止非线性优化解退化：https://zhuanlan.zhihu.com/p/258159552
      if (iterCount == 0)  // 第一次迭代时，判断(J^T)J是否退化，详见网络参考资料和odometry对应部分注释
      {
         Eigen::Matrix<float, 1, 6> matE;
         Eigen::Matrix<float, 6, 6> matV;
         Eigen::Matrix<float, 6, 6> matV2;

         Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
         matE = esolver.eigenvalues().real();
         matV = esolver.eigenvectors().real();

         // matV2 = matV;
         matV2 = matV.transpose();  // 复制一份特征向量矩阵，并且使行是特征向量

         isDegenerate = false;
         float eignThre[6] = { 100, 100, 100, 100, 100, 100 }; // 判断矩阵是否退化的特征值阈值，比odometry中的大
         for (int i = 0; i < 6; i++)
         {
            if (matE(0, i) < eignThre[i])
            {
               for (int j = 0; j < 6; j++)
               {
                  matV2(i, j) = 0;
               }
               isDegenerate = true;
            }
            else
            {
               break;
            }
         }
         matP = matV.transpose().inverse() * matV2;
         // matP = matV.inverse() * matV2;
      }

      if (isDegenerate)
      {
         Eigen::Matrix<float, 6, 1> matX2(matX);
         matX = matP * matX2;
      }

      // 更新被优化变量
      _transformTobeMapped.rot_x += matX(0, 0);
      _transformTobeMapped.rot_y += matX(1, 0);
      _transformTobeMapped.rot_z += matX(2, 0);
      _transformTobeMapped.pos.x() += matX(3, 0);
      _transformTobeMapped.pos.y() += matX(4, 0);
      _transformTobeMapped.pos.z() += matX(5, 0);

      // 计算更新量向量的幅值
      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      // 若更新量足够小，则停止迭代
      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
         break;
   }  // END for (L-M算法迭代)

   // 用本次里程计和全局优化结果更新_transformBefMapped和_transformAftMapped，供下次全局优化计算初始猜测使用；若有IMU数据，则用其略微修正pitch和roll，再更新
   transformUpdate();
}


} // end namespace loam
