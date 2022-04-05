#include "loam_velodyne/BasicLaserOdometry.h"

#include "math_utils.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam
{

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;


BasicLaserOdometry::BasicLaserOdometry(float scanPeriod, size_t maxIterations) :
   _scanPeriod(scanPeriod),
   _systemInited(false),
   _frameCount(0),
   _maxIterations(maxIterations),
   _deltaTAbort(0.1),
   _deltaRAbort(0.1),
   _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
   _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
   _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
   _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
   _coeffSel(new pcl::PointCloud<pcl::PointXYZI>())
{}



void BasicLaserOdometry::transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   // 插值系数 = “当前点相对sweep开始时刻的相对时间”/“扫描周期” = 当前点的时间占扫描周期的比例
   // 注意：所谓相对时间是指时间起点是相对的，其数值仍为绝对秒数
   float s = (1.f / _scanPeriod) * (pi.intensity - int(pi.intensity));

   /** “sweep内匀速运动”假设下，可用线性插值实现投影
    * 投影时先平移再旋转，是因为位姿变换是 X2 = R·X1 + t，即先旋转再平移，所以消除位姿变换时先减去平移，再反向旋转
    * 关于正负号：设在一个sweep内lidar(本地坐标系原点)发生的变化是-yaw,-pitch,-row,-t(-ry,-rx,-rz,-t，有先后顺序)，则本地坐标系中点坐标的变化是yaw,pitch,row,t
    *           于是把点投影回sweep开始时的坐标系的操作应当是-t,-row,-pitch,-yaw(-t,-rz,-rx,-ry)；线性插值即上述所有量乘s
    * 第一次迭代时，_transform是上一sweep处理后的状态，即上一sweep结束时(这一sweep开始时)；之后每一次迭代都要更新_transform并用新的_transform做变换、计算更新量
    * (其他条件理想的情况下，如果_transform完全准确，则投影后特征点和配准特征间的距离d应当接近零)
    */
   po.x = pi.x - s * _transform.pos.x();
   po.y = pi.y - s * _transform.pos.y();
   po.z = pi.z - s * _transform.pos.z();
   po.intensity = pi.intensity;  // 此处intensity表示时间戳，仍不变

   Angle rx = -s * _transform.rot_x.rad();   // pitch
   Angle ry = -s * _transform.rot_y.rad();   // yaw
   Angle rz = -s * _transform.rot_z.rad();   // roll
   rotateZXY(po, rz, rx, ry); // 原本是点本身的时间戳的lidar坐标系，转到sweep开始时的lidar坐标系
}



size_t BasicLaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
   size_t cloudSize = cloud->points.size();  // 点云中点数

   for (size_t i = 0; i < cloudSize; i++) // 对点云中每一个点
   {
      pcl::PointXYZI& point = cloud->points[i]; // 用引用表示点，使代码简洁

      // 插值系数 = “当前点相对sweep开始时刻的相对时间”/“扫描周期”
      float s = (1.f / _scanPeriod) * (point.intensity - int(point.intensity));

      /** “sweep内匀速运动”假设下，可用线性插值实现投影
       * 正负号：参见transformToStart的注释
       */
      point.x -= s * _transform.pos.x();
      point.y -= s * _transform.pos.y();
      point.z -= s * _transform.pos.z();
      point.intensity = int(point.intensity);  // 此处intensity表示时间戳，仍不变

      Angle rx = -s * _transform.rot_x.rad();
      Angle ry = -s * _transform.rot_y.rad();
      Angle rz = -s * _transform.rot_z.rad();
      rotateZXY(point, rz, rx, ry); // 原本是点本身的时间戳的lidar坐标系，转到sweep开始时的lidar坐标系
      // _transform本身记录的就是该sweep开始到结束lidar坐标系下点坐标的变化(ry,rx,rz,t)
      rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

      point.x += _transform.pos.x() - _imuShiftFromStart.x();
      point.y += _transform.pos.y() - _imuShiftFromStart.y();
      point.z += _transform.pos.z() - _imuShiftFromStart.z();

      // 用IMU数据去除非线性运动(此处指角速度非零，即转向)失真
      rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart); // 有IMU数据时，所有点在处理中都投影到了sweep开始时，此时转到世界坐标系
      rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);    // 转到sweep结束时刻(下一sweep开始时刻)
   }

   return cloudSize;
}



void BasicLaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,   // 修正前欧拉角
                                           const Angle& blx, const Angle& bly, const Angle& blz,   // IMU在sweep开始时的欧拉角
                                           const Angle& alx, const Angle& aly, const Angle& alz,   // IMU在sweep终止时的欧拉角
                                           Angle &acx, Angle &acy, Angle &acz)                     // 修正后欧拉角(调用时直接放回修正前的变量里了)
{
   float sbcx = bcx.sin();
   float cbcx = bcx.cos();
   float sbcy = bcy.sin();
   float cbcy = bcy.cos();
   float sbcz = bcz.sin();
   float cbcz = bcz.cos();

   float sblx = blx.sin();
   float cblx = blx.cos();
   float sbly = bly.sin();
   float cbly = bly.cos();
   float sblz = blz.sin();
   float cblz = blz.cos();

   float salx = alx.sin();
   float calx = alx.cos();
   float saly = aly.sin();
   float caly = aly.cos();
   float salz = alz.sin();
   float calz = alz.cos();

   float srx = -sbcx * (salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly)
      - cbcx * cbcz*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                     - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - cbcx * sbcz*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                     - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz);
   acx = -asin(srx);

   float srycrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      + cbcx * sbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   float crycrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      + cbcx * cbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

   float srzcrx = sbcx * (cblx*cbly*(calz*saly - caly * salx*salz) - cblx * sbly*(caly*calz + salx * saly*salz) + calx * salz*sblx)
      - cbcx * cbcz*((caly*calz + salx * saly*salz)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(sbly*sblz + cbly * cblz*sblx)
                     - calx * cblx*cblz*salz)
      + cbcx * sbcz*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + calx * cblx*salz*sblz);
   float crzcrx = sbcx * (cblx*sbly*(caly*salz - calz * salx*saly) - cblx * cbly*(saly*salz + caly * calz*salx) + calx * calz*sblx)
      + cbcx * cbcz*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
                     + calx * calz*cblx*cblz)
      - cbcx * sbcz*((saly*salz + caly * calz*salx)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*cblz + sblx * sbly*sblz)
                     - calx * calz*cblx*sblz);
   acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}



void BasicLaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,     // 历史累积旋转
                                            Angle lx, Angle ly, Angle lz,     // 本次sweep期间优化解出的旋转 的取负号
                                            Angle &ox, Angle &oy, Angle &oz)  // 结果存放处
{
   float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin()
      - cx.cos()*cz.cos()*lx.sin()
      - lx.cos()*ly.cos()*cx.sin();
   ox = -asin(srx);

   float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin())
      + lx.cos()*ly.sin()*(cy.cos()*cz.cos() + cx.sin()*cy.sin()*cz.sin())
      + lx.cos()*ly.cos()*cx.cos()*cy.sin();
   float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos()
      - lx.cos()*ly.sin()*(cz.cos()*cy.sin() - cy.cos()*cx.sin()*cz.sin())
      - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
   oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

   float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin())
      + cx.cos()*cz.sin()*(ly.cos()*lz.cos() + lx.sin()*ly.sin()*lz.sin())
      + lx.cos()*cx.cos()*cz.cos()*lz.sin();
   float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos()
      - cx.cos()*cz.sin()*(ly.cos()*lz.sin() - lz.cos()*lx.sin()*ly.sin())
      - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
   oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}

void BasicLaserOdometry::updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans)
{
   assert(4 == imuTrans.size()); // 断言接收到的IMU位姿变换信息是否符合4维(scan registration模块的定义)
   // sweep开始时刻的yaw-pitch-roll欧拉角
   _imuPitchStart = imuTrans.points[0].x;
   _imuYawStart = imuTrans.points[0].y;
   _imuRollStart = imuTrans.points[0].z;

   // 最新(sweep最后一个点)的yaw-pitch-roll欧拉角
   _imuPitchEnd = imuTrans.points[1].x;
   _imuYawEnd = imuTrans.points[1].y;
   _imuRollEnd = imuTrans.points[1].z;

   // 非线性运动(加减速)导致的累积漂移误差(相对sweep开始时)(在sweep开始时刻的IMU坐标系中)
   _imuShiftFromStart = imuTrans.points[2];
   // IMU速度变化(相对sweep开始时)(：)在sweep开始时刻的IMU坐标系中)
   _imuVeloFromStart = imuTrans.points[3];
}

void BasicLaserOdometry::process()
{
   /** 初始化：因为里程计算法需要至少两帧状态做比较，所以系统刚启动时第一帧输入没法单独做里程计，直接扔进“上一次”数据，等待第二帧输入 */
   if (!_systemInited)
   {
      // 角点和平面点直接作为上一次的角和平面特征点云(交换前，_last点云为空)
      _cornerPointsLessSharp.swap(_lastCornerCloud);
      _surfPointsLessFlat.swap(_lastSurfaceCloud);

      // 交换后，上一次角和平面特征点云构建KD树，便于查找其中点
      _lastCornerKDTree.setInputCloud(_lastCornerCloud);
      _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

      // 初始IMU位姿：注意loam内部使用X左Y上Z前的坐标系。初始只记录俯仰角和滚转角(偏航角是水平运动，没必要初始记录)
      _transformSum.rot_x += _imuPitchStart;
      _transformSum.rot_z += _imuRollStart;

      // 标记：系统已初始化，可以开始做里程计
      _systemInited = true;
      return;
   }

   pcl::PointXYZI coeff;               // 临时存放权重系数和已加权偏导数
   bool isDegenerate = false;          // L-M算法第一次迭代中(J^T)J是否退化的flag
   Eigen::Matrix<float, 6, 6> matP;    // L-M算法第一次迭代中(J^T)J退化时使用

   _frameCount++; // 已处理帧数+1
   _transform.pos -= _imuVeloFromStart * _scanPeriod; // 使用IMU数据估算并消除速度变化引起的位移


   size_t lastCornerCloudSize = _lastCornerCloud->points.size();     // 上一帧角点特征点数
   size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();   // 上一帧平面特征点数

   /** 优化求解位姿 */
   // 只有上一帧特征点数目足够多(角点>10且平面点>100)才进行
   if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
   {
      std::vector<int> pointSearchInd(1);       // 查找到的最邻近点的索引
      std::vector<float> pointSearchSqDis(1);   // “查找到的最邻近点”到“被查找点”的距离的平方
      std::vector<int> indices;                 // 去除NaN无效点时用到的辅助容器

      pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices); // 谨慎起见，锋锐角点云再去除一次NaN无效点
      size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();     // 当前新数据锋锐角点数目
      size_t surfPointsFlatNum = _surfPointsFlat->points.size();           // 当前新数据平坦平面点数目

      // 特征点查找索引缓冲器的最大容量改成当前新数据中相应种类特征点的数目
      _pointSearchCornerInd1.resize(cornerPointsSharpNum);
      _pointSearchCornerInd2.resize(cornerPointsSharpNum);
      _pointSearchSurfInd1.resize(surfPointsFlatNum);
      _pointSearchSurfInd2.resize(surfPointsFlatNum);
      _pointSearchSurfInd3.resize(surfPointsFlatNum);

      /** Levenberg-Marquardt(L-M)算法：非线性最小二乘优化。最大迭代次数=_maxIterations。由于lambda=0，实际上是高斯牛顿法 */
      for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
      {
         pcl::PointXYZI pointSel,                     // 当前帧特征点投影到sweep开始时坐标系后的点
                        pointProj,                    // 目前无用
                        tripod1, tripod2, tripod3;    // 三脚架：计算点到对应配准线段或平面片的距离时，参与构造平行四边形或四棱柱的上一帧特征点
         _laserCloudOri->clear();   // 上一次迭代中已匹配特征点清空
         _coeffSel->clear();        // 上一次迭代中已匹配特征点的偏导数和距离清空

         /** 特征对应配准 */
         for (int i = 0; i < cornerPointsSharpNum; i++)  // i：对每个锋锐角点
         {
            transformToStart(_cornerPointsSharp->points[i], pointSel);  // 锋锐角点投影到sweep开始时的坐标系，结果存放于pointSel(不改变原坐标，因为下次迭代还要用新位姿计算投影)

            if (iterCount % 5 == 0) // 每迭代5次，搜索一次角点的最邻近点和平方距离最小点
            {
               pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices); // 谨慎起见，上一帧锋锐角点云再去除一次NaN无效点
               _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis); // 找1个当前锋锐角点的K最邻近点，记录索引和距离平方

               int closestPointInd = -1,  // 最邻近点的索引，默认-1表示无
                     minPointInd2 = -1;   // 平方距离最小点的索引，默认-1表示无
               if (pointSearchSqDis[0] < 25) // 若锋锐角点和找到的最邻近点距离平方<25，即相距<5m，才认为真的很邻近并采用
               {
                  closestPointInd = pointSearchInd[0];
                  int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity); // 最邻近点所在扫描环ID

                  float pointSqDis,             // 存放两点间平方距离的临时变量
                        minPointSqDis2 = 25;    // 初始化遍历过程中找到的最小平方距离记录；默认值25表示要求该点距离不得大于或等于5m，即若所有点距离>=5米，则结果为不存在
                  // 在附近5条扫描环上搜索平方距离最小点
                  for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)  // 对上一帧中最邻近点索引之后的角点
                  {
                     // 如果在后2条扫描环更后面的扫描环上，则不考虑它
                     if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5)
                     {
                        break;
                     }

                     // 计算上一帧角点和当前锋锐角点的平方距离
                     pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

                     // 如果在后1条扫描环(intensity中relTime>0)或后2条扫描环(intensity中relTime<0)上
                     if (int(_lastCornerCloud->points[j].intensity) > closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2) // 若这个角点的平方距离 < 记录的最小平方距离，则更新最小平方距离，并将该点记录为平方距离最小点
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                  }
                  for (int j = closestPointInd - 1; j >= 0; j--)  // 对上一帧中最邻近点索引之前的角点
                  {
                     // 如果在前2条扫描环更前面的扫描环上，则不考虑它
                     if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5)
                     {
                        break;
                     }

                     // 计算上一帧角点和当前锋锐角点的平方距离
                     pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

                     // 如果在前2条扫描环(intensity中relTime>0)或前1条扫描环(intensity中relTime无限制)或同一条扫描环(intensity中relTime<0)上
                     if (int(_lastCornerCloud->points[j].intensity) < closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2) // 若这个角点的平方距离 < 记录的最小平方距离，则更新最小平方距离，并将该点记录为平方距离最小点
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                  }
               }

               _pointSearchCornerInd1[i] = closestPointInd;
               _pointSearchCornerInd2[i] = minPointInd2;
            }  // END if (迭代索引是5的整数倍，则搜索最邻近点和平方距离最小点)

            if (_pointSearchCornerInd2[i] >= 0) // 存在平方距离<25的“平方距离最小点”
            {
               tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]]; // 最邻近点
               tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]]; // 平方距离最小点

               // 提取三点坐标：0是当前帧特征点，1和2是组成对应配准线段的上一帧特征点
               float x0 = pointSel.x;
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               float x1 = tripod1.x;
               float y1 = tripod1.y;
               float z1 = tripod1.z;
               float x2 = tripod2.x;
               float y2 = tripod2.y;
               float z2 = tripod2.z;

               // 论文式(2)分子：向量10和20的叉积的模，即平行四边形面积(可用叉积的向量坐标行列式表达推导)
               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                 * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

               // 论文式(2)分母：向量21的模，即1和2的欧式距离
               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

               // 偏导数
               // d_epsilon对x0
               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

               // d_epsilon对y0
               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               // d_epsilon对z0
               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float ld2 = a012 / l12; // Eq. (2) 论文式(2)，0到线段12的距离d_epsilon

               // TODO: Why writing to a variable that's never read?
               pointProj = pointSel;
               pointProj.x -= la * ld2;
               pointProj.y -= lb * ld2;
               pointProj.z -= lc * ld2;

               // 当前特征点权重
               float s = 1;
               if (iterCount >= 5)  // 第5次迭代后开始添加权重
               {
                  /** 角点配准
                   * 原则：点离对应配准特征距离越远，就认为这次配准越不可靠，权重越低
                   * 系数的来源：应当使权重为正数。1.8是工程经验？
                   * 注意：这里没有用论文上的复杂二次公式，而用了简单线性的表达式
                   */
                  s = 1 - 1.8f * fabs(ld2);
               }

               // 权重是偏导数参与优化计算时的权重
               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.intensity = s * ld2; // 定性地：到对应配准线段的距离按比例缩小到可靠，使得权重等比例缩小

               if (s > 0.1 && ld2 != 0)   // 权重<=0.1的点和距离==0的点舍弃，后者是因为优化目标就是d->0
               {
                  _laserCloudOri->push_back(_cornerPointsSharp->points[i]);   // 找到匹配并计算了距离的特征点；注意：记录的是没有投影到sweep开始前的自身时间点坐标系的坐标
                  _coeffSel->push_back(coeff);                                // 该特征点对应的已加权偏导数和到配准距离；注意：是用投影到sweep开始时的坐标计算的
               }
            }  // END if (存在平方距离<25的“平方距离最小点”，即有3个点，则计算当前特征点到对应配准线段的距离，并赋予特征点权重)
         }  // END for (当前帧锋锐角点)

         for (int i = 0; i < surfPointsFlatNum; i++)  // i：对每个平面点
         {
            transformToStart(_surfPointsFlat->points[i], pointSel);  // 平面角点投影到sweep开始时的坐标系，结果存放于pointSel(不改变原坐标，因为下次迭代还要用新位姿计算投影)

            if (iterCount % 5 == 0) // 每迭代5次，搜索一次平面点的最邻近点和平方距离最小点
            {
               _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis); // 找1个当前平坦平面点的K最邻近点，记录索引和距离平方
               int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;   // 对平面点，找1个最邻近点和2个平方距离最小点
               if (pointSearchSqDis[0] < 25) // 若平坦平面点和找到的最邻近点距离平方<25，即相距<5m，才认为真的很邻近并采用
               {
                  closestPointInd = pointSearchInd[0];
                  int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity); // 最邻近点所在扫描环ID

                  float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;   // 初始化遍历过程中第一小和第二小平方距离记录；默认值25表示要求该点距离不得大于或等于5m，即若所有点距离>=5米，则结果为不存在
                  // 在附近5条扫描环上搜索平方距离最小点
                  for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)  // 对上一帧中最邻近点索引之后的平面点
                  {
                     // 如果在后2条扫描环更后面的扫描环上，则不考虑它
                     if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5)
                     {
                        break;
                     }

                     // 计算上一帧平面点和当前平坦平面点的平方距离
                     pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

                     // 考虑到平坦平面的分布范围和点运动范围大概率比锋锐边缘大很多，这里搜索范围有一边是开放的(半开区间)
                     // 如果在同一条扫描环(intensity中relTime<=0)或之前扫描环(intensity中relTime无限制)上
                     if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2) // 若这个角点的平方距离 < 记录的第一个最小平方距离，则更新最小平方距离，并将该点记录为平方距离最小点1
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                     else
                     {
                        if (pointSqDis < minPointSqDis3) // 若这个角点的平方距离比第一个记录大，但比第二个记录小，则更新第二个最小平方距离，并将该点记录为平方距离最小点2
                        {
                           minPointSqDis3 = pointSqDis;
                           minPointInd3 = j;
                        }
                     }
                  }
                  for (int j = closestPointInd - 1; j >= 0; j--)  // 对上一帧中最邻近点索引之前的平面点
                  {
                     // 如果在前2条扫描环更前面的扫描环上，则不考虑它
                     if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5)
                     {
                        break;
                     }

                     // 计算上一帧平面点和当前平坦平面点的平方距离
                     pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

                     // 考虑到平坦平面的分布范围和点运动范围大概率比锋锐边缘大很多，这里搜索范围有一边是开放的(半开区间)
                     // 如果在同一条扫描环(intensity中relTime>=0)或之后扫描环(intensity中relTime无限制)上
                     if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2) // 若这个角点的平方距离 < 记录的第一个最小平方距离，则更新最小平方距离，并将该点记录为平方距离最小点1
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                     else
                     {
                        if (pointSqDis < minPointSqDis3) // 若这个角点的平方距离比第一个记录大，但比第二个记录小，则更新第二个最小平方距离，并将该点记录为平方距离最小点2
                        {
                           minPointSqDis3 = pointSqDis;
                           minPointInd3 = j;
                        }
                     }
                  }
               }

               _pointSearchSurfInd1[i] = closestPointInd;
               _pointSearchSurfInd2[i] = minPointInd2;
               _pointSearchSurfInd3[i] = minPointInd3;
            }  // END if (迭代索引是5的整数倍，则搜索最邻近点和2个平方距离最小点)

            if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) // 存在平方距离<25的2个“平方距离最小点”
            {
               tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]]; // 最邻近点
               tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]]; // 平方距离最小点1
               tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]]; // 平方距离最小点2

               // 向量21和31的叉积结果向量的各分量
               float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                  - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);           // x分量
               float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                  - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);           // y分量
               float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                  - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);           // z分量
               // 论文式(3)分子 = 向量10(四棱柱侧棱) 和 向量21和31的叉积结果向量 的 内积(四棱柱体积) = (0到原点的向量 - 1到原点的向量) 点乘 向量21和31的叉积结果向量
               // pd = - 1到原点的向量 点乘 向量21和31的叉积结果向量
               float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

               // 论文式(3)分母：向量21和31的叉积结果向量的模，等于四棱柱底面平行四边形面积
               float ps = sqrt(pa * pa + pb * pb + pc * pc);
               // 分子各项除以分母(此时有的项还差一个和点0，即pointSel的坐标有关的因子)
               pa /= ps;   // 除以分母后，pa变为d_H对x0的偏导数
               pb /= ps;   // 除以分母后，pb变为d_H对y0的偏导数
               pc /= ps;   // 除以分母后，pc变为d_H对z0的偏导数
               pd /= ps;

               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd; //Eq. (3) 论文式(3) 平面点到对应配准平面片的距离d_H

               // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
               pointProj = pointSel;
               pointProj.x -= pa * pd2;
               pointProj.y -= pb * pd2;
               pointProj.z -= pc * pd2;

               // 当前特征点权重
               float s = 1;
               if (iterCount >= 5)  // 第5次迭代后开始添加权重
               {
                  /** 平面点配准
                   * 原则1：点离对应配准特征距离越远，就认为这次配准越不可靠，权重越低
                   * 原则2：到原点距离越近，平面随lidar运动的位姿变化越剧烈，配准越不可靠，权重越低
                   * 系数的来源：应当使权重为正数。1.8和原则2是工程经验？
                   * 注意：这里没有用论文上的复杂二次公式，而用了简单的(特征点离对应配准平面片距离/特征点距原点距离的平方根)
                   */
                  s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
               }

               // 权重是偏导数参与优化计算时的权重
               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.intensity = s * pd2; // 定性地：到对应配准平面片的距离按比例缩小到可靠，使得权重等比例缩小

               if (s > 0.1 && pd2 != 0)   // 权重<=0.1的点和距离==0的点舍弃，后者是因为优化目标就是d->0
               {
                  _laserCloudOri->push_back(_surfPointsFlat->points[i]);   // 找到匹配并计算了距离的特征点；注意：记录的是没有投影到sweep开始前的自身时间点坐标系的坐标
                  _coeffSel->push_back(coeff);                             // 该特征点对应的已加权偏导数和到配准距离；注意：是用投影到sweep开始时的坐标计算的
               }
            }  // END if (存在平方距离<25的2个“平方距离最小点”，即有4个点，则计算当前特征点到对应配准平面片的距离，并赋予特征点权重)
         }  // END for (当前帧平坦平面点)

         int pointSelNum = _laserCloudOri->points.size();   // 成功配准的特征点数目
         if (pointSelNum < 10)   // 只有成功配准的点数>=10才进行位姿估计的优化计算；可用点太少时，优化视为不可靠，跳过此轮，开始下一轮迭代
         {
            continue;
         }

         /** 优化求解位姿 */
         // 声明一系列矩阵运算中间变量

         Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);  // 雅可比矩阵J
         Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum); // 雅可比矩阵转置J^T
         Eigen::Matrix<float, 6, 6> matAtA;  // J^T·J
         Eigen::VectorXf matB(pointSelNum);  // 距离向量d
         Eigen::Matrix<float, 6, 1> matAtB;  // J^T·d
         Eigen::Matrix<float, 6, 1> matX;    // 位姿的每次迭代更新量

         // 计算雅可比矩阵
         for (int i = 0; i < pointSelNum; i++)  // 对每个已配准特征点，有一个方程，对应雅可比矩阵的一行(对6个变量待优化变量rx ry rz tx ty tz的偏导数)
         {
            const pcl::PointXYZI& pointOri = _laserCloudOri->points[i]; // 当前特征点
            coeff = _coeffSel->points[i];                               // 当前特征点的偏导数和到对应配准特征的距离

            float s = 1;   // 根据相对时间进行插值的系数，本来每个点的应该不同，但这里都视为1，没有实际作用（可能改成正确的插值系数的计算上产生较大代价，大于性能提升？）

            float srx = sin(s * _transform.rot_x.rad()); // 待优化位姿的pitch的sin值
            float crx = cos(s * _transform.rot_x.rad()); // 待优化位姿的pitch的cos值
            float sry = sin(s * _transform.rot_y.rad()); // 待优化位姿的yaw的sin值
            float cry = cos(s * _transform.rot_y.rad()); // 待优化位姿的yaw的cos值
            float srz = sin(s * _transform.rot_z.rad()); // 待优化位姿的roll的sin值
            float crz = cos(s * _transform.rot_z.rad()); // 待优化位姿的roll的cos值
            float tx = s * _transform.pos.x();           // 待优化位姿的平移：X
            float ty = s * _transform.pos.y();           // 待优化位姿的平移：Y
            float tz = s * _transform.pos.z();           // 待优化位姿的平移：Z

            // 使用求导的链式法则：d对位姿的导数 = d对特征点坐标(投影到sweep开始时)的导数 · 特征点坐标(投影到sweep开始时)对位姿的导数
            // 对rx的偏导数
            float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
                         + s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
               + (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
                  + s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
               + (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
                  + s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;

            // 对ry的偏导数
            float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
                         + (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
                         + tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
                         + s * tz*crx*cry) * coeff.x
               + ((s*cry*crz - s * srx*sry*srz)*pointOri.x
                  + (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
                  + s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
                  - tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

            // 对rz的偏导数
            float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
                         + tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
               + (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
                  + s * ty*crx*srz + s * tx*crx*crz) * coeff.y
               + ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
                  + tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

            // 对tx的偏导数
            float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
               - s * (crz*sry + cry * srx*srz) * coeff.z;

            // 对ty的偏导数
            float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
               - s * (sry*srz - cry * crz*srx) * coeff.z;

            // 对tz的偏导数
            float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

            float d2 = coeff.intensity;

            matA(i, 0) = arx;
            matA(i, 1) = ary;
            matA(i, 2) = arz;
            matA(i, 3) = atx;
            matA(i, 4) = aty;
            matA(i, 5) = atz;
            matB(i, 0) = -0.05 * d2;
            /** 为了加速d->0收敛，直接把d设置到很小(d除了用来计算权重和舍弃d=0的点外没有别的用途，这两个在本轮迭代中到此处都早已完结了，所以此举不会有其他影响)；
             *注意更新量方程的等号右边有个负号，加到这里了 
             */
         }  // END for (每个已配准特征点)
         matAt = matA.transpose();
         matAtA = matAt * matA;
         matAtB = matAt * matB;

         // 目标是d->0，L-M方法是用( T - (J^T·J+lambda·diag(J^T·J))^(-1)·(J^T)·d )去更新T；每次迭代更新量x = - (J^T·J+lambda·diag(J^T·J))^(-1)·(J^T)·d
         // 实现中lambda=0，实际是高斯牛顿法，此时x = - (J^T·J)^(-1)(J^T)d，即方程(J^T·J)·x = (J^T)(-d)的解
         // 使用列选主元的Householder QR分解求解线性方程组 (J^T)J·x = (J^T)(-d)
         matX = matAtA.colPivHouseholderQr().solve(matAtB);

         // 关于防止非线性优化解退化：https://zhuanlan.zhihu.com/p/258159552
         if (iterCount == 0)  // 第一次迭代时，判断(J^T)J是否退化
         {
            Eigen::Matrix<float, 1, 6> matE;
            Eigen::Matrix<float, 6, 6> matV;
            Eigen::Matrix<float, 6, 6> matV2;

            // 实数域自伴矩阵即对称阵；此处求对称阵(J^T)J的特征值和特征向量
            Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
            matE = esolver.eigenvalues().real();   // 注意：这个库函数会把特征值从小到大排序
            matV = esolver.eigenvectors().real();  // 注意：这个库函数求出的特征向量已经单位化(欧几里得范数 = 1)；列是特征向量

            // matV2 = matV;
            matV2 = matV.transpose();  // 复制一份特征向量矩阵，并且使行是特征向量

            isDegenerate = false;
            float eignThre[6] = { 10, 10, 10, 10, 10, 10 }; // 判断矩阵是否退化的特征值阈值
            for (int i = 0; i < 6; i++)
            {
               if (matE(0, i) < eignThre[i]) // 若某一特征值<阈值，则与之对应的特征向量置零，重复，直到遇到>=阈值的特征值
               {
                  for (int j = 0; j < 6; j++)
                  {
                     matV2(i, j) = 0;
                  }
                  isDegenerate = true; // 出现<阈值的特征值即判为有退化
               }
               else
               {
                  break;   // 若最小特征值 >= 阈值，则可直接判为无退化
               }
            }
            //
            matP = matV.transpose().inverse() * matV2;
            // matP = matV.inverse() * matV2;
         }

         if (isDegenerate) // 每次迭代中，若第一次迭代中发现(J^T)J有特征值非常小(已退化)，则在求解结果中左乘matP
         {
            Eigen::Matrix<float, 6, 1> matX2(matX);
            matX = matP * matX2;
         }

         // 更新被优化变量
         _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
         _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
         _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
         _transform.pos.x() += matX(3, 0);
         _transform.pos.y() += matX(4, 0);
         _transform.pos.z() += matX(5, 0);

         // 更新后结果若不是有限值(即是NaN)，则重置为零
         if (!pcl_isfinite(_transform.rot_x.rad())) _transform.rot_x = Angle();
         if (!pcl_isfinite(_transform.rot_y.rad())) _transform.rot_y = Angle();
         if (!pcl_isfinite(_transform.rot_z.rad())) _transform.rot_z = Angle();

         if (!pcl_isfinite(_transform.pos.x())) _transform.pos.x() = 0.0;
         if (!pcl_isfinite(_transform.pos.y())) _transform.pos.y() = 0.0;
         if (!pcl_isfinite(_transform.pos.z())) _transform.pos.z() = 0.0;

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
   }  // END if (上一帧可供配准的两类特征点数目都足够多)

   /** 结果累积和坐标转换 */
   // 注意：_transform中记录的是sweep开始时刻的点到结束时刻的坐标所经历的YXZ旋转和平移(ry=yaw,rx=pitch,rz=roll,t)，等效于坐标系的反方向变化，因此算坐标系位姿变化时取负号代入
   // 累积结果_transformSum中记录的是使sweep结束时的lidar坐标系中的点回到世界坐标系所要经历的ZXY旋转和平移(rz=roll,rx=pitch,ry=yaw,t)
   // 因此，世界坐标到局部坐标需要先-t，再转-yaw,-pitch,-roll，说明点不动时，局部坐标系的变化是先+t，再转yaw,pitch,roll
   Angle rx, ry, rz;
   accumulateRotation(_transformSum.rot_x,
                      _transformSum.rot_y,
                      _transformSum.rot_z,
                      -_transform.rot_x,
                      -_transform.rot_y.rad() * 1.05, // yaw修正：计算结果可能偏小，乘1.05
                      -_transform.rot_z,
                      rx, ry, rz);

   // 使用IMU数据修正加减速引起的非线性运动失真
   Vector3 v(_transform.pos.x() - _imuShiftFromStart.x(),
             _transform.pos.y() - _imuShiftFromStart.y(),
             _transform.pos.z() * 1.05 - _imuShiftFromStart.z()); // 竖直位移修正：计算结果可能偏小，乘1.05
   rotateZXY(v, rz, rx, ry);  // 修正后的位移转到世界坐标系
   Vector3 trans = _transformSum.pos - v; // 位移累积

   // 使用IMU信息修正转向引起的非线性运动失真
   pluginIMURotation(rx, ry, rz,
                     _imuPitchStart, _imuYawStart, _imuRollStart,
                     _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                     rx, ry, rz);

   _transformSum.rot_x = rx;  // 更新pitch
   _transformSum.rot_y = ry;  // 更新yaw
   _transformSum.rot_z = rz;  // 更新roll
   _transformSum.pos = trans; // 更新平移

   // 角点和平面点转换到sweep结束时刻(下一sweep开始时刻)，并用IMU信息消除加减速和转向非线性运动失真，供下一sweep里程计使用
   // 并记录其中点数信息，若点数充足，再用其构建KD树并在下一sweep中进行配准和迭代优化
   transformToEnd(_cornerPointsLessSharp);
   transformToEnd(_surfPointsLessFlat);

   _cornerPointsLessSharp.swap(_lastCornerCloud);
   _surfPointsLessFlat.swap(_lastSurfaceCloud);

   lastCornerCloudSize = _lastCornerCloud->points.size();
   lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

   if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
   {
      _lastCornerKDTree.setInputCloud(_lastCornerCloud);
      _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
   }

}



} // end namespace loam
