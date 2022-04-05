#include <pcl/filters/voxel_grid.h>

#include "loam_velodyne/BasicScanRegistration.h"
#include "math_utils.h"

namespace loam
{

//**********RegistrationParams类的构造函数**********
RegistrationParams::RegistrationParams(const float& scanPeriod_,
                                       const int& imuHistorySize_,
                                       const int& nFeatureRegions_,
                                       const int& curvatureRegion_,
                                       const int& maxCornerSharp_,
                                       const int& maxSurfaceFlat_,
                                       const float& lessFlatFilterSize_,
                                       const float& surfaceCurvatureThreshold_)
    : scanPeriod(scanPeriod_),
      imuHistorySize(imuHistorySize_),
      nFeatureRegions(nFeatureRegions_),
      curvatureRegion(curvatureRegion_),
      maxCornerSharp(maxCornerSharp_),
      maxCornerLessSharp(10 * maxCornerSharp_),
      maxSurfaceFlat(maxSurfaceFlat_),
      lessFlatFilterSize(lessFlatFilterSize_),
      surfaceCurvatureThreshold(surfaceCurvatureThreshold_)
{};

void BasicScanRegistration::processScanlines(const Time& scanTime, std::vector<pcl::PointCloud<pcl::PointXYZI>> const& laserCloudScans)
{
  // reset internal buffers and set IMU start state based on current scan time 根据当前scan时间重置内部缓冲器并设置IMU初始状态
  reset(scanTime);  

  // construct sorted full resolution cloud 构建一次sweep的全分辨率点云
  size_t cloudSize = 0;
  for (int i = 0; i < laserCloudScans.size(); i++) {  // 每个扫描面(scan)都加入最终点云，顺序按线号从小到大
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);   // 当前scan对应的点云点索引范围
    cloudSize += laserCloudScans[i].size(); // 总点数 += 当前scan点数
    range.second = cloudSize > 0 ? cloudSize - 1 : 0; // 当前scan点索引范围 = (之前总点数, 加上当前scan后总点数-1)
    _scanIndices.push_back(range);  // 记录当前scan对应的点索引范围
  }

  extractFeatures();  // 提取特征点
  updateIMUTransform(); // 更新IMU位姿信息；无IMU可用时总是发送全默认(全0)
}

bool BasicScanRegistration::configure(const RegistrationParams& config)
{
  _config = config; // 配置设置  
  _imuHistory.ensureCapacity(_config.imuHistorySize); // 根据设置调整IMU数据缓冲器的尺寸
  return true;  // 这个返回好像根本没用……为什么不用void？
}

void BasicScanRegistration::reset(const Time& scanTime)
{
  _scanTime = scanTime; // 第一个scan开始时间 = 当前scan时间

  // re-initialize IMU start index and state
  _imuIdx = 0;  // 查找IMU数据使用的索引归零
  if (hasIMUData()) { //若IMU数据缓冲器中有数据，则插值得到第一个scan开始时刻，即sweep开始时刻(0时刻)IMU状态
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep
  if (true/*newSweep*/) { // 处理时以sweep(即scan组)为单位，每次sweep开始时清理内部点云缓冲器
    _sweepStart = scanTime; // sweep开始时间 = 第一个scan开始时间

    // clear cloud buffers 清空所有点云缓冲器
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();

    // clear scan indices vector 清空存储各scan的索引的缓冲器
    _scanIndices.clear();
  }
}


void BasicScanRegistration::updateIMUData(Vector3& acc, IMUState& newState)
{
  // 使用IMU的上一个状态和加速度数据估计新状态，并加入IMU状态缓冲器
  // 假设：在两次IMU状态之间，是匀变速运动；因此已知瞬时加速度的条件下，采用匀变速估计运动
  // 注意：运算都是向量运算。每个乘式中只有一个三维向量，相加仍是三维向量，相当于XYZ三个方向分别计算

  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    /** 加速度转换到世界坐标系下：
     * 当前坐标系是世界坐标系经过yaw,pitch,roll旋转得到的，
     * 则当前坐标系中的向量经roll,pitch,yaw旋转，等效于向量不动，且当前坐标系做-roll,-pitch,-yaw旋转回到和世界坐标系重合，
     * 使向量处于世界坐标系中
     */
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    const IMUState& prevState = _imuHistory.last(); // 提取上一个IMU状态
    float timeDiff = toSec(newState.stamp - prevState.stamp); // 和上一个IMU状态间的时间间隔
    // 位置 = 初始位置 + 初速度*时间 + (1/2)*加速度*时间^2
    newState.position = prevState.position
                        + (prevState.velocity * timeDiff)
                        + (0.5 * acc * timeDiff * timeDiff);
    // 速度 = 初速度 + 加速度*时间
    newState.velocity = prevState.velocity
                        + acc * timeDiff;
  }

  _imuHistory.push(newState); // 把更新后的当前状态加入到缓冲器中
}


void BasicScanRegistration::projectPointToStartOfSweep(pcl::PointXYZI& point, float relTime)
{
  // project point to the start of the sweep using corresponding IMU data
  if (hasIMUData()) // 若有IMU数据可用
  {
    setIMUTransformFor(relTime);  // 插值得到当前点时刻的IMU状态，然后计算当前点时刻的IMU位置漂移误差
    transformToStartIMU(point);   // 利用当前点时刻的IMU在世界坐标系中的状态和位移，得到当前点的世界坐标，然后修正非线性运动导致的位置漂移误差，最后投影到sweep开始时的IMU坐标系(即无非线性运动的坐标系)中
  }
}


void BasicScanRegistration::setIMUTransformFor(const float& relTime)
{
  interpolateIMUStateFor(relTime, _imuCur); // 插值得到当前点时刻的IMU状态

  // 计算当前点距sweep开始时刻的时间间隔 = (最近scan的时间戳 - sweep开始时刻) + 距最近scan的时间
  float relSweepTime = toSec(_scanTime - _sweepStart) + relTime;
  /** 实际中不可能像假设一样一直匀速运动。IMU可以修正非线性运动的影响，此处修正的是加减速
   * 如果没有IMU，论文假设的是激光雷达做匀速运动(translation)，_imuPositionShift恒等于零；如果有IMU，就可以考虑激光雷达的非匀速运动(translation)，_imuPositionShift就是用来校正非匀速运动的，剩下来的匀速运动就和没有IMU的时候的求法一样了
   * 估算当前点时刻IMU位置漂移 = (插值所得)当前点时刻的IMU位置 - (sweep开始时IMU位置 + sweep开始时IMU速度 * 当前点距sweep开始时刻的时间间隔)[即：假设在当前sweep开始后匀速运动，应当到达的位置，称为累积位置]
   */
  _imuPositionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;
}



void BasicScanRegistration::transformToStartIMU(pcl::PointXYZI& point)
{
  // rotate point to global IMU system 把点旋转到世界坐标系
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift 世界坐标系中：利用IMU修正非线性运动造成的位置漂移
  // 加上位置偏移量，加是因为激光运动和点云运动是相反的。
  // 可以考虑加速运动下，PositionShift为正，并且点云测量坐标系变小，因此需要让点云坐标变大。
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state 把世界坐标系中的点旋转到该sweep开始时的无非线性运动IMU坐标系中
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}



void BasicScanRegistration::interpolateIMUStateFor(const float &relTime, IMUState &outputState)
{
  // 计算距上次IMU数据的时间间隔 = (最近scan时间 - 上次IMU数据时间) + 距最近scan的时间
  double timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;

  // 插值需要未来IMU状态，故最新IMU数据需要比relTime超前，即需要timeDiff<=0
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    // IMU数据用尽前，循环寻找更加新鲜的IMU数据，若找到timeDiff<=0即超前或同时刻的数据，则停止
    _imuIdx++;
    timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  }

  // 若只有一个IMU数据，或所有数据均滞后，则无法插值，直接将最后一个IMU状态当作输出
  if (_imuIdx == 0 || timeDiff > 0) {
    outputState = _imuHistory[_imuIdx];
  } else {  // 若存在更超前的IMU数据，则计算距离超前数据的时间间隔占两IMU数据时间间隔的比例，并按该比例插值获得输出
    float ratio = -timeDiff / toSec(_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp);
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio, outputState);
  }
}


void BasicScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  // extract features from individual scans
  size_t nScans = _scanIndices.size();  // scan的数目
  for (size_t i = beginIdx; i < nScans; i++) {  // i：scan循环索引；注意beginIdx在项目中只有取默认值0一种用法
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>); // 用指针新建一个pcl的XYZI点云
    size_t scanStartIdx = _scanIndices[i].first;  // 当前scan开头点的索引
    size_t scanEndIdx = _scanIndices[i].second;   // 当前scan结尾点的索引

    // skip empty scans 跳过点数不够用的scan
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
      continue; // 结尾索引-开头索引 = scan点数 - 1 <= 2*曲率邻域大小 则视为点数不够多
    }

    // Quick&Dirty fix for relative point time calculation without IMU data 无IMU数据时计算点相对时间的“快而脏”方法
    /*float scanSize = scanEndIdx - scanStartIdx + 1;  // 当前scan点数
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {  // 对当前scan每个点
      _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize; // 相对时间 = scan周期 * (当前点在scan中的索引 / scan点数)；再加scan序号构成intensity
    }*/

    // reset scan buffers 重置scan缓冲器，并且把不可靠的点标记为“周围已选”以将其排除出特征点提取
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions 对特征点的提取要在划分的等大区域内分别进行；默认6个区域
    for (int j = 0; j < _config.nFeatureRegions; j++) { // j：特征区域循环索引
      /** 等分区域的开始点和结束点去除了最前面和最后面curvatureRegion个点，因为那些点周围点不够，不借用其它scan的点便无法用统一公式计算曲率(c值)，而跨scan计算是不允许的
       * 计算起点终点位置：可看作使用线性加权平均
       * 第j个等分区域的起点 sp = 开始点 + (结束点-开始点)*(j/区域数) = 开始点 * (区域数-j)/区域数 + 结束点 * j/区域数
       * 第j个等分区域的终点 ep = 第(j+1)个区域的开始点 - 1 = 开始点 + (结束点-开始点)*((j+1)/区域数) - 1 = 开始点 * (区域数-j-1)/区域数 + 结束点 * (j+1)/区域数 - 1
       */
      size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j)
                   + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

      // skip empty regions 终点索引 <= 起点索引，则说明region为空，跳过该region
      if (ep <= sp) {
        continue;
      }

      // 当前region内点的个数
      size_t regionSize = ep - sp + 1;

      // reset region buffers 重置当前region缓冲器，计算region内各点曲率，并把点按曲率从小到大排序(冒泡排序)
      setRegionBuffersFor(sp, ep);


      // extract corner features 提取特征角点
      int largestPickedNum = 0; // 计数器：作为锋锐角点选出的曲率最大点
      for (size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;) {  // 找最大的，需从后往前；直到区域点遍历完，或该区域角点特征点名额用完
        size_t idx = _regionSortIndices[--k]; // 先自减，再提取；提取出该点在点云中的索引
        size_t scanIdx = idx - scanStartIdx;  // 该点在所在scan中的索引号
        size_t regionIdx = idx - sp;          // 该点在所在region中的索引号

        // 若没被标记为“邻点已选”，并且曲率大于阈值，则选为角点特征点
        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold) {

          largestPickedNum++;  // 曲率最大点数目+1
          // 若锋锐角点名额没用完，则选为锋锐角点，并且进入较不锋锐角点，否则只选为较不锋锐角点
          if (largestPickedNum <= _config.maxCornerSharp) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp.push_back(_laserCloud[idx]);
          } else {
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          }
          _cornerPointsLessSharp.push_back(_laserCloud[idx]);
          // 如果被选为特征点，则将其周围点标记为“邻点已选”
          markAsPicked(idx, scanIdx);
        }
      }

      // extract flat surface features 提取平坦平面特征点
      int smallestPickedNum = 0; // 计数器：作为平坦平面点选出的曲率最小点
      for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++) {  // 找最大的，需从前往后；直到区域点遍历完，或该区域平坦平面特征点名额用完
        size_t idx = _regionSortIndices[k];   // 提取出该点在点云中的索引
        size_t scanIdx = idx - scanStartIdx;  // 该点在所在scan中的索引号
        size_t regionIdx = idx - sp;          // 该点在所在region中的索引号

        // 若没被标记为“邻点已选”，并且曲率小于阈值，则选为平面特征点
        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {

          smallestPickedNum++;  // 曲率最小点数目+1
          // 选为平坦平面点
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(_laserCloud[idx]);
          // 如果被选为特征点，则将其周围点标记为“邻点已选”
          markAsPicked(idx, scanIdx);
        }
      }

      // extract less flat surface features 提取较不平坦平面特征点：平坦平面点和剩余点全部进入较不平坦平面点云
      for (int k = 0; k < regionSize; k++) {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }
    } // END for (feature regions)

    // down size less flat surface point cloud of current scan 每个scan处理末尾：精简(下采样)较不平坦平面特征点点云
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;   // 存储精简(下采样)后的较不平坦平面点点云
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;              // 体素栅格滤波器
    downSizeFilter.setInputCloud(surfPointsLessFlatScan); // 滤波器输入
    downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize); // 滤波器体素尺寸参数
    downSizeFilter.filter(surfPointsLessFlatScanDS);  // 执行滤波器，输出到引用的参数

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;   // 精简(下采样)后的点云叠加到该sweep的较不平坦平面点点云中
  } // END for (individual scans)
}



void BasicScanRegistration::updateIMUTransform()
{
  // sweep开始时刻的yaw-pitch-roll欧拉角
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  // 最新(sweep最后一个点)的yaw-pitch-roll欧拉角
  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  // 非线性运动导致的累积漂移误差(相对sweep开始时)：旋转到sweep开始时的IMU坐标系中记录；点向负方向旋转相当于坐标系向正方向旋转使点位于新坐标系中
  Vector3 imuShiftFromStart = _imuPositionShift;
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  // IMU速度变化(相对sweep开始时)：旋转到sweep开始时的IMU坐标系中记录
  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();
}


void BasicScanRegistration::setRegionBuffersFor(const size_t& startIdx, const size_t& endIdx)
{
  // resize buffers 配置缓冲器尺寸
  size_t regionSize = endIdx - startIdx + 1;  // 当前region内点数
  _regionCurvature.resize(regionSize);  // 存储曲率的缓冲器大小 = 当前region内点数
  _regionSortIndices.resize(regionSize);  // 存储排序后点索引的缓冲器大小 = 当前region内点数
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT); // 存储特征点类别的缓冲器大小 = 当前region内点数，默认特征类别 = 较不平坦平面点

  // calculate point curvatures and reset sort indices 计算各点曲率，重置排序索引
  float pointWeight = -2 * _config.curvatureRegion;  // 当前点在曲率计算中的权重：用了2*curvatureRegion个点，每个都需要一份中心点坐标做差，共2*curvatureRegion份

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) { // i：区域内点在scan点云中的循环索引；regionIdx：区域内点相对于本区域首个点的循环索引，即region内索引
    // 提取当前点的XYZ坐标，并乘权重
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;
    // 和两侧各curvatureRegion个点做差
    for (int j = 1; j <= _config.curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;  // 根据论文公式(1)，取2-范数当作曲率值；已经是做差得到的相对值，不需要再除以距离归一化
    _regionSortIndices[regionIdx] = i;  // 按区域内索引号顺序，记录下点在scan点云中的索引号，便于提取
  }

  // sort point curvatures 把点按c值从小到大排序(冒泡排序)
  for (size_t i = 1; i < regionSize; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {  // 提取曲率值用region内索引号
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);  // 记录大小顺序的是索引buffer的顺序；c值buffer内顺序不变；用索引buffer内的值作为c值buffer的索引，找对应c值
      }
    }
  }
}


void BasicScanRegistration::setScanBuffersFor(const size_t& startIdx, const size_t& endIdx)
{
  // resize buffers 配置缓冲器大小
  size_t scanSize = endIdx - startIdx + 1;  // scan尺寸 = 结尾索引 - 开头索引 + 1
  _scanNeighborPicked.assign(scanSize, 0);  // 分配各点的“相邻点是否被选”标记的占用空间(std::vector<int>)，初始值为0(即false)

  // mark unreliable points as picked 将不可靠的点标记为“邻点已选”以将其排除
  for (size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++) {
    const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]); // 前一个点
    const pcl::PointXYZI& point = (_laserCloud[i]);             // 当前点
    const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);     // 后一个点

    float diffNext = calcSquaredDiff(nextPoint, point); // 当前点和后一个点的距离的平方

    /** 针对论文的Fig.4(b)情况（被遮挡区域的角点）
     * 若除以深度使长度归一化后的两点距离(weighted_distance)过近，则说明归一化的两条边组成的等腰三角形的顶角过小，二者更有可能有前后遮挡关系
     * 阈值要求<0.1对应归一化等腰三角形的顶角要求<5.732度 = 2 * arcsin( (0.1/2) / 1 )
     * 计算weighted_distance时，实现时选择分情况(理论结果相同)，把较远点拉近计算，减少上溢？？？
     */
    if (diffNext > 0.1) {   // 若当前点和后一个点的距离的平方 > 0.1，即彼此有一定距离，更可能位于两个互为遮挡关系的平面上，而非连续平面上的两个距离不同的点
      float depth1 = calcPointDistance(point);      // 当前点的深度
      float depth2 = calcPointDistance(nextPoint);  // 后一个点的深度

      if (depth1 > depth2) {  // 若当前点比后一个点更远，则当前点可能是被遮挡区域的角点
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1) {  // 若距离过近，则在当前scan的_scanNeighborPicked中将当前点及之前的curvatureRegion个点都标记为“邻点已选”以排除
          std::fill_n(&_scanNeighborPicked[i - startIdx - _config.curvatureRegion], _config.curvatureRegion + 1, 1);

          continue; // 由于之前的点被标记排除了，所以不用再计算前一个点和看(a)情况了，直接处理下一个点
        }
      } else {                // 若当前点比后一个点更近，则后一个点可能是被遮挡区域的角点
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1) {  // 若距离过近，则在当前scan的_scanNeighborPicked中将后一个点及之后的curvatureRegion个点都标记为“邻点已选”以排除
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _config.curvatureRegion + 1, 1);
        }
      }
    }

    float diffPrevious = calcSquaredDiff(point, previousPoint); // 当前点和前一个点的距离的平方
    float dis = calcSquaredPointDistance(point); // 当前点的深度的平方

    /** 针对论文的Fig.4(a)情况（和激光射线接近平行的平面上的点），还针对某些强烈凹凸点和空旷区域上的孤立点
     * 若“和前一个点的距离平方”以及“和后一个点的距离平方”都大于某个“和深度的平方成正比(这个‘正比’把边比值条件引入三角形，从而限制了角度)的阈值”，则视为(离群的)不可靠点
     * 因为：对两点，对应两条射线夹角相同的情况下，两点间距离越大，代表过两点的平面和射线的夹角(锐角)越小
     * 即两点所在平面越接近和射线平行，则越有可能是不可靠点
     * 
     * 0.0002比“平面片法线和激光束夹角80度，扫描分辨率0.25度”时正弦定理推导出的系数还小一个数量级，说明论文上说的条件可能太宽松了，实际工程容忍距离阈值更小
     */
    if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  } // END for (points (in the scan))
}



void BasicScanRegistration::markAsPicked(const size_t& cloudIdx, const size_t& scanIdx)
{
  _scanNeighborPicked[scanIdx] = 1; // 该点本身设为“已选”

  // 对于该点后的curvatureRegion个点：
  for (int i = 1; i <= _config.curvatureRegion; i++) {
    // 如果它和自己的前一个点间的距离足够大，则保留选为另一特征点的可能
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }
    // 否则标记为“邻点已选”
    _scanNeighborPicked[scanIdx + i] = 1;
  }

  // 对于该点前的curvatureRegion个点：
  for (int i = 1; i <= _config.curvatureRegion; i++) {
    // 如果它和自己的后一个点间的距离足够大，则保留选为另一特征点的可能
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
      break;
    }
    // 否则标记为“邻点已选”
    _scanNeighborPicked[scanIdx - i] = 1;
  }
}


}
