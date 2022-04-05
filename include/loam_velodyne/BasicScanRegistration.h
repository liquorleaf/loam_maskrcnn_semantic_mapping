#pragma once

#include <utility>
#include <vector>

#include <pcl/point_cloud.h>

#include "Angle.h"
#include "Vector3.h"
#include "CircularBuffer.h"
#include "time_utils.h"

/** 扫描的配准(基础) 头文件
 * IndexRange类型：一对整数，起点和终点
 * PointLabel枚举：特征点的种类
 * RegistrationParams类：扫描(scan)的参数
 * IMUState类：IMU状态数据
 * BasicScanRegistration类：
 */

namespace loam
{

  // IndexRange类型：一个整数对，功能是用起始和终点描述一个范围
  /** \brief A pair describing the start and end index of a range. */
  typedef std::pair<size_t, size_t> IndexRange;


  // 特征点种类枚举：角点是2，不锋锐一些的角点是1，不平坦一点的平面点是0，平面点是-1
  /** Point label options. */
  enum PointLabel
  {
    CORNER_SHARP = 2,       ///< sharp corner point
    CORNER_LESS_SHARP = 1,  ///< less sharp corner point
    SURFACE_LESS_FLAT = 0,  ///< less flat surface point
    SURFACE_FLAT = -1       ///< flat surface point
  };


  // 扫描的参数：共9个
  /** Scan Registration configuration parameters. */
  class RegistrationParams
  {
  public:
    /** 构造函数，实现在cpp里 */
    RegistrationParams(const float& scanPeriod_ = 0.1,
      const int& imuHistorySize_ = 200,
      const int& nFeatureRegions_ = 6,
      const int& curvatureRegion_ = 5,
      const int& maxCornerSharp_ = 2,
      const int& maxSurfaceFlat_ = 4,
      const float& lessFlatFilterSize_ = 0.2,
      const float& surfaceCurvatureThreshold_ = 0.1);

    /** The time per scan.
     *  每个扫描的时间长度(秒)，默认0.1秒
     */
    float scanPeriod;

    /** The size of the IMU history state buffer.
     * IMU历史状态缓存器的尺寸，默认200
     */
    int imuHistorySize;

    /** The number of (equally sized) regions used to distribute the feature extraction within a scan. 
     * 同一次扫描分成的(等大小的)区域个数。用于使提取的特征点分散，默认6
     */
    int nFeatureRegions;

    /** The number of surrounding points (+/- region around a point) used to calculate a point curvature.
     * 用于计算一个点曲率的附近点(即该点周围的+/-区域？)的个数，默认5
     */
    int curvatureRegion;

    /** The maximum number of sharp corner points per feature region.
     * 允许每个特征区域贡献的最大锋锐角点数目，默认2
     */
    int maxCornerSharp;

    /** The maximum number of less sharp corner points per feature region.
     * 允许每个特征区域贡献的最大角点数目，是maxCornerSharp的10倍，即默认20
     */
    int maxCornerLessSharp;

    /** The maximum number of flat surface points per feature region.
     * 允许每个特征区域贡献的最大平坦平面点数目，默认4
     */
    int maxSurfaceFlat;

    /** The voxel size used for down sizing the remaining less flat surface points.
     * 用于精简(下采样)平面点点云的体素的尺寸，默认0.2
     */
    float lessFlatFilterSize;

    /** The curvature threshold below / above a point is considered a flat / corner point.
     * 划分c值在其下/其上则分别为平面/角点的阈值，默认0.1
     */
    float surfaceCurvatureThreshold;
  };


  /** IMU state data. 
   * IMU状态：时间戳+6个数据(3欧拉角+位移+速度+加速度)
   */
  typedef struct IMUState
  {
    //*****时间戳*****

    /** The time of the measurement leading to this state (in seconds).
     * 导致当前状态的测量的时间点(秒)
     */
    Time stamp;

    //*****以下三个：欧拉角*****

    /** The current roll angle.
     * 当前状态：滚转角(roll angle)
     */
    Angle roll;

    /** The current pitch angle.
     * 当前状态：俯仰角(pitch angle)
     */
    Angle pitch;

    /** The current yaw angle.
     * 当前状态：偏航角(yaw angle)
     */
    Angle yaw;

    //*****以下三个：平移状态*****

    /** The accumulated global IMU position in 3D space.
     * 在三维空间中的累积全局IMU位置
     */
    Vector3 position;

    /** The accumulated global IMU velocity in 3D space.
     *  在三维空间中的累积全局IMU速度
     */
    Vector3 velocity;

    /** The current (local) IMU acceleration in 3D space.
     *  在三维空间中的当前(局部)IMU加速度
     */
    Vector3 acceleration;

    //*****成员函数*****

    /** \brief Interpolate between two IMU states.
    * 在两个IMU状态间线性插值的静态成员函数。结果用直接引用方式传到一个参数表中的参数中。离start状态的值和end状态的值的距离比是ratio:(1-ratio)
    * @param start the first IMUState
    * @param end the second IMUState
    * @param ratio the interpolation ratio
    * @param result the target IMUState for storing the interpolation result
    */
    static void interpolate(const IMUState& start,
      const IMUState& end,
      const float& ratio,
      IMUState& result)
    {
      float invRatio = 1 - ratio;
      // 滚转角和俯仰角：直接线性插值，离start状态的值和end状态的值的距离比是ratio:(1-ratio)
      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
      // 偏航角：线性插值，离start状态的值和end状态的值的距离比是ratio:(1-ratio)
      // 总是从两点间的劣弧一侧插值：当两角相差为优角时，若后一状态的偏航角更小/大，则使其加/减2pi，再插值
      if (start.yaw.rad() - end.yaw.rad() > M_PI)
      {
        result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() + 2 * M_PI) * ratio;
      }
      else if (start.yaw.rad() - end.yaw.rad() < -M_PI)
      {
        result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() - 2 * M_PI) * ratio;
      }
      else
      {
        result.yaw = start.yaw.rad() * invRatio + end.yaw.rad() * ratio;
      }
      // 速度和位置：直接线性插值，离start状态的值和end状态的值的距离比是ratio:(1-ratio)
      result.velocity = start.velocity * invRatio + end.velocity * ratio;
      result.position = start.position * invRatio + end.position * ratio;
    };
  } IMUState; // 立即创建一个IMU状态对象

  /** BasicScanRegistration类：扫描配准的基础方法，除几个提取private成员变量的方法外，成员函数实现都在cpp里
   * public:
   *  processScanlines：            处理一个新点云(点云形式是扫描线的向量)
   *  configure：                   为扫描配置参数
   *  updateIMUData：               更新IMU状态。注意：使用引用传递，会改变参数变量
   *  projectPointToStartOfSweep：  使用对应的IMU数据，把一个点投影到一个sweep开始时刻
   * private:
   *  hasIMUData, setIMUTransformFor, transformToStartIMU, reset, extractFeatures, 
   *  setRegionBuffersFor, setScanBuffersFor, markAsPicked, interpolateIMUStateFor, updateIMUTransform
   */
  class BasicScanRegistration
  {
  public:
    /** \brief Process a new cloud as a set of scanlines.
     * 处理一个由一组scan构成的新点云(点云形式是pcl::PointCloud<pcl::PointXYZI>的vector，每一元素是一个扫描面)，提取特征点，更新IMU位姿信息
     * @param relTime the time relative to the scan time
     */
    void processScanlines(const Time& scanTime, std::vector<pcl::PointCloud<pcl::PointXYZI>> const& laserCloudScans); // 没有*，则const出现位置无影响

    /** 使用RegistrationParams对象为扫描配置参数(只有return true) */
    bool configure(const RegistrationParams& config = RegistrationParams()); 

    /** \brief Update new IMU state. NOTE: MUTATES ARGS!
     * 使用IMU的上一个状态(若可用，否则直接加入，作为初始参考)和(在此处转换到世界坐标系的)加速度数据估算新状态(世界坐标系下的速度和位置)，并加入IMU状态缓冲器
     * 注意：使用引用传递，会改变参数变量
     * @param acc 加速度
     * @param newState IMU状态
     */
    void updateIMUData(Vector3& acc, IMUState& newState);

    /** \brief Project a point to the start of the sweep using corresponding IMU data
    * 使用对应的IMU数据，把一个点投影到一个sweep开始时刻
    * @param point The point to modify
    * @param relTime The time to project by
    */
    void projectPointToStartOfSweep(pcl::PointXYZI& point, float relTime);

    //以下：提取private数据成员的方法

    auto const& imuTransform          () { return _imuTrans             ; }
    auto const& sweepStart            () { return _sweepStart           ; }
    auto const& laserCloud            () { return _laserCloud           ; }
    auto const& cornerPointsSharp     () { return _cornerPointsSharp    ; }
    auto const& cornerPointsLessSharp () { return _cornerPointsLessSharp; }
    auto const& surfacePointsFlat     () { return _surfacePointsFlat    ; }
    auto const& surfacePointsLessFlat () { return _surfacePointsLessFlat; }
    auto const& config                () { return _config               ; }

  private:

    // 以下工具函数以private封装

    /** \brief Check is IMU data is available.
     * 检查IMU数据缓存器中是否有数据可用(size是否>0)
     */
    inline bool hasIMUData() { return _imuHistory.size() > 0; };

    /** \brief Set up the current IMU transformation for the specified relative time.
     * 尝试插值算得给定时间点(相对scan时间)的IMU位姿变换，并将该位姿变换设置给该时间点
     * @param relTime the time relative to the scan time
     */
    void setIMUTransformFor(const float& relTime);

    /** \brief Project the given point to the start of the sweep, using the current IMU state and position shift.
     * 使用当前IMU状态和位移，把给定点投影到sweep开始时的坐标系内(消除非线性运动的影响后，sweep内IMU坐标系的旋转不变，只有平移)
     * @param point the point to project
     */
    void transformToStartIMU(pcl::PointXYZI& point);

    /** \brief Prepare for next scan / sweep.
     * 重新初始化、清理缓冲器，准备下一个scan/sweep
     * @param scanTime the current scan time
     * @param newSweep indicator if a new sweep has started (没出现)
     */
    void reset(const Time& scanTime);

    /** \brief Extract features from current laser cloud.
     * 从当前点云提取特征点
     * @param beginIdx the index of the first scan to extract features from “将被提取特征的第一个scan”的索引，默认为0
     */
    void extractFeatures(const uint16_t& beginIdx = 0);

    /** \brief Set up region buffers for the specified point range.
     * 为特定索引范围的点设置各个region缓冲器，计算其中点的曲率(c值)，并根据曲率从小到大排序
     * @param startIdx the region start index
     * @param endIdx the region end index
     */
    void setRegionBuffersFor(const size_t& startIdx,
      const size_t& endIdx);

    /** \brief Set up scan buffers for the specified point range.
     * 根据开始和结束索引设置scanNeighborPicked缓冲器，并且把不可靠的点标记为“周围已选”以将其排除特征点提取
     * @param startIdx the scan start index
     * @param endIdx the scan start index
     */
    void setScanBuffersFor(const size_t& startIdx,
      const size_t& endIdx);

    /** \brief Mark a point and its neighbors as picked.
     *
     * This method will mark neighboring points within the curvature region as picked,
     * as long as they remain within close distance to each other.
     * 标记工具：只要已选特征点和它曲率计算邻域中的点保持彼此距离很近，就给后者标记“周围点已被选”
     * @param cloudIdx the index of the picked point in the full resolution cloud
     * @param scanIdx the index of the picked point relative to the current scan
     */
    void markAsPicked(const size_t& cloudIdx,
      const size_t& scanIdx);

    /** \brief Try to interpolate the IMU state for the given time.
     * 尝试插值得到特定时间点(相对scan时间)的IMU状态
     * @param relTime the time relative to the scan time
     * @param outputState the output state instance
     */
    void interpolateIMUStateFor(const float& relTime, IMUState& outputState);

    /** 每个sweep更新IMU位姿变换信息(存储到具有一定格式的_imuTrans中，准备作为ros message发送；如果无IMU，这里无实际作用) */
    void updateIMUTransform();

  private:
    RegistrationParams _config;  ///< registration parameter

    pcl::PointCloud<pcl::PointXYZI> _laserCloud;   ///< full resolution input cloud 全分辨率的输入点云
    std::vector<IndexRange> _scanIndices;          ///< start and end indices of the individual scans within the full resolution cloud 全分辨率点云中各scan的起始和终止索引

    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;      ///< sharp corner points cloud 锋锐角点的点云
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;  ///< less sharp corner points cloud 所有角点的点云
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;      ///< flat surface points cloud 平坦平面点的点云
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat;  ///< less flat surface points cloud 所有平面点的点云

    Time _sweepStart;            ///< time stamp of beginning of current sweep 当前sweep的开始时间戳
    Time _scanTime;              ///< time stamp of most recent scan 最近一次scan的时间戳
    IMUState _imuStart;                     ///< the interpolated IMU state corresponding to the start time of the currently processed laser scan 其实是“当前处理的sweep的第一个scan开始时间”对应的IMU状态(插值所得)
    IMUState _imuCur;                       ///< the interpolated IMU state corresponding to the time of the currently processed laser scan point “当前处理的scan点”对应的IMU状态(插值所得)
    Vector3 _imuPositionShift;              ///< position shift between accumulated IMU position and interpolated IMU position (非线性运动时非零)IMU位置漂移(即加减速的影响)：(假设sweep开始后匀速运动所得)累积IMU位置和(插值所得)当前点时刻IMU位置间的位移
    size_t _imuIdx = 0;                         ///< the current index in the IMU history 在IMU历史状缓冲器中使用的当前索引
    CircularBuffer<IMUState> _imuHistory;   ///< history of IMU states for cloud registration 存储IMU历史状态的队列

    pcl::PointCloud<pcl::PointXYZ> _imuTrans = { 4,1 };  ///< IMU transformation information 创建宽4高1的XYZ点云(等效于4*1*3的数组)，用于存放IMU位姿变换信息，具体为：_imuStart的3个欧拉角、_imuCur的3个欧拉角、imuShiftFromStart的三维坐标、imuVelocityFromStart的三维坐标

    std::vector<float> _regionCurvature;      ///< point curvature buffer 存储各点曲率(c值)
    std::vector<PointLabel> _regionLabel;     ///< point label buffer 存储各点特征类型
    std::vector<size_t> _regionSortIndices;   ///< sorted region indices based on point curvature 根据点曲率排序后的区域索引
    std::vector<int> _scanNeighborPicked;     ///< flag if neighboring point was already picked 相邻点是否已被选为特征点的flag；相邻点被选上后，原则上该点不再被选
  };

}

