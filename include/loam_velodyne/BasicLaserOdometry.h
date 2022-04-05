#pragma once
#include "Twist.h"
#include "nanoflann_pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   * LOAM激光里程计(基础)组件的实现
   */
  class BasicLaserOdometry
  {
  public:
    /** 构造函数：默认扫描周期0.1秒，默认最大迭代次数25， */
    explicit BasicLaserOdometry(float scanPeriod = 0.1, size_t maxIterations = 25);

    /** \brief Try to process buffered data. 尝试处理缓冲的数据：(第一次运行时)初始化、(迭代)(两帧间特征对应配准、位姿优化)、结果累积和坐标转换 */
    void process();
    /** 根据接收到的IMU位姿信息更新本类内部的IMU信息 */
    void updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans);

    /** 对private成员变量的基本操作：
     * 提取各种里程计内部存储和变量的方法；
     * 设置扫描周期、最大迭代次数、deltaT和deltaR迭代停止阈值的方法
     */

    auto& cornerPointsSharp()     { return _cornerPointsSharp; }
    auto& cornerPointsLessSharp() { return _cornerPointsLessSharp; }
    auto& surfPointsFlat()        { return _surfPointsFlat; }
    auto& surfPointsLessFlat()    { return _surfPointsLessFlat; }
    auto& laserCloud() { return _laserCloud; }

    auto const& transformSum() { return _transformSum; }
    auto const& transform()    { return _transform;    }
    auto const& lastCornerCloud () { return _lastCornerCloud ; }
    auto const& lastSurfaceCloud() { return _lastSurfaceCloud; }

    void setScanPeriod(float val)     { _scanPeriod    = val; }
    void setMaxIterations(size_t val) { _maxIterations = val; }
    void setDeltaTAbort(float val)    { _deltaTAbort = val;   }
    void setDeltaRAbort(float val)    { _deltaRAbort = val;   }

    auto frameCount()    const { return _frameCount;    }
    auto scanPeriod()    const { return _scanPeriod;    }
    auto maxIterations() const { return _maxIterations; }
    auto deltaTAbort()   const { return _deltaTAbort;   }
    auto deltaRAbort()   const { return _deltaRAbort;   }

    /** \brief Transform the given point cloud to the end of the sweep.
     * 把给出的点云投影到sweep结束时(下一sweep开始时)
     * @param cloud the point cloud to transform
     * @return 1
     */
    size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  private:
    /** \brief Transform the given point to the start of the sweep.
     * 把给定点投影到sweep开始时(上一sweep结束时)
     * @param pi the point to transform 不改变输入pi
     * @param po the point instance for storing the result 结果存放于po中
     */
    void transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po);

    /** IMU优化欧拉角：修正sweep内转向引起的非线性运动失真 */
    void pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                           const Angle& blx, const Angle& bly, const Angle& blz,
                           const Angle& alx, const Angle& aly, const Angle& alz,
                           Angle &acx, Angle &acy, Angle &acz);

    /** 累积旋转：之前的累积旋转矩阵·帧内旋转矩阵 = 新累积旋转矩阵，再用欧拉角旋转矩阵->旋转角公式算出总旋转角 */
    void accumulateRotation(Angle cx, Angle cy, Angle cz,
                            Angle lx, Angle ly, Angle lz,
                            Angle &ox, Angle &oy, Angle &oz);

  private:

    float _scanPeriod;       ///< time per scan 扫描周期
    long _frameCount;        ///< number of processed frames 处理过的帧数
    size_t _maxIterations;   ///< maximum number of iterations 最大迭代次数
    bool _systemInited;      ///< initialization flag 是否已初始化的flag

    float _deltaTAbort;     ///< optimization abort threshold for deltaT 优化中deltaT的迭代终止阈值
    float _deltaRAbort;     ///< optimization abort threshold for deltaR 优化中deltaR的迭代终止阈值

    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud;    ///< last corner points cloud 上一次的角点云(指针)
    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud;   ///< last surface points cloud 上一次的平面点云(指针)

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;      ///< point selection 存放找到匹配并计算了距离的特征点(指针)；注意：记录的是没有投影到sweep开始前的自身时间点坐标系的坐标
    pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;           ///< point selection coefficients 存放找到匹配并计算了距离的特征点的已加权偏导数和距离(指针)；注意：是用投影到sweep开始时的坐标计算的

    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree 上一次的角点云(存储于KD树，查找效率更高)
    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree 上一次的平面点云(存储于KD树，查找效率更高)

    /** 输入点云分类别存储 */
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud 锋锐角点云(指针)
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud 所有角点云(指针)
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat;         ///< flat surface points cloud 平坦平面点云(指针)
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud 所有平面点云(指针)
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud 全分辨率点云(指针)

    /** 角点查找索引缓冲器 */
    std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer 角点查找索引缓冲器1：最邻近点
    std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer 角点查找索引缓冲器2：平方距离最小点

    /** 平面点查找索引缓冲器 */
    std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer 平面点查找索引缓冲器1
    std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer 平面点查找索引缓冲器2
    std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer 平面点查找索引缓冲器3

    /** 最优化方法估计的位姿变换 */
    Twist _transform;     ///< optimized pose transformation 本次sweep(这一帧)的开始到结尾的最优六自由度位姿变换(指lidar位姿变换后lidar坐标系下点的坐标变换)；每次里程计运行开始时，用上一帧的结果当作优化的初始guess
    Twist _transformSum;  ///< accumulated optimized pose transformation 累积最优六自由度位姿变换

    /** IMU位姿信息 */
    Angle _imuRollStart, _imuPitchStart, _imuYawStart;  // IMU在sweep起始时状态的欧拉角
    Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;        // IMU在sweep终止时状态的欧拉角

    Vector3 _imuShiftFromStart;                         // IMU相对sweep起始时的非线性运动位置漂移
    Vector3 _imuVeloFromStart;                          // IMU相对sweep起始时的速度变化
  };

} // end namespace loam
