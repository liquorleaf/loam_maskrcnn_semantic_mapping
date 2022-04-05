// 2022, Li Longwei, Sun Yet-sen University

#pragma once

#include <vector>
#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>

#include "Twist.h"
#include "common.h"
#include "nanoflann_pcl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

namespace loam
{

constexpr float DEFAULT_CONFI_THRE = 0.5f;  // 置信概率阈值的默认值
constexpr float DEFAULT_MASK_THRE  = 0.5f;  // 掩膜创建阈值的默认值

constexpr double BOUND_BOX_COLOR_R = 255.0;  // 目标检测框的颜色的R分量
constexpr double BOUND_BOX_COLOR_G = 178.0;  // 目标检测框的颜色的G分量
constexpr double BOUND_BOX_COLOR_B = 50.0;   // 目标检测框的颜色的B分量
constexpr int BOUND_BOX_THICKNESS = 3;       // 目标检测框的线宽

constexpr int LABEL_FONT = cv::FONT_HERSHEY_SIMPLEX;  // 目标标签的字体名称
constexpr int LABEL_THICKNESS = 2;                    // 目标标签的字体粗细
constexpr double LABEL_SCALE = 1.0;                   // 目标标签的字体缩放比例
constexpr double LABEL_HOR_PAD_SCALE = 0.05;          // 目标标签水平方向上左/右边框和文字间空白占文字总长的比例
constexpr double LABEL_HOR_SCALE_TOTAL = 1.0 + 2.0 * LABEL_HOR_PAD_SCALE; // 目标标签水平方向长度占文字总长的比例
constexpr double LABEL_COLOR_R = .0;                  // 目标标签的字体颜色的R分量
constexpr double LABEL_COLOR_G = .0;                  // 目标标签的字体颜色的G分量
constexpr double LABEL_COLOR_B = .0;                  // 目标标签的字体颜色的B分量
constexpr double LABEL_BOX_COLOR_R = 255.0;           // 目标标签的填充颜色的R分量
constexpr double LABEL_BOX_COLOR_G = 255.0;           // 目标标签的填充颜色的G分量
constexpr double LABEL_BOX_COLOR_B = 255.0;           // 目标标签的填充颜色的B分量

constexpr double MASK_COLOR_RATIO = 0.3;                        // 目标分割结果上色时特征色的比例 
constexpr double MASK_COLOR_RATIO_INV = 1 - MASK_COLOR_RATIO;   // 目标分割结果上色时原像素的比例 
constexpr int MASK_CONTOUR_THICKNESS = 5;                       // 目标分割结果边界绘制宽度
constexpr int MASK_MAX_LEVEL = 100;                             // 目标分割结果边界绘制时最大追溯拓扑结构层数

constexpr float CAM_DEPTH_SCALE = 1000.0f;  // 深度相机的像素值比例尺

constexpr float DEFAULT_MATCH_SQ_DIST_THRE = 0.01f; // 认为lidar点云点和rgbd语义实例匹配成功的距离(米)的平方的默认值


/** \brief 类：LOAM语义建图的具体基础方法 */
class BasicSemanticMapping
{
public:
    /** 构造函数 */
    explicit BasicSemanticMapping(const float& confidenceThreshold = DEFAULT_CONFI_THRE, const float& maskThreshold = DEFAULT_MASK_THRE, const float& matchSqDistThreshold = DEFAULT_MATCH_SQ_DIST_THRE);

    /** 尝试处理缓冲的数据 */
    void process();
    /** 用接收到的最新位姿变换更新_transformAftMapped */
    void updateTransform(double pitch, double yaw, double roll, double x, double y, double z);

    /** 取得或设置各成员的基本函数 */

    auto& laserCloudFullRes() { return *_laserCloudFullRes; }
    auto& rgbImageData() { return _rgbImageData; }
    auto& depthImageData() { return _depthImageData; }
    auto& rgbImageSegmented() { return _rgbImageSegmented; }

    auto const& semanticCloud()           const { return *_semanticCloud; }
    auto const& semanticMapCloudFullRes() const { return *_semanticMapCloudFullRes; }
    auto const& transformAftMapped()      const { return _transformAftMapped; }
    
    void setRGBImageEncoding(const std::string& rgbImageEncoding) { _rgbImageEncoding = rgbImageEncoding; }
    void setDepthImageEncoding(const std::string& depthImageEncoding) { _depthImageEncoding = depthImageEncoding; }
    void setConfidenceThreshold(const float& confidenceThreshold) { _confidenceThreshold = confidenceThreshold; }
    void setMaskThreshold(const float& maskThreshold) { _maskThreshold = maskThreshold; }
    void setMaxMatchingSquareDistance(const float& maxMatchingSquareDistance) { _maxMatchingSquareDistance = maxMatchingSquareDistance; }

private:  // 成员函数

    /** 提取分割结果掩膜，并绘制实例分割结果展示图 */
    void postSegProcess(cv::Mat &frame, const std::vector<cv::Mat> &outs);
    /** 在postSegProcess中调用：在图上画检测识别框和分割结果掩膜 */
    void drawBoxAndMask(cv::Mat &frame, int classId, float conf, cv::Rect &box, cv::Mat &segMask);


private:  // 成员变量

    /** 激光点语义标注和建图 */
    Time _laserCloudTime;    ///< 激光点云的时间戳，TODO: 可供IMU插值使用

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;        ///< 本次mapping的全分辨率点云(世界坐标系)
    pcl::PointCloud<pcl::PointXYZL>::Ptr _semanticCloud;            ///< 本次mapping的语义点云(世界坐标系)
    pcl::PointCloud<pcl::PointXYZL>::Ptr _semanticMapCloudFullRes;  ///< 累积语义全分辨率点云(世界坐标系)
        
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _instanceCloud;     ///< 实例点云的向量
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _instanceCloudDS;   ///< 下采样后实例点云的向量
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterInstance;               ///< 下采样实例点云的体素栅格滤波器
    nanoflann::KdTreeFLANN<pcl::PointXYZI>* _instanceKDTree;              ///< 实例点云的向量(存储于KD树，查找效率更高)

    float _maxMatchingSquareDistance;  ///< lidar点云和实例点匹配时，认为匹配成功的最大距离，超出则匹配失败

    Twist _transformAftMapped;  ///< 上次全局优化后的累积位姿；包含(roll,pitch,yaw,t)即(rz,rx,ry,tx,ty,tz)使得：世界坐标 = R_ZXY·lidar坐标 + t

    /** 图像处理：Mask R-CNN等 */
    cv::dnn::Net _MaskrcnnNet;              ///< Mask R-CNN网络对象
    std::vector<cv::Mat> _outs;             ///< 网络输出
    std::vector<std::string> _outNames;     ///< 要提取的网络输出的名称

    float _confidenceThreshold;     ///< 置信概率阈值 Confidence threshold
    float _maskThreshold;           ///< 掩膜创建阈值 Mask threshold

    std::vector<std::string> _classNames;   ///< 存放分类名称字符串的向量
    std::vector<cv::Scalar> _colors;        ///< 存放值着色色号数值的向量
    std::string _classNamesFile;            ///< 文件地址：分类名称
    std::string _colorsFile;                ///< 文件地址：着色色号
	std::string _modelConfigFile;           ///< 文件地址：模型参数
	std::string _modelWeightsFile;          ///< 文件地址：模型权重

    int _numMaskDetections;                 ///< 当前帧“置信概率超过阈值，从而决定为其创建掩膜”的目标数
    std::vector<cv::Rect> _objectBox;       ///< 存放当前帧创建了掩膜的目标的识别框
    std::vector<cv::Mat> _objectSegMask;    ///< 存放当前帧创建了掩膜的目标的(识别框内)掩膜
    std::vector<int> _objectClassId;        ///< 存放当前帧创建了掩膜的目标的分类

    cv::Mat _rgbImageData;            ///< rgb图像像素数据
    cv::Mat _depthImageData;          ///< 深度图像素数据
    std::string _rgbImageEncoding;    ///< rgb图像编码格式，opencv处理时为"bgr8"
    std::string _depthImageEncoding;  ///< 深度图像编码格式
    cv::Mat _blob;                    ///< 
    cv::Mat _rgbImageSegmented;       ///< rgb图像实例分割结果像素数据

    /** 传感器 */
    std::string _camIntrinsicFile;            ///< 文件地址：相机内参yaml
    std::string _camLidarExtrinsicFile;       ///< 文件地址：相机-lidar外参yaml

    cv::Mat _cameraMatrix;             ///< 相机内参：相机坐标到像素坐标的变换参数K
    cv::Mat _cameraMatrixINV;          ///< 相机内参：像素坐标到相机坐标的变换参数K^(-1)
    std::string _distortionModel;      ///< 相机内参：畸变模型
    cv::Mat _distortionCoeff;          ///< 相机内参：畸变参数D

    cv::Mat _camLidarTrans;             ///< 相机-lidar外参：位姿变换
};


} // END namespace loam
