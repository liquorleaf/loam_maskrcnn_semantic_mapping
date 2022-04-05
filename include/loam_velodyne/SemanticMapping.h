// 2022, Li Longwei, Sun Yet-sen University

#ifndef LOAM_SEMANTICMAPPING_H
#define LOAM_SEMANTICMAPPING_H

#include "loam_velodyne/BasicSemanticMapping.h"
#include "common.h"

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace loam
{

constexpr int IMAGE_BUFFER_SIZE = 300;   // 图像message缓冲器大小(相机帧率30，loam建图输出延迟约1s，语义建图一次约)(太小无法同步，太大时空资源消耗太大)
constexpr double SYNCHRON_THRE = 0.016; // (用于判断图像和点云message同步)时间戳差值的阈值(相机30帧左右，估计每0.033秒来一次图像，一半算0.016s)

/** /brief 类：语义建图组件的实现。 */
class SemanticMapping : public BasicSemanticMapping
{
public:
    /** 构造函数 */
    explicit SemanticMapping(const float& confidenceThreshold = DEFAULT_CONFI_THRE, const float& maskThreshold = DEFAULT_MASK_THRE, const float& matchSqDistThreshold = DEFAULT_MATCH_SQ_DIST_THRE);

    /** 启动结点。
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
    virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

    /** 接收新的建图和全局优化后的全分辨率点云。
     * @param laserCloudFullResMsg the new full resolution cloud message
     */
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

    /** 接收建图和全局优化后的lidar累积位姿变换数据。
     * @param transformAftMappedMsg the new laser odometry after mapped transformations message
     */
    void transformAftMappedHandler(const nav_msgs::Odometry::ConstPtr& transformAftMappedMsg);

    /** 接收RGB图像数据。
     * @param rgbImageMsg the new RGB image message
     */
    void rgbImageHandler(const sensor_msgs::ImageConstPtr& rgbImageMsg);

    /** 接收已配准深度图数据。
     * @param depthImageMsg the new depth image message
     */
    void depthImageHandler(const sensor_msgs::ImageConstPtr& depthImageMsg);

    /** 活跃模式：循环尝试接收并处理message，直到程序关闭。 */
    void spin();

    /** 尝试处理接收到的新数据；若没有收到完整到可处理的新数据，则本次process什么也不做。 */
    void process();


protected:
   /** 重置是否接收到新的数据的flag为false。 */
   void reset();

   /** 检查执行一次语义建图所需的数据是否齐全并满足要求(时间戳基本相同)。 */
   bool hasNewData();

   /** 从指定topic发布结果。 */
   void publishResult();

private:
    /** data received */
    cv_bridge::CvImage _rgbSegmented;               ///< Mask R-CNN实例分割结果展示图(时间是本次mapping正对的sweep开始时)
    std::vector<sensor_msgs::Image> _rgbMsgBuffer;    ///< 时间同步辅助工具：rgb图像缓冲器
    std::vector<sensor_msgs::Image> _depthMsgBuffer;  ///< 时间同步辅助工具：深度图像缓冲器
    int _rgbMsgBufferHead;                            ///< rgb图像缓冲器队首(时间最早的项)位置
    int _depthMsgBufferHead;                          ///< 深度图像缓冲器队首(时间最早的项)位置

    /** data to be published */
    sensor_msgs::Image _rgbSegmentedMsg;    ///< Mask R-CNN实例分割结果展示图message

    /** time stamp */
    ros::Time _timeLaserCloudFullRes;      ///< 当前全分辨率点云的时间戳
    ros::Time _timeTransformAftMapped;     ///< 当前全局位姿变换的时间戳
    ros::Time _timeRGBImage;               ///< 当前RGBD图像的时间戳
    ros::Time _timeDepthImage;             ///< 当前深度图的时间戳

    /** new data flag */
    bool _newLaserCloudFullRes;    ///< 是否接收到全分辨率点云的flag
    bool _newTransformAftMapped;   ///< 是否接收到建图全局优化后位姿变换的flag
    bool _newRGBImage;             ///< 是否接收到RGB图像的flag
    bool _newDepthImage;           ///< 是否接收到深度图的flag

    /** publisher */
    ros::Publisher _pubSemanticMap;             ///< 发布：语义地图点云
    ros::Publisher _pubSegmentationResult;         ///< 发布：本次图像实例分割结果

    /** subscriber */
    ros::Subscriber _subLaserCloudFullRes;      ///< 订阅：全分辨率点云
    ros::Subscriber _subTransformAftMapped;     ///< 订阅：全局优化后lidar位姿变换
    ros::Subscriber _subRGBImage;               ///< 订阅：RGB图像
    ros::Subscriber _subDepthImage;             ///< 订阅：已配准深度图
};


} // END namespace loam


#endif //LOAM_SEMANTICMAPPING_H
