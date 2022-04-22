// 2022, Li Longwei, Sun Yet-sen University

#include "loam_velodyne/SemanticMapping.h"

namespace loam
{

SemanticMapping::SemanticMapping(const float& confidenceThreshold, const float& maskThreshold, const float& matchSqDistThreshold):
    BasicSemanticMapping(confidenceThreshold, maskThreshold, matchSqDistThreshold),
    _rgbMsgBufferHead(0),
    _depthMsgBufferHead(0)
{
    reset();
}


bool SemanticMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
    // fetch and check semantic mapping parameters
    float fParam;
    int iParam;

    if (privateNode.getParam("confidenceThreshold", fParam))
    {   // 置信概率阈值必须>0并且<=1
        if (fParam <= 0 || fParam > 1)
        {
            ROS_ERROR("Invalid confidenceThreshold parameter: %f (expected > 0 and <= 1)", fParam);
            return false;
        }
        else
        {
            BasicSemanticMapping::setConfidenceThreshold(fParam);
            ROS_INFO("Set confidenceThreshold: %f", fParam);
        }
    }
    if (privateNode.getParam("maskThreshold", fParam))
    {   // 掩膜创建阈值必须>0并且<=1
        if (fParam <= 0 || fParam > 1)
        {
            ROS_ERROR("Invalid maskThreshold parameter: %f (expected > 0 and <= 1)", fParam);
            return false;
        }
        else
        {
            BasicSemanticMapping::setMaskThreshold(fParam);
            ROS_INFO("Set maskThreshold: %f", fParam);
        }
    }
    if (privateNode.getParam("matchSqDistThreshold", fParam))
    {   // 点匹配距离阈值必须>0
        if (fParam <= 0)
        {
            ROS_ERROR("Invalid matchSqDistThreshold parameter: %f (expected > 0)", fParam);
            return false;
        }
        else
        {
            BasicSemanticMapping::setMaxMatchingSquareDistance(fParam);
            ROS_INFO("Set matchSqDistThreshold: %f", fParam);
        }
    }

    // advertise semantic mapping topics
    _pubSemanticMap = node.advertise<sensor_msgs::PointCloud2>("/semantic_cloud_map", 2);
    _pubSemanticMapLidarInst = node.advertise<sensor_msgs::PointCloud2>("/semantic_cloud_map_lidar_inst", 2);
    _pubSemanticMapTotal = node.advertise<sensor_msgs::PointCloud2>("/semantic_cloud_map_total", 2);
    _pubSegmentationResult = node.advertise<sensor_msgs::Image>("/instance_segmentation_result", 100);

    // subscribe to laser mapping topics
    _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
        ("/velodyne_cloud_registered", 2, &SemanticMapping::laserCloudFullResHandler, this);
    _subTransformAftMapped = node.subscribe<nav_msgs::Odometry>
        ("/aft_mapped_to_init", 5, &SemanticMapping::transformAftMappedHandler, this);

    // subscribe to camera input topics
    _subRGBImage = node.subscribe<sensor_msgs::Image>
        ("/camera/color/image_raw", 100, &SemanticMapping::rgbImageHandler, this);
    _subDepthImage = node.subscribe<sensor_msgs::Image>
        ("/camera/aligned_depth_to_color/image_raw", 100, &SemanticMapping::depthImageHandler, this);
    return true;
}


void SemanticMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
    _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
    laserCloudFullRes().clear();   // 清空接收buffer中上次的点云
    pcl::fromROSMsg(*laserCloudFullResMsg, laserCloudFullRes());
    _newLaserCloudFullRes = true;
}

void SemanticMapping::transformAftMappedHandler(const nav_msgs::Odometry::ConstPtr& transformAftMappedMsg)
{
    _timeTransformAftMapped = transformAftMappedMsg->header.stamp;

    // 从四元数形式的message中取得累积欧拉角
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = transformAftMappedMsg->pose.pose.orientation;
    // 发布前创建时有x1=-y0,y1=-z0,z1=x0，此处提取时应有x0=z1,y0=-x1,z0=-y1
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    // 该次mapping算得的累积位姿：发布前创建时四元数使用了(roll,-pitch,-yaw)，故此处提取时pitch和yaw应当取负
    updateTransform(-pitch, -yaw, roll,
                    transformAftMappedMsg->pose.pose.position.x,
                    transformAftMappedMsg->pose.pose.position.y,
                    transformAftMappedMsg->pose.pose.position.z);

    _newTransformAftMapped = true;
}

void SemanticMapping::rgbImageHandler(const sensor_msgs::ImageConstPtr& rgbImageMsg)
{
    if (_rgbMsgBuffer.size() >= IMAGE_BUFFER_SIZE)  // 到设置的容量上限，则覆盖最早的数据
    {
        _rgbMsgBuffer.at(_rgbMsgBufferHead) = (*rgbImageMsg);
        _rgbMsgBufferHead = (_rgbMsgBufferHead + 1) % IMAGE_BUFFER_SIZE;
    }
    else  // 未到设置的容量上限，则直接加入
    {
        _rgbMsgBuffer.push_back(*rgbImageMsg);
    }

    _newRGBImage = true;
}

void SemanticMapping::depthImageHandler(const sensor_msgs::ImageConstPtr& depthImageMsg)
{
    if (_depthMsgBuffer.size() >= IMAGE_BUFFER_SIZE)  // 到设置的容量上限，则覆盖最早的数据
    {
        _depthMsgBuffer.at(_depthMsgBufferHead) = (*depthImageMsg);
        _depthMsgBufferHead = (_depthMsgBufferHead + 1) % IMAGE_BUFFER_SIZE;
    }
    else  // 未到设置的容量上限，则直接加入
    {
        _depthMsgBuffer.push_back(*depthImageMsg);
    }

    _newDepthImage = true;
}


void SemanticMapping::spin()
{
    ros::Rate rate(100);  // ros::Rate帮助循环以指定的速率运行（原则上应当比数据产生频率更快？？？）
    bool status = ros::ok(); // ros是否还在继续运行；一旦ros::shutdown()被调用，变为false

    // loop until shutdown 循环直到关闭
    while (status)
    {
      ros::spinOnce();  // 执行一次回调(此后真正收到并可以处理订阅的message)

      // try processing new data 检查是否接收到新数据，并尝试处理
      process();

      status = ros::ok(); // 检查ros是否还在继续运行
      rate.sleep(); // 本次循环剩余时间里休眠(sleep还有一个判断循环速度是否满足设定的返回，这里没用到)
    }
}

void SemanticMapping::process()
{
   if (!hasNewData()) // waiting for new data to arrive...
      return;

   reset(); // reset flags, etc.

   BasicSemanticMapping::process(_cloudTime);

   publishResult();
}


void SemanticMapping::reset()
{
    _newLaserCloudFullRes = false;
    _newTransformAftMapped = false;
    _newRGBImage = false;
    _newDepthImage = false;
}
//loam建图输出的时间戳是sweep开始时的时间戳
bool SemanticMapping::hasNewData()
{  
    //std::cout<<"\n\n"<<_timeLaserCloudFullRes<<" "<<(_timeLaserCloudFullRes-_timeRGBImage)<<" "<<_timeRGBImage<<"\n\n"; // 测试用：查看各message的时间戳
    /** 只有下列条件全部满足才会返回true(开始处理)：
     * 1. 全分辨率点云、位姿变换、rgb图像、深度图像都有接收到的新数据(语义建图需要数据种类齐全)，其中位姿变换和点云的时间戳在发送是即被设为相同；
     * 2. 存在时间戳和全分辨率点云相差在阈值(秒)内的rgb图像(loam建图输出平均约1Hz，是低频方，需要高频的图像数据配合)；
     * 3. 存在和rgb图像时间戳相同的深度图像。
     */ 

    _cloudTime = _timeLaserCloudFullRes.toSec();  // 点云时间戳
    // 若数据不全，则无法处理
    if (!(_newLaserCloudFullRes && _newTransformAftMapped && _newRGBImage && _newDepthImage))
        return false;
    if (_rgbMsgBuffer.empty() || _depthMsgBuffer.empty())
        return false;
    // 若最早的rgb图像也比点云晚太多，或最晚的rgb图像也比点云早太多，则不存在足够早的能同步的图像
    if (_rgbMsgBuffer.at(_rgbMsgBufferHead).header.stamp.toSec() - _cloudTime > SYNCHRON_THRE)
        return false;
    else if (_rgbMsgBuffer.at((_rgbMsgBufferHead-1<0)?(_rgbMsgBuffer.size()-1):(_rgbMsgBufferHead-1)).header.stamp.toSec() - _cloudTime < SYNCHRON_THRE)
        return false;

    bool hasNewDataFlag = false;
    // 搜索时间相差在阈值内的rgb图像，若存在，则搜索匹配的深度图像
    for (int ind = 0; ind < _rgbMsgBuffer.size(); ind++)
    {
        _timeRGBImage = _rgbMsgBuffer.at(ind).header.stamp;
        if (abs(_timeRGBImage.toSec() - _cloudTime) <= SYNCHRON_THRE)
        {
            // 提取rgb图像及其信息
            _rgbSegmented.header.stamp = _rgbMsgBuffer.at(ind).header.stamp;
            _rgbSegmented.header.frame_id = _rgbMsgBuffer.at(ind).header.frame_id;
            _rgbSegmented.encoding = _rgbMsgBuffer.at(ind).encoding;

            setRGBImageEncoding("bgr8");    // 把rgb转成opencv使用的bgr格式
            rgbImageData() = cv_bridge::toCvCopy(_rgbMsgBuffer.at(ind), "bgr8")->image;
            rgbImageSegmented() = cv_bridge::toCvCopy(_rgbMsgBuffer.at(ind), "bgr8")->image;   // 检测结果展示图以原图为模板

            // 若深度图像出现了不匹配(小概率事件)，则搜索匹配的深度图像，若没找到则无法处理
            int indDepth = ind;
            _timeDepthImage = _depthMsgBuffer.at(indDepth).header.stamp;
            if (_timeDepthImage != _timeRGBImage)
            {
                for (indDepth = 0; indDepth < _depthMsgBuffer.size(); indDepth++)
                {
                    _timeDepthImage = _depthMsgBuffer.at(indDepth).header.stamp;
                    if (_timeDepthImage == _timeRGBImage)
                        break;
                    else if (indDepth == _depthMsgBuffer.size() - 1)
                        return false;
                }
            }
            // 提取深度图像及其信息
            std::string encoding_scheme = _depthMsgBuffer.at(indDepth).encoding.c_str();
            setDepthImageEncoding(encoding_scheme);
            depthImageData() = cv_bridge::toCvCopy(_depthMsgBuffer.at(indDepth), encoding_scheme)->image;

            hasNewDataFlag = true;
            break;
        }
    }

    return hasNewDataFlag;
}

void SemanticMapping::publishResult()
{
    // 发布实例分割结果框展示图
    _rgbSegmented.image = rgbImageSegmented();
    _rgbSegmented.toImageMsg(_rgbSegmentedMsg);
    _pubSegmentationResult.publish(_rgbSegmentedMsg);

    // 发布语义地图点云
    publishCloudMsg(_pubSemanticMap, semanticCloud(), _timeLaserCloudFullRes, "camera_init");
    publishCloudMsg(_pubSemanticMapLidarInst, semanticCloudLidarInst(), _timeLaserCloudFullRes, "camera_init");
    publishCloudMsg(_pubSemanticMapTotal, semanticMapCloudFullRes(), _timeLaserCloudFullRes, "camera_init");
}


} // END namespace loam
