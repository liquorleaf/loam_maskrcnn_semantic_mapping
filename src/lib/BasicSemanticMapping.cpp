// 2022, Li Longwei, Sun Yet-sen University

#include "loam_velodyne/BasicSemanticMapping.h"
#include "math_utils.h"

namespace loam
{

BasicSemanticMapping::BasicSemanticMapping(const float& confidenceThreshold, const float& maskThreshold, const float& matchSqDistThreshold):
    _confidenceThreshold(confidenceThreshold),
    _maskThreshold(maskThreshold),
    _maxMatchingSquareDistance(matchSqDistThreshold),
    _classNamesFile("/mnt/SSD0/dataset/CV/coco/coco_mask_rcnn/coco.names"),
    _colorsFile("/mnt/SSD0/dataset/CV/coco/coco_mask_rcnn/colors.txt"),
    _modelConfigFile("/mnt/SSD0/dataset/CV/coco/coco_mask_rcnn/mask_rcnn_inception_v2_coco_2018_01_28/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt"),
    _modelWeightsFile("/mnt/SSD0/dataset/CV/coco/coco_mask_rcnn/mask_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb"),
    _camIntrinsicFile("/home/guide02/SLAM/Laser_SLAM/loam_maskrcnn_semantic_mapping/d435i_VLP16_calibration/d435i_vlp16_extrinsic_calib/camera_intrinsic_color.yaml"),
    _camLidarExtrinsicFile("/home/guide02/SLAM/Laser_SLAM/loam_maskrcnn_semantic_mapping/d435i_VLP16_calibration/d435i_vlp16_extrinsic_calib/cam_lidar_extrinsic.yaml"),
    _numMaskDetections(0),
    _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
    _semanticCloud(new pcl::PointCloud<pcl::PointXYZL>()),
    _semanticMapCloudFullRes(new pcl::PointCloud<pcl::PointXYZL>())
{
    /** 加载Mask R-CNN网络 */
    // 加载类别名
    std::ifstream ifs(_classNamesFile.c_str());
    std::string line;
    while (getline(ifs, line)) { _classNames.push_back(line); }
    // 加载颜色
    ifs.close();
    ifs.open(_colorsFile.c_str());
    while (getline(ifs, line))
    {
        char *pEnd;
        double r, g, b;
        r = strtod(line.c_str(), &pEnd);
        g = strtod(pEnd, NULL);
        b = strtod(pEnd, NULL);
        _colors.push_back(cv::Scalar(r, g, b, 255.0));
    }
    ifs.close();
    // 加载网络
    _MaskrcnnNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    _MaskrcnnNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    _MaskrcnnNet = cv::dnn::readNetFromTensorflow(_modelWeightsFile, _modelConfigFile);
    // 网络输出名称
    _outNames.resize(2);
    _outNames[0] = "detection_out_final";
    _outNames[1] = "detection_masks";
    // 显示网络已载入的信息
    ROS_INFO("Mask R-CNN network loaded successfully.");

    /** 加载传感器参数 */
    // 相机内参
    cv::FileStorage fileReadingIntr;
    fileReadingIntr.open(_camIntrinsicFile, cv::FileStorage::READ);
    if (!fileReadingIntr.isOpened()) { ROS_ERROR("Failed to open camera instrinsic params file at %s !!! ", _camIntrinsicFile.c_str()); }
    else { ROS_INFO("Opened camera instrinsic params file at %s", _camIntrinsicFile.c_str()); }
    fileReadingIntr["camera_matrix"] >> _cameraMatrix;
    fileReadingIntr["distortion_model"] >> _distortionModel;
    fileReadingIntr["distortion_coefficients"] >> _distortionCoeff;
    fileReadingIntr.release();
    // 显示内参
    std::cout << std::endl;
    std::cout << "Camera instrinsic matrix K:\n" << _cameraMatrix << std::endl;
    std::cout << "Camera distortion model: " << _distortionModel << std::endl;
    std::cout << "Camera distortion coefficients D:\n" << _distortionCoeff << std::endl;
    // 相机矩阵的逆阵
    _cameraMatrixINV = _cameraMatrix.inv();

    // 相机-lidar外参
    cv::FileStorage fileReadingExtr;
    fileReadingExtr.open(_camLidarExtrinsicFile, cv::FileStorage::READ);
    if (!fileReadingExtr.isOpened()) { ROS_ERROR("Failed to open camera-lidar exstrinsic params file at %s !!! ", _camLidarExtrinsicFile.c_str()); }
    else { ROS_INFO("Opened camera-lidar exstrinsic params file at %s", _camLidarExtrinsicFile.c_str()); }
    fileReadingExtr["extrinsic_params_matrix"] >> _camLidarTrans;
    fileReadingExtr.release();
    // 显示外参
    std::cout << "Camera-lidar exstrinsic pose transformation(rx,ry,rz;tx,ty,tz):\n" << _camLidarTrans << "\n" << std::endl;

    /** 设置体素栅格滤波器参数 */
    float dsFilterInstSize = sqrt(matchSqDistThreshold/3.0f);  // 实例点云下采样体素栅格滤波器尺寸 = 以匹配阈值距离作为立方体中心到顶点距离时，立方体的边长
    _downSizeFilterInstance.setLeafSize(dsFilterInstSize, dsFilterInstSize, dsFilterInstSize);
}


void BasicSemanticMapping::process()
{
    timeval t1,t2;
    gettimeofday(&t1, NULL);

    _semanticCloud->clear(); // 清除上一帧lidar语义点云

    /** Mask R-CNN实例分割：得到语义标签 */
    /* Create a 4D blob from a frame.
     * 注意：opencv以BGR存储彩色图像，而blob处理中假设图像以RGB存储，所以交换第1、3个通道的flag为true
     */
    cv::dnn::blobFromImage(_rgbImageData, _blob, 1.0, cv::Size(_rgbImageData.cols, _rgbImageData.rows), cv::Scalar(), true, false);
    // 设置网络输入
    _MaskrcnnNet.setInput(_blob);
    // 对RGB图像做实例分割
    _outs.clear();
    _MaskrcnnNet.forward(_outs, _outNames);
    // 画实例分割结果框展示图
    postSegProcess(_rgbImageSegmented, _outs);
    // 结果框展示图从bgr转换为rgb格式以便ros显示
    cv::cvtColor(_rgbImageSegmented, _rgbImageSegmented, CV_BGR2RGB, 0);

    /** 利用实例分割结果和深度图，给激光点云点标注语义信息 */
    // 清理上次的实例点云，并resize到这次需要的大小
    for(int instInd = 0; instInd<_instanceCloud.size(); instInd++)
    {
        _instanceCloud[instInd]->clear();
        _instanceCloudDS[instInd]->clear(); 
    }
    _instanceCloud.clear();
    _instanceCloudDS.clear();
    _instanceCloud.resize(_numMaskDetections);
    _instanceCloudDS.resize(_numMaskDetections);
    for(int instInd = 0; instInd<_numMaskDetections; instInd++)
    {
        _instanceCloud[instInd].reset(new pcl::PointCloud<pcl::PointXYZI>);
        _instanceCloudDS[instInd].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    _instanceKDTree = new nanoflann::KdTreeFLANN<pcl::PointXYZI>[_numMaskDetections];
    // 对每个检测到并建立掩膜的目标实例，建立实例点云和相应的KD树
    for (int instInd = 0; instInd < _numMaskDetections; instInd++)
    {
        /* 提取目标掩膜对应深度图，获得像素坐标对应的深度；
         * 利用相机内参，获取目标掩膜点云的相机坐标；
         * 利用相机和lidar间的外参，将目标掩膜点云转换到lidar坐标系
         * 利用lidar坐标系的位姿，将目标掩膜点云转换到世界坐标系
         */ 
        // 新建一幅全黑图，用frame(box)法获得掩膜图
        pcl::PointXYZ pTempCL;           // 从相机坐标系到lidar坐标系时临时存放点
        pcl::PointXYZI pTempLW;          // 从lidar坐标系到世界坐标系时临时存放点
        float pDepth = 0.0f, pX = 0.0f, pY = 0.0f, pXd = 0.0f, pYd = 0.0f, // 临时存放点深度、畸变校正前后的X坐标和Y坐标等
              pXpY = 0.0f, pXpX = 0.0f, pYpY = 0.0f; // 畸变校正计算中间变量
        float rSqr = 0.0f, rSqrSqr = 0.0f, rSqrSqrSqr = 0.0f; // 临时存放点到中心的平方距离等
        cv::Mat frameInstSeg; // 新建空白掩膜图
        frameInstSeg = cv::Mat(_depthImageData.rows, _depthImageData.cols, CV_8UC1, 0.0);   // Mask R-CNN的输出原始掩膜15x15图是8位1通道
        _objectSegMask[instInd].copyTo(frameInstSeg(_objectBox[instInd]),_objectSegMask[instInd]); // 绘制掩膜
        // 遍历掩膜图每个像素点，遇到非零值即掩膜上的点，则提取深度，加入当前实例点云中              cv::namedWindow("WinName", cv::WINDOW_NORMAL);cv::imshow("WinName",frameInstSeg);cv::waitKey();
        for(int i = 0; i < _depthImageData.rows; i++)
        {
            for (int j = 0; j < _depthImageData.cols; j++)
            {
                if (frameInstSeg.at<unsigned char>(i,j) > 0) // 若该像素点属于掩膜，则提取并变换；typedef unsigned char uchar/uint8_t
                {
                    // 深度比例尺1000，因为深度图像素值单位是毫米(详见realsense2_camera::ROS_DEPTH_SCALE的定义，const float ROS_DEPTH_SCALE = 0.001;)
                    pDepth = float(_depthImageData.at<unsigned short>(i,j)) / CAM_DEPTH_SCALE; // 提取深度，且单位转换为米；typedef unsigned short uint16_t
                    // 提取像素坐标，归一化相机坐标 = K^(-1)*像素坐标；typedef float _Float32
                    pX = (float(j) - _cameraMatrix.at<float>(0,2)) * _cameraMatrixINV.at<float>(0,0); // x = (u-x0)/fx；u轴水平向右，对应列索引
                    pY = (float(i) - _cameraMatrix.at<float>(1,2)) * _cameraMatrixINV.at<float>(1,1); // y = (v-y0)/fy；v轴垂直向下，对应行索引
                    // 畸变校正
                    if (_distortionModel == "plumb_bob") // 畸变模型plumb_bob参数：k1,k2,p1,p2,k3
                    {
                        pXpY = pX * pY;
                        pXpX = pX * pX;
                        pYpY = pY * pY;
                        rSqr = pXpX + pYpY;
                        rSqrSqr = rSqr * rSqr;
                        rSqrSqrSqr = rSqrSqr * rSqr;
                        pXd = pX * (1.0f + _distortionCoeff.at<float>(0,0) * rSqr + _distortionCoeff.at<float>(0,1) * rSqrSqr
                            + _distortionCoeff.at<float>(0,4) * rSqrSqrSqr) + 2.0f * _distortionCoeff.at<float>(0,2) * pXpY
                            + _distortionCoeff.at<float>(0,3) * (rSqr + 2.0f * pXpX);
                        pYd = pY * (1.0f + _distortionCoeff.at<float>(0,0) * rSqr + _distortionCoeff.at<float>(0,1) * rSqrSqr
                            + _distortionCoeff.at<float>(0,4) * rSqrSqrSqr) + 2.0f * _distortionCoeff.at<float>(0,3) * pXpY
                            + _distortionCoeff.at<float>(0,2) * (rSqr + 2.0f * pYpY);
                    }
                    // 相机坐标 = 深度*归一化相机坐标
                    pTempCL.x = pDepth * pXd;
                    pTempCL.y = pDepth * pYd;
                    pTempCL.z = pDepth; // * 1.0
                    // lidar坐标 = 相机坐标使用外参做(rx,ry,rz,t)变换(此处使用弧度制，且定义的相机坐标系X右Y下Z前，lidar坐标系X前Y左Z上)
                    rotX(pTempCL, _camLidarTrans.at<float>(0,0));
                    rotY(pTempCL, _camLidarTrans.at<float>(0,1));
                    rotZ(pTempCL, _camLidarTrans.at<float>(0,2));
                    pTempCL.x += _camLidarTrans.at<float>(1,0);
                    pTempCL.y += _camLidarTrans.at<float>(1,1);
                    pTempCL.z += _camLidarTrans.at<float>(1,2);
                    // 坐标轴统一：lidar坐标系X前Y左Z上，loam统一坐标系X左Y上Z前
                    pTempLW.x = pTempCL.y;
                    pTempLW.y = pTempCL.z;
                    pTempLW.z = pTempCL.x;
                    // 世界坐标 = lidar坐标使用SLAM部分得到的位姿做(roll,pitch,yaw,t)，即(rz,rx,ry,t)变换
                    rotZ(pTempLW, _transformAftMapped.rot_z);
                    rotX(pTempLW, _transformAftMapped.rot_x);
                    rotY(pTempLW, _transformAftMapped.rot_y);
                    pTempLW.x += _transformAftMapped.pos.x();
                    pTempLW.y += _transformAftMapped.pos.y();
                    pTempLW.z += _transformAftMapped.pos.z();
                    // 该语义点进入实例点云
                    _instanceCloud[instInd]->push_back(pTempLW);
//pcl::PointXYZL ptLabeled0;ptLabeled0.x = pTempLW.x;ptLabeled0.y = pTempLW.y;ptLabeled0.z = pTempLW.z;ptLabeled0.label = _objectClassId[instInd];
//_semanticCloud->push_back(ptLabeled0);
                }
            }
        } // END for (掩膜图像素点遍历)

        // 实例点云下采样精简、去除NaN无效点、构建KD树
        _downSizeFilterInstance.setInputCloud(_instanceCloud[instInd]);
        _downSizeFilterInstance.filter(*_instanceCloudDS[instInd]);
        std::vector<int> pclInd;    // 去除NaN无效点时用到的辅助容器
        pcl::removeNaNFromPointCloud(*(_instanceCloudDS[instInd]), *(_instanceCloudDS[instInd]), pclInd);   // 谨慎起见，实例点云去除一次NaN无效点
//for(auto& pt : *(_instanceCloud[instInd])){pcl::PointXYZL ptLabeled0;ptLabeled0.x = pt.x;ptLabeled0.y = pt.y;ptLabeled0.z = pt.z;ptLabeled0.label = _objectClassId[instInd];_semanticCloud->push_back(ptLabeled0);}
        _instanceKDTree[instInd].setInputCloud(_instanceCloudDS[instInd]);    // 实例点云构建KD树，便于查找其中点
    } // END for (Mask R-CNN分割结果实例遍历)

    // 遍历lidar点云，和实例点云做距离匹配，若成功，则打上距离最小的实例的语义标签
    std::vector<std::vector<int>> pointSearchInd(_numMaskDetections);       // 搜索到的最近实例点的索引
    std::vector<std::vector<float>> pointSearchSqrDist(_numMaskDetections); // 搜索到的最近实例点离当前lidar点的距离平方
    pcl::PointXYZL ptLabeled;                                               // 临时存放打上标签的目标点
    pcl::PointXYZI pt;                                                      // 临时存放提取的lidar点
    int laserCloudSize = _laserCloudFullRes->size();
    for (int ptInd = 0; ptInd < laserCloudSize; ptInd++) // 对lidar点云中的每个点遍历
    {
        pt = _laserCloudFullRes->at(ptInd);

        // if (pt.z > 0) // 可选速率优化？？？Z坐标<=0的点在相机视野外，不参与语义地图；但不应加到这里，因为这里是世界坐标系；不知道全点转换坐标系和多搜索一半的点哪个用时更长

        // 默认分类为“无”
        ptLabeled.x = pt.x;
        ptLabeled.y = pt.y;
        ptLabeled.z = pt.z;
        ptLabeled.label = _classNames.size();
        if (_numMaskDetections > 0) // 若有实例，再进行语义搜索
        {
            // 搜索每个实例最近的实例点，记录平方距离
            for (int instInd = 0; instInd < _numMaskDetections; instInd++)
            {
                pointSearchInd[instInd].resize(1);
                pointSearchSqrDist[instInd].resize(1);
                _instanceKDTree[instInd].nearestKSearch(pt, 1, pointSearchInd[instInd], pointSearchSqrDist[instInd]);
            }
            // 搜索最近点平方距离最小的实例
            int instNearestInd = 0;
            for (int instInd = 1; instInd < _numMaskDetections; instInd++)
            {
                if (pointSearchSqrDist[instInd][0] < pointSearchSqrDist[instNearestInd][0])
                    instNearestInd = instInd;
            }
            // 如果存在很近平方距离低于阈值的实例点，则认为匹配有效，lidar点打上语义标签(如coco共80种，编号0-89，其中有10个无效值不会用到)
            if (pointSearchSqrDist[instNearestInd][0] <= _maxMatchingSquareDistance)
                ptLabeled.label = _objectClassId[instNearestInd];
        }

        // lidar点加入当前帧语义点云
        _semanticCloud->push_back(ptLabeled);
    } // END for (lidar当前帧每个点遍历)

    /** 把本次做了语义处理的点云合并到总体语义点云地图中 */
    _semanticMapCloudFullRes->concatenate(*_semanticMapCloudFullRes, *_semanticCloud);

    /** 释放内存 */
    delete[] _instanceKDTree;

    //计算用时
    gettimeofday(&t2, NULL);
    u_int64_t T = (t2.tv_sec - t1.tv_sec) * 1000 + (t2.tv_usec - t1.tv_usec) / 1000;
    std::cout << T << "ms" << std::endl;
}


void BasicSemanticMapping::updateTransform(double pitch, double yaw, double roll, double x, double y, double z)
{
    _transformAftMapped.rot_x = pitch;
    _transformAftMapped.rot_y = yaw;
    _transformAftMapped.rot_z = roll;

    _transformAftMapped.pos.x() = float(x);
    _transformAftMapped.pos.y() = float(y);
    _transformAftMapped.pos.z() = float(z);
}


void BasicSemanticMapping::postSegProcess(cv::Mat &frame, const std::vector<cv::Mat> &outs)
{
    // 清理上一帧的识别结果
    _numMaskDetections = 0;
    _objectBox.clear();
    _objectSegMask.clear();
    _objectClassId.clear();

    cv::Mat outDetections = outs[0];
    cv::Mat outMasks = outs[1];

    /** 掩膜输出的尺寸是NxCxHxW，其中
     * N-检测到的目标box数
     * C-分类总数 (不包含“背景”类)，例如对于coco数据集是80
     * HxW-在该尺寸的矩阵中存储了当前目标的掩膜图像，例如对于coco数据集的预训练模型，是15x15
     */
    const int numDetections = outDetections.size[2];  // 检测出的目标数
    const int numClasses = outMasks.size[1];          // 掩膜

    outDetections = outDetections.reshape(1, outDetections.total() / 7); // 整合检测结果，使得只剩一个channel(即每行放1个目标的7个结果，所有目标在同一矩阵中)
    for (int i = 0; i < numDetections; ++i) // i：对每个检测出的目标
    {
        float score = outDetections.at<float>(i, 2);    // 提取当前目标的置信概率
        if (score > _confidenceThreshold)   // 只处理置信概率大于阈值的目标
        {
            // 提取识别框：结果坐标是一个占整体的比例值，需计算相应行/列号
            int classId = static_cast<int>(outDetections.at<float>(i, 1));              // 提取当前目标的分类
            int left = static_cast<int>(frame.cols * outDetections.at<float>(i, 3));    // 提取当前目标识别框的左边列号(横坐标)
            int top = static_cast<int>(frame.rows * outDetections.at<float>(i, 4));     // 提取当前目标识别框的上边行号(纵坐标)
            int right = static_cast<int>(frame.cols * outDetections.at<float>(i, 5));   // 提取当前目标识别框的右边列号(横坐标)
            int bottom = static_cast<int>(frame.rows * outDetections.at<float>(i, 6));  // 提取当前目标识别框的下边行号(纵坐标)

            // 将识别框的边限制在图像内
            left = std::max(0, std::min(left, frame.cols - 1));
            top = std::max(0, std::min(top, frame.rows - 1));
            right = std::max(0, std::min(right, frame.cols - 1));
            bottom = std::max(0, std::min(bottom, frame.rows - 1));
            cv::Rect box = cv::Rect(left, top, right - left + 1, bottom - top + 1);     // 根据四条边的行/列号创建矩形识别框对象

            /** 提取当前目标掩膜：
             * 使用的构造函数是Mat(int rows, int cols, int type, void* data, size_t step=AUTO_STEP);
             * 四个参数分别是高、宽、类型(CV_32F每个像素是单通道32位[0,1]值)、当前目标在结果分类上的掩膜图像数据(指针指向原数据，而非复制)
             */
            cv::Mat objectMask(outMasks.size[2], outMasks.size[3], CV_32F, outMasks.ptr<float>(i, classId));
            /* 从掩膜中筛选出值大于创建阈值的点，作为分割结果，并上色 */
            cv::resize(objectMask, objectMask, cv::Size(box.width, box.height)); // 把固定大小的目标掩膜图像缩放到和目标识别框同样大小
            cv::Mat segMask = (objectMask > _maskThreshold);   // 筛选出大于阈值的掩膜点，构建可信的掩膜
            segMask.convertTo(segMask, CV_8U); // 由CV_32F转为CV_8U(每个像素是unsigned int8)，以便能用findContours函数处理
            // 在结果展示图上画识别框和掩膜
            drawBoxAndMask(frame, classId, score, box, segMask);

            // 保存当前目标的box和segMask和对应的classId
            _objectBox.push_back(box);
            _objectSegMask.push_back(segMask);
            _objectClassId.push_back(classId);
            _numMaskDetections++;
        }
    }
}

void BasicSemanticMapping::drawBoxAndMask(cv::Mat &frame, int classId, float conf, cv::Rect &box, cv::Mat &segMask)
{
    /* 画一个代表目标识别框的矩形；使用左上顶点和右下顶点定位 */
    cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x+box.width, box.y+box.height),
                cv::Scalar(BOUND_BOX_COLOR_B, BOUND_BOX_COLOR_G, BOUND_BOX_COLOR_R), BOUND_BOX_THICKNESS);

    /* 提取分类编号对应的分类名称及其置信概率 */
    std::string label = cv::format("%.2f", conf);   // 置信概率显示两位小数
    //if (!_classNames.empty())   // 鲁棒性：以防分类名称读取失败或不存在；实验版可去掉
    //{
    //    CV_Assert(classId < (int)_classNames.size());   // 鲁棒性：以防分类名称访问越界；实验版可去掉
        label = _classNames[classId] + ":" + label; // 标签 = “分类名称:置信概率”
    //}

    /* 在识别框左上角显示标签 */
    int baseLine;   // 存放标签顶边缘到文字顶在y轴上的距离
    // 指定字体、缩放、粗细，计算标签尺寸
    cv::Size labelSize = cv::getTextSize(label, LABEL_FONT, LABEL_SCALE, LABEL_THICKNESS, &baseLine);
    // 画出标签边框：定位用左上角和右下角的坐标
    cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x + round(LABEL_HOR_SCALE_TOTAL*labelSize.width), box.y + round(2*baseLine) + labelSize.height),
                cv::Scalar(LABEL_BOX_COLOR_B,LABEL_BOX_COLOR_G,LABEL_BOX_COLOR_R), cv::FILLED);
    // 写上标签文字：定位用左下角；文字的上下各留baseLine高，左右各留LABEL_HOR_PAD_SCALE*labelSize.width宽
    putText(frame, label, cv::Point(box.x + round(LABEL_HOR_PAD_SCALE*labelSize.width), box.y + labelSize.height + baseLine), LABEL_FONT, LABEL_SCALE,
            cv::Scalar(LABEL_COLOR_B,LABEL_COLOR_G,LABEL_COLOR_R), LABEL_THICKNESS);

    /* 掩膜染色方案1：使用由分类编号决定的固定颜色 */
    cv::Scalar color = _colors[classId%_colors.size()];
    /* 掩膜染色方案2：每个实例都使用随机颜色 */
    //int colorInd = std::rand() % colors.size();
    //cv::Scalar color = colors[colorInd];

    /* 从掩膜中筛选出值大于创建阈值的点，作为分割结果，并上色 */
    cv::Mat coloredRoi = (MASK_COLOR_RATIO * color + MASK_COLOR_RATIO_INV * frame(box)); // 提取图像识别框内的部分，添加纯色滤镜
    coloredRoi.convertTo(coloredRoi, CV_8UC3);  // 染色后的识别框图像由原格式转为8位3通道(彩色)，以便与后面转为8位的segMask融合

    /* 绘制分割结果掩膜的边界 */
    std::vector<cv::Mat> contours;  // 存放边界
    cv::Mat hierarchy;              // 存放边界的拓扑结构
    /** 寻找mask边界：
     * 结果为contours，是边(即点集合)的向量；hierarchy是拓扑结构(每条边的4个相邻：同层次的前一条、后一条边，第一条子边，父边)，
     * 检索模式：cv::RETR_CCOMP，检索所有边，以二层形式组织，上层是外边界，第二层是洞边界
     * 边界精简模式：cv::CHAIN_APPROX_SIMPLE，水平、竖直、斜对角的边会被压缩到只留起点和终点
     */
    findContours(segMask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    /** 绘制mask边界：
     * 第三个参数为负，表示全部边界都绘制；使用线型cv::LINE_8即8连通线
     * 给予拓扑结构信息，最大追溯MASK_MAX_LEVEL层结构
     */
    drawContours(coloredRoi, contours, -1, color, MASK_CONTOUR_THICKNESS, cv::LINE_8, hierarchy, MASK_MAX_LEVEL);
    // 将染色后识别框内图像(coloredRoi)中“segMask为非零值的点”复制到frame的(box)部分中，即frame中(box)内掩膜染色（注意，该函数要求了coloredRoi和frame(box)大小相同）
    coloredRoi.copyTo(frame(box), segMask);
}


} // END namespace loam
