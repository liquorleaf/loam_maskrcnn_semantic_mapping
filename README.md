#  loam_maskrcnn_semantic_mapping

LOAM and Mask R-CNN Semantic Mapping: semantic labelling for lidar point cloud with a color and a depth camera. Tested with Velodyne VLP-16, Intel D435i, Ubuntu 20.04, OpenCV 4.5.4 and ROS noetic.


LOAM: https://github.com/laboshinl/loam_velodyne/

Mask R-CNN: https://github.com/matterport/Mask_RCNN/

Mask R-CNN model example: https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API


Color file and  Mask R-CNN class name file are provided in `./mask_rcnn_documents`

Before running, please change the path of intrinsic and extrinsic parameter files, color file, Mask R-CNN class name, model and config files in `./src/lib/BasicSemanticMapping.cpp`


## How to build with catkin

```
$ cd ${any path you like}
$ mkdir -p catkin_ws/src
$ cd ./catkin_ws/src
$ git clone https://github.com/Yinshideguanghui/loam_maskrcnn_semantic_mapping.git
$ catkin_init_workspace
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ./devel/setup.bash
```

## Running

```
roslaunch loam_velodyne_semantic_mapping loam_semantic_mapping.launch
```

## LOAM Troubleshooting

### `multiScanRegistration` crashes right after playing bag file

Issues [#71](https://github.com/laboshinl/loam_velodyne/issues/71) and
[#7](https://github.com/laboshinl/loam_velodyne/issues/7) address this
problem. The current known solution is to build the same version of PCL that
you have on your system from source, and set the `CMAKE_PREFIX_PATH`
accordingly so that catkin can find it. See [this
issue](https://github.com/laboshinl/loam_velodyne/issues/71#issuecomment-416024816)
for more details.


---
[Quantifying Aerial LiDAR Accuracy of LOAM for Civil Engineering Applications.](https://ceen.et.byu.edu/sites/default/files/snrprojects/wolfe_derek.pdf) Derek Anthony Wolfe

[ROS & Loam_velodyne](https://ishiguro440.wordpress.com/2016/04/05/%E5%82%99%E5%BF%98%E9%8C%B2%E3%80%80ros-loam_velodyne/) 
