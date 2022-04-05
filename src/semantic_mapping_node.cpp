#include <ros/ros.h>
#include "loam_velodyne/SemanticMapping.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "semanticMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::SemanticMapping semanticMap(loam::DEFAULT_CONFI_THRE, loam::DEFAULT_MASK_THRE, loam::DEFAULT_MATCH_SQ_DIST_THRE);

  if (semanticMap.setup(node, privateNode)) {
    // initialization successful
    semanticMap.spin();  // 持续运行使用的是自编方法
  }

  return 0;
}
