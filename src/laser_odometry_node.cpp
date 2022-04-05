#include <ros/ros.h>
#include "loam_velodyne/LaserOdometry.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserOdometry laserOdom(0.1);

  if (laserOdom.setup(node, privateNode)) {
    // initialization successful
    laserOdom.spin();  // 持续运行使用的是自编方法
  }

  return 0;
}
