#pragma once

#include <chrono>

//**********把C++标准类里的时间点拉出来，作为ros::Time的一个替代品**********

namespace loam
{ 
  /** \brief A standard non-ROS alternative to ros::Time.*/
  using Time = std::chrono::system_clock::time_point;   // 使用using进行 别名指定

  // 得到duration对应的秒数。其中先把std::chrono::system_clock::time_point::duration转为std::chrono::duration<double>，再用后者的函数提取

  // helper function
  inline double toSec(Time::duration duration)
  {
    return std::chrono::duration<double>(duration).count();
  };
}
