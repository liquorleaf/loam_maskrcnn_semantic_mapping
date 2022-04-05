#ifndef LOAM_TWIST_H
#define LOAM_TWIST_H


#include "Angle.h"
#include "Vector3.h"

// 1个工具类：Twist

namespace loam {


/** \brief Twist composed by three angles and a three-dimensional position.
 * Twist类：六自由度位姿（3个角度、1个三维位置向量）
 */
class Twist {
public:
  // 默认构造函数：旋转角和平移向量全0
  Twist()
        : rot_x(),
          rot_y(),
          rot_z(),
          pos() {};

  // 4个成员：x,y,z旋转角，三维位置向量
  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;
};

} // end namespace loam

#endif //LOAM_TWIST_H
