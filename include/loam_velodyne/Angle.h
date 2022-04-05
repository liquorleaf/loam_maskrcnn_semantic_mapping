#ifndef LOAM_ANGLE_H
#define LOAM_ANGLE_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

// 1个工具类：Angle

namespace loam {


/** \brief Class for holding an angle.
 *
 * This class provides buffered access to sine and cosine values to the represented angular value.
 * 
 * 建立一个角（弧度）对应的cos和sin值供查询；有表可查则可避免每次都算，加快运行速度
 */
class Angle {
public:
  // 默认构造函数：0，对应cos值1，sin值0
  Angle()
      : _radian(0.0),
        _cos(1.0),
        _sin(0.0) {}

  // 构造函数：输入弧度制度数，计算其cos和sin值并存储
  Angle(float radValue)
      : _radian(radValue),
        _cos(std::cos(radValue)),
        _sin(std::sin(radValue)) {}

  // 复制构造函数
  Angle(const Angle &other)
      : _radian(other._radian),
        _cos(other._cos),
        _sin(other._sin) {}

  // 重载 = 运算符，因为没有用到new的，所以深浅拷贝一样（两种形式：void 和 类名&,返回*this ，这里用了第一种）
  void operator=(const Angle &rhs) {
    _radian = (rhs._radian);
    _cos = (rhs._cos);
    _sin = (rhs._sin);
  }

  // 重载+= 用当前的弧度加上另一弧度数值或另一对象的弧度值，得到新弧度值，再用其构造一个对象
  void operator+=(const float &radValue) { *this = (_radian + radValue); }

  void operator+=(const Angle &other) { *this = (_radian + other._radian); }

  // 重载-= 用当前的弧度减去另一弧度数值或另一对象的弧度值，得到新弧度值，再用其构造一个对象
  void operator-=(const float &radValue) { *this = (_radian - radValue); }

  void operator-=(const Angle &other) { *this = (_radian - other._radian); }

  // 重载- 单目运算符（负），而非双目（减）。弧度取相反数，cos不变，sin取相反数
  Angle operator-() const {
    Angle out;
    out._radian = -_radian;
    out._cos = _cos;
    out._sin = -(_sin);
    return out;
  }

  // 提取该Angle的弧度、角度、cos值、sin值的函数
  
  float rad() const { return _radian; }

  float deg() const { return float(_radian * 180 / M_PI); }

  float cos() const { return _cos; }

  float sin() const { return _sin; }

// 3个数据成员：弧度，以及对应的cos值、sin值
private:
  float _radian;    ///< angle value in radian
  float _cos;       ///< cosine of the angle
  float _sin;       ///< sine of the angle
};

} // end namespace loam

#endif //LOAM_ANGLE_H
