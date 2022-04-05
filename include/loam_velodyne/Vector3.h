#ifndef LOAM_VECTOR3_H
#define LOAM_VECTOR3_H


#include <pcl/point_types.h>

// 1个工具类：Vector3

namespace loam {

/** \brief Vector4f decorator for convenient handling.
 * 从Eigen::Vector4f类公有继承，多了一个数值存储位的三维坐标向量
 */
class Vector3 : public Eigen::Vector4f {
public:
  // 构造函数：沿用，用三维坐标，新加维度赋0
  Vector3(float x, float y, float z)
      : Eigen::Vector4f(x, y, z, 0) {}

  // 默认构造函数：沿用，所有坐标全零
  Vector3(void)
      : Eigen::Vector4f(0, 0, 0, 0) {}

  // 复制构造函数：沿用
  template<typename OtherDerived>
  Vector3(const Eigen::MatrixBase <OtherDerived> &other)
      : Eigen::Vector4f(other) {}

  // 从pcl的XYZI点类构造：沿用，用三维坐标，不用intensity，新加维度赋0
  Vector3(const pcl::PointXYZI &p)
      : Eigen::Vector4f(p.x, p.y, p.z, 0) {}

  // 重载= 可用Eigen模板对象赋值
  template<typename OtherDerived>
  Vector3 &operator=(const Eigen::MatrixBase <OtherDerived> &rhs) {
    this->Eigen::Vector4f::operator=(rhs);
    return *this;
  }

  // 重载= 可用pcl的XYZ点赋值，用三维坐标
  Vector3 &operator=(const pcl::PointXYZ &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }
  // 重载= 可用pcl的XYZI点赋值，用三维坐标，不用intensity
  Vector3 &operator=(const pcl::PointXYZI &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  // 提取x,y,z坐标值的函数
  float x() const { return (*this)(0); }

  float y() const { return (*this)(1); }

  float z() const { return (*this)(2); }

  // 返回x,y,z坐标的引用的函数
  float &x() { return (*this)(0); }

  float &y() { return (*this)(1); }

  float &z() { return (*this)(2); }

  // easy conversion  类型转换：Vector3转换为pcl::PointXYZI，继承三维坐标，intensity赋0
  operator pcl::PointXYZI() {
    pcl::PointXYZI dst;
    dst.x = x();
    dst.y = y();
    dst.z = z();
    dst.intensity = 0;
    return dst;
  }
};

} // end namespace loam

#endif //LOAM_VECTOR3_H
