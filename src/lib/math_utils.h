#ifndef LOAM_MATH_UTILS_H
#define LOAM_MATH_UTILS_H


#include "loam_velodyne/Angle.h"
#include "loam_velodyne/Vector3.h"

#include <cmath>

/** 6种inline的工具函数：（无特殊声明的都只有float没有double）
 * 1. rad2deg
 *    弧度变角度，有double和float两种重载
 * 2. deg2rad
 *    角度变弧度，有double和float两种重载
 * 3. calcSquaredDiff
 *    用XYZ坐标计算两点间距离的平方，有基础和“第二个向量用一float数值加权”两种重载
 * 4. calcPointDistance/calcSquaredPointDistance
 *    计算点和原点间的距离/距离的平方
 * 5. rotX/rotY/rotZ
 *    绕X/Y/Z轴将对象旋转一定角度（从转轴正向看回去是逆时针），有对象分别为向量(Vector3)/点(模板类)的两种重载
 * 6. rotateZXY/rotateYXZ
 *    用5，以ZXY/YXZ的顺序为轴，将对象分别旋转一定角度，有对象分别为向量(Vector3)/点(模板类)的两种重载
 */

namespace loam {

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}



/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians)
{
  return (float) (radians * 180.0 / M_PI);
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees)
{
  return (float) (degrees * M_PI / 180.0);
}




/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b)
{
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}



/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b, const float& wb)
{
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}


/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float calcPointDistance(const PointT& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}



/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float calcSquaredPointDistance(const PointT& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}



/** \brief Rotate the given vector by the specified angle around the x-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotX(Vector3& v, const Angle& ang)
{
  float y = v.y();
  v.y() = ang.cos() * y - ang.sin() * v.z();
  v.z() = ang.sin() * y + ang.cos() * v.z();
}

/** \brief Rotate the given point by the specified angle around the x-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotX(PointT& p, const Angle& ang)
{
  float y = p.y;
  p.y = ang.cos() * y - ang.sin() * p.z;
  p.z = ang.sin() * y + ang.cos() * p.z;
}



/** \brief Rotate the given vector by the specified angle around the y-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotY(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x + ang.sin() * v.z();
  v.z() = ang.cos() * v.z() - ang.sin() * x;
}

/** \brief Rotate the given point by the specified angle around the y-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotY(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x + ang.sin() * p.z;
  p.z = ang.cos() * p.z - ang.sin() * x;
}



/** \brief Rotate the given vector by the specified angle around the z-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotZ(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x - ang.sin() * v.y();
  v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given point by the specified angle around the z-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotZ(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x - ang.sin() * p.y;
  p.y = ang.sin() * x + ang.cos() * p.y;
}



/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 * rotateZXY(rz,rx,ry)使点从局部坐标系到世界坐标系，或rotateZXY(-rz,-rx,-ry)使点从sweep某点坐标系到sweep开始时坐标系：
 * 此工程中坐标系默认为X左(pitch)Y上(yaw)Z前(roll)，rotateZXY(rz,rx,ry)表示点转动(roll,pitch,yaw)，等效于点不动、坐标系转动(-roll,-pitch,-yaw)回到世界坐标系
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(v, angZ);
  rotX(v, angX);
  rotY(v, angY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 * rotateZXY(rz,rx,ry)使点从局部坐标系到世界坐标系，或rotateZXY(-rz,-rx,-ry)使点从sweep某点坐标系到sweep开始时坐标系：
 * 此工程中坐标系默认为X左(pitch)Y上(yaw)Z前(roll)，rotateZXY(rz,rx,ry)表示点转动(roll,pitch,yaw)，等效于点不动、坐标系转动(-roll,-pitch,-yaw)回到世界坐标系
 * @param p the point to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
template <typename PointT>
inline void rotateZXY(PointT& p,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(p, angZ);
  rotX(p, angX);
  rotY(p, angY);
}



/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 * rotateYXZ(-ry,-rx,-rz)使点从世界坐标系到局部坐标系，或rotateYXZ(ry,rx,rz)使点从sweep开始时坐标系到sweep之后某点坐标系：
 * 此工程中坐标系默认为X左(pitch)Y上(yaw)Z前(roll)，rotateYXZ(-ry,-rx,-rz)表示点转动(-yaw,-pitch,-roll)，等效于点不动、坐标系从世界坐标系转动(yaw,pitch,roll)
 * @param v the vector to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(v, angY);
  rotX(v, angX);
  rotZ(v, angZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 * rotateYXZ(-ry,-rx,-rz)使点从世界坐标系到局部坐标系，或rotateYXZ(ry,rx,rz)使点从sweep开始时坐标系到sweep之后某点坐标系：
 * 此工程中坐标系默认为X左(pitch)Y上(yaw)Z前(roll)，rotateYXZ(-ry,-rx,-rz)表示点转动(-yaw,-pitch,-roll)，等效于点不动、坐标系转动(yaw,pitch,roll)
 * @param p the point to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
template <typename PointT>
inline void rotateYXZ(PointT& p,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(p, angY);
  rotX(p, angX);
  rotZ(p, angZ);
}

} // end namespace loam


#endif // LOAM_MATH_UTILS_H
