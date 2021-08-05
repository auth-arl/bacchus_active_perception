#include <active_perception/detector.h>
#include <chrono>


PinholeCamera::PinholeCamera(double fx, double fy, double cx, double cy, double size_x, double size_y)
  : fx(fx)
  , fy(fy)
  , cx(cx)
  , cy(cy)
  , size_x(size_x)
  , size_y(size_y)
{
}

Eigen::Vector3d PinholeCamera::project(const Eigen::Vector3d& point_3d) const
{
  Eigen::Vector3d point;
  point[0] = point_3d[0] * this->fx / point_3d[2] + this->cx;
  point[1] = point_3d[1] * this->fy / point_3d[2] + this->cy;
  // point[2] = point_3d[2] * 1000.0f;
  point[2] = point_3d[2] ;
  return point;
}

Eigen::Vector3d PinholeCamera::backProject(const Eigen::Vector2d& point, float depth_value) const
{
  Eigen::Vector3d point_3d;
  point_3d[2] = depth_value ;
  // point_3d[2] = depth_value / 1000.0f;
  point_3d[0] = (point[0] - this->cx) * point_3d[2] / this->fx ;
  point_3d[1] = (point[1] - this->cy) * point_3d[2] / this->fy ;
  return point_3d;
}

Eigen::Matrix3d PinholeCamera::matrix() const
{
  Eigen::Matrix3d cam_matrix;
  cam_matrix.setZero();
  cam_matrix(0, 0) = this->fx;
  cam_matrix(1, 1) = this->fy;
  cam_matrix(0, 2) = this->cx;
  cam_matrix(1, 2) = this->cy;
  cam_matrix(2, 2) = 1.0;
  return cam_matrix;
}

