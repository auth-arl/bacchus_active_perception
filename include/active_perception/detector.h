#ifndef PCTOSURF_ROS_DETECTOR_H
#define PCTOSURF_ROS_DETECTOR_H


// system
#include <iostream>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>

// eigen
#include <eigen3/Eigen/Geometry>


#include <chrono>



/**
 * @brief A model for a Pinhole Camera which receives the intrinsics as input
 * 
 * More info here: https://en.wikipedia.org/wiki/Pinhole_camera_model
 *
 */
class PinholeCamera
{
public:
  PinholeCamera(double fx, double fy, double cx, double cy, double size_x, double size_y);

  /**
   * @brief Projects a point on the image plane to 3-D given the depth value on
   *        the pixel and the intrinsics of the camera
   *
   * @param point A 2-D point on the image plane
   * @param depth_value The depth value for this 2-D point
   * 
   * @returns The 3-D point with respect to the camera's frame
   */
  Eigen::Vector3d backProject(const Eigen::Vector2d& point, float depth_value) const;
  
  Eigen::Vector3d project(const Eigen::Vector3d& point_3d) const;

  /**
   * @brief Returns camera matrix.
   *        
   *
   * @returns The rotation matrix: 
   * 
   * \f[
   *  \begin{bmatrix}
   *   f_x &   0 & c_x \\
   *    0  & f_y & c_y \\
   *    0  &   0 &  1 
   *  \end{bmatrix}
   * \f]
   */
  Eigen::Matrix3d matrix() const;

  /**
   * @brief Focal length in x-axis (millimeters)
   */
  double fx;

  /**
   * @brief Focal length in y-axis (millimeters)
   */
  double fy;
  
  /**
   * @brief The x-dimension (along height) of the principal point (in pixels)
   */
  double cx;

  /**
   * @brief The y-dimension (along width) of the principal point (in pixels)
   */
  double cy;

  /**
   * @brief The height of the image (in pixels)
   */
  double size_x;

  /**
   * @brief The witdh of the image (in pixels)
   */
  double size_y;
};

/**
 * \brief Wrapper class of Apriltag for detecting a tag
 * 
 * \param params The apriltag parameters
 * \param refine_pose Parameter for better accuracy with the cost of
 * performance. Set to 0 for fast detection with low accuracy and to 1 for slow
 * detection with high accuracy
 */



#endif  // PCTOSURF_ROS_DETECTOR_H
