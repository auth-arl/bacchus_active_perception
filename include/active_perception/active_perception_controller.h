#ifndef ACTIVE_PERCEPTION_CONTROLLER_DEF
#define ACTIVE_PERCEPTION_CONTROLLER_DEF

#include <ros/ros.h>
#include <math.h>
#include <memory>
#include <autharl_core/robot/controller.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <active_perception/detector.h>



namespace arl
{
namespace controller
{
class ActivePerceptionController : public arl::robot::Controller
{
public:
  ActivePerceptionController(const std::shared_ptr<arl::robot::Robot>& robot);
  void init();
  void measure();
  void update();
  void command();
private:
  void stemIdentCallback(const std_msgs::Int32MultiArray::ConstPtr feedback);
  void rgbCallback(const sensor_msgs::Image::ConstPtr rgb_feedback);
  void depthCallback(const sensor_msgs::Image::ConstPtr depth_feedback);
  int findClosestStamp(double current_stamp, std::vector<double> stamp_list);
  ros::NodeHandle nh;
  double k;
  ros::Subscriber stem_subscriber;
  ros::Subscriber rgb_subscriber;

  Eigen::MatrixXi stem_mask;
  int stem_mask_npoints;

  std::vector<cv::Mat> image_buffer;
  std::vector<double> image_stamps;

  std::vector<cv::Mat> depth_buffer;
  std::vector<double> depth_stamps;

  // std::map< int, std::string,> buffer_map;
  int buffer_size;
  int cyclic_index_rgb;
  int cyclic_index_depth;
  bool valid_ident;

  Eigen::Vector3d p_Target_world;
  Eigen::Vector3d p_Target_vision;
  int r_region;

  Eigen::Vector3d p_camera;
  Eigen::Matrix3d R_camera;

  Eigen::MatrixXf stem_pointCloud;
  int stem_num_points;
  Eigen::MatrixXf obs_pointCloud;
  int obs_num_points;

  Eigen::MatrixXd J_ee;

  bool vision_target;

  Eigen::Vector3d v_filter;
  double k_filter, m_filter, d_filter, Tc;

};
}  // namespace controller
}  // namespace arl

#endif // ACTIVE_PERCEPTION_CONTROLLER_DEF
