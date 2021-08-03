#ifndef ACTIVE_PERCEPTION_CONTROLLER_DEF
#define ACTIVE_PERCEPTION_CONTROLLER_DEF

#include <ros/ros.h>
#include <math.h>
#include <memory>
#include <autharl_core/robot/controller.h>
#include <std_msgs/Int32MultiArray.h>



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
  ros::NodeHandle nh;
  double k;
  ros::Subscriber camera_subscriber;
};
}  // namespace controller
}  // namespace arl

#endif // ACTIVE_PERCEPTION_CONTROLLER_DEF
