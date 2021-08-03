#ifndef ACTIVE_PERCEPTION_CONTROLLER_DEF
#define ACTIVE_PERCEPTION_CONTROLLER_DEF

#include <ros/ros.h>
#include <math.h>
#include <memory>
#include <autharl_core/robot/controller.h>


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
  ros::NodeHandle nh;
  double k;
};
}  // namespace controller
}  // namespace arl

#endif // ACTIVE_PERCEPTION_CONTROLLER_DEF
