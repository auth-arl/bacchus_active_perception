#include <active_perception/active_perception_controller.h>

namespace arl
{
namespace controller
{
ActivePerceptionController::ActivePerceptionController(const std::shared_ptr<arl::robot::Robot>& robot) :
  arl::robot::Controller(robot, "Active Perception Controller")
{
  nh = ros::NodeHandle("~");
  nh.getParam("k", k);
}

void PPC::init()
{
}

void PPC::measure()
{
}

void PPC::update()
{
}

void PPC::command()
{
}

}  // namespace controller
} // namespace arl