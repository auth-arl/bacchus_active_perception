#include <active_perception/active_perception_controller.h>




namespace arl
{
namespace controller
{


void ActivePerceptionController::stemIdentCallback(const std_msgs::Int32MultiArray::ConstPtr feedback)
{
  
  int num_stems = feedback->layout.dim[0].size; 

  std::cout<<"Stems identified: " << num_stems << std::endl;

}



ActivePerceptionController::ActivePerceptionController(const std::shared_ptr<arl::robot::Robot>& robot) :
  arl::robot::Controller(robot, "Active Perception Controller")
{
  nh = ros::NodeHandle("~");
  ros::NodeHandle nh_new ;
  nh.getParam("k", k);
  camera_subscriber = nh_new.subscribe("/stem_coords", 1, &ActivePerceptionController::stemIdentCallback, this, ros::TransportHints().tcpNoDelay(true)); 

  std::cout<<"Ending constructor. "  << std::endl;
}

void ActivePerceptionController::init()
{
  std::cout<<"Ending init. "  << std::endl;
}

void ActivePerceptionController::measure()
{
  std::cout<<"Ending measure. "  << std::endl;
}

void ActivePerceptionController::update()
{
   std::cout<<"Ending update. "  << std::endl;
}

void ActivePerceptionController::command()
{
  // ros::spinOnce();
  std::cout<<"Ending command. "  << std::endl;
}

}  // namespace controller
} // namespace arl