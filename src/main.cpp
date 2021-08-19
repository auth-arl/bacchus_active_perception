#include <ros/ros.h>
#include <autharl_core/viz/ros_state_publisher.h>
#include <autharl_core/robot/ros_model.h>
#include <autharl_core/robot/robot.h>
#include <autharl_core/robot/robot_sim.h>
#include <bacchus_robot/robot_dual.h>
#include <active_perception/active_perception_controller.h>
#include <autharl_core/controller/joint_trajectory.h>
#include <thread>

int main ( int argc, char** argv )
{


  // Initialize the ROS node

  ros::init(argc, argv, "active_perception_node");
  // Create the robot after you have launch the URDF on the parameter server
  std::shared_ptr<arl::robot::Model> model = std::make_shared<arl::robot::ROSModel>("/robot/robot_description");

  // Create a simulated robot, use can use a real robot also
  // std::shared_ptr<arl::robot::Robot> robot = std::make_shared<arl::robot::RobotSim>(model, 1e-3);
  auto robot = std::make_shared<arl::bac::Robot>(model);
  
  // Create a visualizater for see the result in rviz
  auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot, "/robot/arm/joint_states");
  std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);

  // Create the joint trajectory controller
  // auto joint_trajectory = std::make_shared<arl::controller::JointTrajectory>(robot);

  // Create the active perception controller
  std::shared_ptr<arl::controller::ActivePerceptionController> ap_controller = std::make_shared<arl::controller::ActivePerceptionController>(robot);

  robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  // joint_trajectory->reference({0.0, -1.71832, -1.62427, -1.08471,  1.82317,  1.00598, 0.0, -1.71832, -1.62427, -1.08471,  1.82317,  1.00598}, 20);

  // Run the trajectory controller
  // joint_trajectory->run();
  std::cout<<"Running controller."  << std::endl;
  // Run the active perception controller
  
  ap_controller->run();
  //ap_controller->init();
  //ros::spin();

  std::cout<<"Ended controller."  << std::endl;

  return 0;
}
