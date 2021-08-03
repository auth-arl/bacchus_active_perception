#include <ros/ros.h>
#include <autharl_core/viz/ros_state_publisher.h>
#include <autharl_core/robot/ros_model.h>
#include <autharl_core/robot/robot.h>
#include <bacchus_robot/robot_dual.h>
#include <ppc/ppc.h>
#include <thread>

int main ( int argc, char** argv )
{
  // Initialize the ROS node
  ros::init(argc, argv, "active_perception_node");
  // Create the robot after you have launch the URDF on the parameter server
  auto model = std::make_shared<arl::robot::ROSModel>("/robot/robot_description");

  // Create a simulated robot, use can use a real robot also
  // auto robot = std::make_shared<arl::robot::RobotSim>(model, 1e-3);
  // auto robot = std::make_shared<arl::ur5e::Robot>(model);
  auto robot = std::make_shared<arl::bac::Robot>(model);

  // Create a visualizater for see the result in rviz
  auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot);

  // Create the joint trajectory controller
  auto joint_trajectory = std::make_shared<arl::controller::JointTrajectory>(robot);

  robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  joint_trajectory->reference({0.0, -1.71832, -1.62427, -1.08471,  1.82317,  1.00598, 0.0, -1.71832, -1.62427, -1.08471,  1.82317,  1.00598}, 20);
  std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);

  // Run the trajectory controller
  joint_trajectory->run();

  return 0;
}
