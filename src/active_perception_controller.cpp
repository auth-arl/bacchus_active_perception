#include <active_perception/active_perception_controller.h>


namespace arl
{
namespace controller
{

// this function finds the closest (in time) stamp from a list of time stamps
int ActivePerceptionController::findClosestStamp(double current_stamp,std::vector<double> stamp_list)
{
  // a big value
  double min_dur = 10000000000;

  // initial index, signifies that no past timestamp was found
  int closest_index = -1;

  // for all the time stamps in the list
  for(int i = 0; i<stamp_list.size; i++){

    // this is the duration from current time
    double dur = current_stamp - stamp_list[i] ;

    // if this time stamp is a past timestamp
    if(dur>=0){

      // and if it is smaller than the minimum one
      if(dur < min_dur){

        // this is the new minimum
        min_dur = dur;

        // and this is the new index
        closest_index = i;
      }
    }
  }

  // return
  return closest_index;

}

// this is the callback for the stem identification result of CERTH
void ActivePerceptionController::stemIdentCallback(const std_msgs::Int32MultiArray::ConstPtr feedback)
{
  
  // This is the numver of stems found
  int num_stems = feedback->layout.dim[0].size; 

  // initializing the pinhole camera
  // THIS REQUIRES THE INTRINSIC CALIBRATION PARAMETERS
  PinholeCamera phC(894.187, 894.187, 642.957, 361.011, 1280, 720);
 
   
  std::cout<<"-----------------===== STEM MESSAGE ======---------------------- "  << std::endl;

  // Find the timestamp of the corresponding image 
  std::string label = feedback->layout.dim[0].label;
  // example: 345.232
  label = label.substr(13, 3) + "." + label.substr(16, 3) ;
  // convert it to DOUBLE
  double current_stamp = std::stod(label);
  std::cout<<"Label: " << current_stamp << std::endl;
  std::cout<<"Stems identified: " << num_stems << std::endl;

  // this is the color for painting the ste in the RGB result
  cv::Vec3b stem_paint_color;
  color[0] = 0;   // B
  color[1] = 0;   // G
  color[2] = 255; // R

  
  // finds the closest timestamo from the current one, both in RGB and Depth
  int closest_rgb_index = findClosestStamp(current_stamp, image_stamps);
  int closest_depth_index = findClosestStamp(current_stamp, depth_stamps);
  
  // printouts
  std::cout << "closest_rgb_index:  " << closest_rgb_index << std::endl;
  std::cout << "closest_depth_index:  " << closest_depth_index << std::endl;
   
  // if there are stems identified and the closest DEPTH index is not -1
  if(num_stems>=1 && closest_depth_index>=0){

    // initialize stem points  
    stem_mask_npoints = 0;

    // set downsampling coefficient, e.g. downsample by 80 times
    int downsampling = 80; 

    // the default size of the array by CERTH is 25000 points
    for (int i = 0; i<25000/downsampling; i++){

        // the downsampling index
        int ds_index = (i*downsampling);

        // if the index is not -1
        if(feedback->data[ds_index]!=-1){

          // then we have a stem point
          stem_mask_npoints = stem_mask_npoints + 1 ;

          // for x and y
          for (int j = 0; j<2; j++){

            // set the stem mask point
            stem_mask(i,j) = feedback->data[ds_index*2 + j];

          }

          // find the point's position with respect to the camera
          Eigen::Vector2f p2d;
          int elem1 = 592 + stem_mask(i,1); // we add the offset of CERTH
          int elem2 =  109 +  stem_mask(i,0); // we add the offset of CERTH
          p2d << elem1, elem2;
          // back project
          Eigen::Vector3f point_temp = phC.backProject( p2d , depth_buffer[closest_depth_index].at( elem1, elem2 ) );
          // register the point in the stem point cloud
          stem_pointCloud.column(i) = point_temp;

          //change the color of the point in the RGB image
          image_buffer[closest_rgb_index].at<cv::Vec3b>(cv::Point(elem1, elem2)) = color;

        }
    }


 

    // In this point, we identify if we have a square of 100x100, which is considered to be an error
    if(stem_mask(9999,0) - stem_mask(0,0) == 99 &&  stem_mask(9999,1) - stem_mask(0,1) == 99){
      
      std::cout<<"-------------------------------------------[!WARNING!] Identification is rejected ... Reason: 100x100 square!!!" <<std::endl; 
      // this is not a valid stem identification
      valid_ident = false;
    }else{
      // this is a valid stem identification !
      valid_ident = true;
      
      // so the stem number of points is set to the counted points
      stem_num_points = stem_mask_npoints;

      // compute the mean of all the stem points
      p_Target_vision = stem_pointCloud.rows(0,stem_num_points-1).columnwise().mean();
    }

    // printouts
    std::cout << "stem_mask_npoints: " << stem_mask_npoints << std::endl;
    std::cout << "closest rgb timestamp:  " << image_stamps[closest_rgb_index] << std::endl;

    // CV image display
    cv::Mat temp_image;
    cv::resize(image_buffer[closest_image_index], temp_image, cv::Size(image_buffer[closest_rgb_index].cols/2, image_buffer[closest_image_index].rows/2));
    cv::imshow("view", temp_image);
    cv::waitKey(1);
    
  }
}

// this is a callback for the RGB image of the ZED2 camera (left)
void ActivePerceptionController::rgbCallback(const sensor_msgs::Image::ConstPtr rgb_feedback)
{
 
  //std::cout<<":: RGB LEFT MESSAGE :: "  << std::endl;

  // Get timestamp in DOUBLE format
  std::string time_stamp_temp = std::to_string(rgb_feedback->header.stamp.toNSec());
  time_stamp_temp = time_stamp_temp.substr(7, 3) + "." + time_stamp_temp.substr(10, 3) ;
 
  // increase the ring buffer's index
  cyclic_index_rgb = cyclic_index_rgb + 1;
  cyclic_index_rgb = cyclic_index_rgb % buffer_size;

  // register the image to the last (int time) position of the buffer
  image_buffer[cyclic_index_rgb] = cv_bridge::toCvShare(rgb_feedback, "bgr8")->image ;
  image_stamps[cyclic_index_rgb] = std::stod(time_stamp_temp);
  //std::cout<<"Stamp: " <<  image_stamps[cyclic_index_rgb]<< std::endl;

}

// this is a callback for the DEPTH matrix of the ZED2 camera
void ActivePerceptionController::depthCallback(const sensor_msgs::Image::ConstPtr depth_feedback)
{
  
  std::cout<<":: DEPTH MESSAGE :: "  << std::endl;
  // Get timestamp in DOUBLE format
  std::string time_stamp_temp = std::to_string(depth_feedback->header.stamp.toNSec());
  time_stamp_temp = time_stamp_temp.substr(7, 3) + "." + time_stamp_temp.substr(10, 3) ;
 
  // increase the ring buffer's index
  cyclic_index_depth = cyclic_index_depth + 1;
  cyclic_index_depth = cyclic_index_depth % buffer_size;

  // register the image to the last (int time) position of the buffer
  // TODO: Search for the 32FC1 type. Is it correct?
  depth_buffer[cyclic_index_depth] = cv_bridge::toCvShare(depth_feedback, "32FC1")->image ;
  depth_stamps[cyclic_index_depth] = std::stod(time_stamp_temp);
  std::cout<<"Stamp: " <<  depth_stamps[cyclic_index_depth]<< std::endl;

}


// the constructor
ActivePerceptionController::ActivePerceptionController(const std::shared_ptr<arl::robot::Robot>& robot) :
  arl::robot::Controller(robot, "Active Perception Controller")
{
  // the node handler
  nh = ros::NodeHandle("~");

  // get the params
  nh.getParam("k", k);
  nh.getParam("buffer_size", buffer_size);
  nh.getParam("k_filter", k_filter);
  nh.getParam("m_filter", m_filter);
  nh.getParam("d_filter", d_filter);

  // printout
  std::cout<<"Ending constructor. "  << std::endl;

}

// the initialization function
void ActivePerceptionController::init()
{

  // resize the stem mask matrix
  stem_mask.resize(25000,2);

  // initialize the cyclic indexes of the ring buffers that hold the latest RGB and Depth images
  cyclic_index_rgb = -1;
  cyclic_index_depth = -1;

  // the valid identification is initially set to false
  valid_ident = false;

  // resize the ring buffers that hold the latest RGB and Depth images
  image_buffer.resize(buffer_size);
  image_stamps.resize(buffer_size);

  // get camera pose
  Eigen::Affine3d tool_pose= robot->getTaskPose();

  // initialization of the center of ROI
  p_camera = tool_pose.translation();
  R_camera = tool_pose.rotation();
  p_Target_world = p_camera + 0.4 * R_camera.column(2);
  p_Target_vision = Eigen::Vector3d::Zero(3);

  // set the radius of ROI
  r_region = 0.3;

  // get the end-effector Jacobian
  J_ee = robot->getJacobian();
  J_ee.rows(0,2) = R_camera.transpose() * J_ee.rows(0,2);
  J_ee.rows(3,5) = R_camera.transpose() * J_ee.rows(3,5);

  // initialize filter velocity
  v_filter = Eigen::Vector3d::Zero(3);

  // get cycle time
  Tc = robot->cycle;

  // Initialize the CV display
  cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
  //cv::startWindowThread();

  // resize the pointcloud arrays
  stem_pointCloud.resize(3,10000);
  obs_pointCloud.resize(3,10000);

  // set the number of points to 0
  stem_num_points = 0;
  obs_num_points = 0;

  // initially the target is absolute
  relative_target = false;

  // start the subscription 
  stem_subscriber = nh.subscribe("/stem_coords", 1, &ActivePerceptionController::stemIdentCallback, this, ros::TransportHints().tcpNoDelay(true)); 
  rgb_subscriber = nh.subscribe("/zed_node_D/left/image_rect_color", 1, &ActivePerceptionController::rgbCallback, this, ros::TransportHints().tcpNoDelay(true)); 
  depth_subscriber = nh.subscribe("/zed_node_D/depth/depth_registered", 1, &ActivePerceptionController::depthCallback, this, ros::TransportHints().tcpNoDelay(true)); 

  // printouts
  std::cout<<"Ending init. "  << std::endl;
}

// this is the measure function
void ActivePerceptionController::measure()
{
  //if the identification found a stem
  if(valid_ident){
    // then we switch to relative target
    vision_target = true;
     
   
  }

  // get camera pose
  Eigen::Affine3d tool_pose= robot->getTaskPose();
  p_camera = tool_pose.translation();
  R_camera = tool_pose.rotation();

  // get Jacobian
  J_ee = robot->getTaskPose();
  p_camera = tool_pose.translation();
  R_camera = tool_pose.rotation();

  // get the end-effector Jacobian
  J_ee = robot->getJacobian();
  J_ee.rows(0,2) = R_camera.transpose() * J_ee.rows(0,2);
  J_ee.rows(3,5) = R_camera.transpose() * J_ee.rows(3,5);

  // std::cout<<"Ending measure. "  << std::endl;
}

// this is the update function
void ActivePerceptionController::update()
{

  // general purpose z vector
  Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
  z(2) = 1;

  Eigen::Vector3d e_p;

  // region reaching signal
  Eigen::Vector3d e_p;
  if(vision_target){
    e_p = p_Target_vision;
  }else{
    e_p = R_camera.transpose() * (p_Target_world - p_camera);
  }

  Eigen::VectorXd u = Eigen::VectorXd::Zero(6);
  // compute distance
  double dista = e_p.norm();

  // if not within the region
  if(dista>r_region){
      // quadratic potential
      u.head(3) =  5.0 * (dista * dista - r_region * r_region ) * e_p;
  }

  // this is the normal vector pointing towards the center of ROI (wrt to the camera frame)
  Eigen::Vector3d rachis_mean_norm = e_p / e_p.norm();

  // get the cross product 
  Eigen::Vector3d e_o =   z.cross( rachis_mean_norm );


  // get the dot product
  double temp_dot = z.dot( rachis_mean_norm );

  // compute eo with respect to the camera frame
  if(temp_dot>=1.0){
      e_o = Eigen::Vector3d::Zero(3);
  }else{
      e_o = e_o * acos( temp_dot) / e_o.norm();
  }

  // add the centering control signal to the total signal
  u.tail(3) =   5.0 * e_o;

  
  
  // compute the next acceleration
  Eigen::VectorXd vddot = ( k_filter * u - d_filter * v_filter ) / m_filter;
  // integrate dynamical system -----
  v_filter = v_filter + vddot * Tc;


  //  std::cout<<"Ending update. "  << std::endl;
}

//this is the command function
void ActivePerceptionController::command()
{

  // Map velocity to the joint space
  Eigen::VectorXd qr = J_ee.inverse() * v_filter;

  // set joint velocity reference
  robot->setJointVelocity(qr);

  // spin the ROS callbacks
  ros::spinOnce();
  // std::cout<<"Ending command. "  << std::endl;
}

}  // namespace controller
} // namespace arl