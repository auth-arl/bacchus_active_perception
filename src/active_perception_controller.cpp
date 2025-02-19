#include <active_perception/active_perception_controller.h>
#include <autharl_core/math/orientation.h>

namespace arl
{
namespace controller
{

// this function finds the closest (in time) stamp from a list of time stamps
int ActivePerceptionController::findClosestStamp(double current_stamp,std::vector<double> stamp_list, bool search_past)
{
  // a big value
  double min_dur = 10000000000;

  // initial index, signifies that no past timestamp was found
  int closest_index = -1;

  // for all the time stamps in the list
  for(int i = 0; i<stamp_list.size(); i++){

    // this is the duration from current time
    double dur = current_stamp - stamp_list[i] ;

    if(!search_past){
      dur = fabs(dur);
    }

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

  // std::cout<<"min_dur= "<<min_dur <<std::endl;

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
  // PinholeCamera phC(894.187, 894.187, 642.957, 361.011, 1280, 720);
  PinholeCamera phC(1056.782470703125, 1056.782470703125, 1102.609130859375, 618.5433959960938, 2208, 1242);
  // K: [1056.782470703125, 0.0, 1102.609130859375, 0.0, 1056.782470703125, 618.5433959960938, 0.0, 0.0, 1.0]


 
   
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
  stem_paint_color[0] = 0;   // B
  stem_paint_color[1] = 0;   // G
  stem_paint_color[2] = 255; // R

  int pp_index = 0;


  

  
  // finds the closest timestamo from the current one, both in RGB and Depth
  int closest_rgb_index = findClosestStamp(current_stamp, image_stamps, true);
  int closest_depth_index = findClosestStamp(current_stamp, depth_stamps, false);


  pcl::PointCloud<pcl::PointXYZ> stem_pcl;

  cv::Mat temp_image_2 ;
  //temp_image_2 = cv::Mat::zeros(temp_image_2.rows,temp_image_2.cols,image_buffer[closest_rgb_index].type());

  depth_buffer[closest_depth_index].convertTo(temp_image_2, image_buffer[closest_rgb_index].type(), 1);

  
  // printouts
  // std::cout << "closest_rgb_index:  " << closest_rgb_index << std::endl;
  // std::cout << "closest_depth_index:  " << closest_depth_index << std::endl;
   
  // if there are stems identified and the closest DEPTH index is not -1
  if(num_stems>=1 && closest_depth_index>=0 && count_images>buffer_size){

    // initialize stem points  
    stem_mask_npoints = 0;

    // set downsampling coefficient, e.g. downsample by 80 times
    int downsampling = 1; 

    // the default size of the array by CERTH is 25000 points
    pp_index = 0;
    for (int i = 0; i<25000/downsampling; i++){

        // the downsampling index
        int ds_index = (i*downsampling);

        // if the index is not -1stem_pc_pub
        if(feedback->data[ds_index]!=-1){

          // then we have a stem point
          stem_mask_npoints = stem_mask_npoints + 1 ;

          // for x and y
          for (int j = 0; j<2; j++){

            // set the stem mask point
            stem_mask(i,j) = feedback->data[ds_index*2 + j];

          }

          // find the point's position with respect to the camera
          Eigen::Vector2d p2d;
          // int offset_width = 296;
          // int offset_height = 55;
          int offset_width = 592;
          int offset_height = 109;
          int elem1 = offset_width + stem_mask(i,1); // we add the offset of CERTH
          int elem2 =  offset_height +  stem_mask(i,0); // we add the offset of CERTH
          p2d << elem1, elem2;
          // back project
          Eigen::Vector3d point_temp = phC.backProject( p2d , (double)depth_buffer[closest_depth_index].at<float>(cv::Point(elem1, elem2)));

          float d_temp = depth_buffer[closest_depth_index].at<float>(cv::Point(elem1, elem2));



           std::cout  << d_temp <<" , ";
           
          // register the point in the stem point cloud
          if(d_temp > -1e4 && d_temp< 1e4 && d_temp == d_temp ){
            stem_pointCloud.col(pp_index) = point_temp;
            pp_index = pp_index + 1;
            std::cout<< d_temp  <<std::endl;
          }

          //change the color of the point in the RGB image
          image_buffer[closest_rgb_index].at<cv::Vec3b>(cv::Point(elem1, elem2)) = stem_paint_color;
          temp_image_2.at<cv::Vec3b>(cv::Point(elem1, elem2)) = stem_paint_color;

        }
         

    }
    std::cout << std::endl;


    

 

    // In this point, we identify if we have a square of 100x100, which is considered to be an error
    //if(stem_mask(9999,0) - stem_mask(0,0) == 99 &&  stem_mask(9999,1) - stem_mask(0,1) == 99 || pp_index==0){
    if(pp_index==0){
      
      std::cout<<"-------------------------------------------[!WARNING!] Identification is rejected ..." <<std::endl; 
      // this is not a valid stem identification
      valid_ident = false;
    }else{
      // this is a valid stem identification !
      valid_ident = true;
      
      // so the stem number of points is set to the counted points
      stem_num_points = pp_index;

      // compute the mean of all the stem points
      // Eigen::MatrixXd temp_mat = stem_pointCloud.leftCols(stem_num_points);
      p_Target_vision = stem_pointCloud.leftCols(stem_num_points).rowwise().mean();
      std::cout << "the new target is at:" << p_Target_vision.transpose() <<std::endl;


      /////////////////////////// UNCOMMENT TO PUBLISH STEM POINT CLOUD //////////////////
      stem_pcl.points.resize (stem_num_points);
      for (int i = 0; i<stem_num_points; i++){
        Eigen::Vector3f temp_point;
        for (int j = 0; j<3; j++){
          temp_point(j) = (float)stem_pointCloud(j,i);
        }
        stem_pcl.points[i].getVector3fMap () = temp_point;
      }

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(stem_pcl, msg_out);
    msg_out.header.frame_id = "zedB_left_camera_optical_frame";

    stem_pc_pub.publish(msg_out);
    std::cout << "Stem point cloud published. " << std::endl;
    //ros::spinOnce();
    //////////////////////////////////////////////////////////////////////////////////


    }



  //temp_image_2.at()

    // printouts
    //  std::cout << "stem_mask_npoints: " << stem_mask_npoints << std::endl;
    //  std::cout << "identification  timestamp:  " << current_stamp << std::endl;
    
    // std::cout << "rgb timestamps:  ";
    // for(int ll = 0; ll<buffer_size; ll++)  std::cout << image_stamps[ll] << " ";
    // std::cout << std::endl;
    //  std::cout << "closest rgb timestamp:  " << image_stamps[closest_rgb_index] << std::endl;
     
    //   std::cout << "depth timestamps:  ";
    // for(int ll = 0; ll<buffer_size; ll++) std::cout  << depth_stamps[ll] << " ";
    // std::cout << std::endl;
    //  std::cout << "closest depth timestamp:  " << depth_stamps[closest_depth_index] << std::endl;
    // std::cout << "stem_num_points: " << stem_num_points<< std::endl;
    // std::cout << "stem_pointCloud: " << stem_pointCloud.leftCols(stem_num_points) << std::endl;

    // CV image display
    cv::Mat temp_image;

    // uncomment to plot RGB IMAGE
    // cv::resize(image_buffer[closest_rgb_index], temp_image, cv::Size(image_buffer[closest_rgb_index].cols/2, image_buffer[closest_rgb_index].rows/2));

    // uncomment to plot Depth IMAGE
    cv::resize(temp_image_2, temp_image, cv::Size(temp_image_2.cols/2, temp_image_2.rows/2));
    
    if(show_image){
      // uncommet to show image
      cv::imshow("view", temp_image_2);
      cv::waitKey(1);
    }
    
  }
}

// this is a callback for the RGB image of the ZED2 camera (left)
void ActivePerceptionController::rgbCallback(const sensor_msgs::Image::ConstPtr rgb_feedback)
{
 
  //  std::cout<<":: RGB LEFT MESSAGE :: "  << std::endl;

   count_images = count_images + 1;

  // Get timestamp in DOUBLE format
  std::string time_stamp_temp = std::to_string(rgb_feedback->header.stamp.toNSec());
  time_stamp_temp = time_stamp_temp.substr(7, 3) + "." + time_stamp_temp.substr(10, 3) ;
 
  // increase the ring buffer's index
  cyclic_index_rgb = cyclic_index_rgb + 1;
  cyclic_index_rgb = cyclic_index_rgb % buffer_size;

  // register the image to the last (int time) position of the buffer
  image_buffer[cyclic_index_rgb] = (cv_bridge::toCvShare(rgb_feedback, "bgr8")->image).clone() ;
  image_stamps[cyclic_index_rgb] = std::stod(time_stamp_temp);
  //  std::cout<<"RGB Stamp: " <<  image_stamps[cyclic_index_rgb] << std::endl;
  // std::cout<<"Image size: " <<  image_buffer[cyclic_index_rgb].size() << std::endl;

}

// this is a callback for the DEPTH matrix of the ZED2 camera
void ActivePerceptionController::depthCallback(const sensor_msgs::Image::ConstPtr depth_feedback)
{
  

  PinholeCamera phC(1056.782470703125, 1056.782470703125, 1102.609130859375, 618.5433959960938, 2208, 1242);

  // std::cout<<":: DEPTH MESSAGE :: "  << std::endl;
  // Get timestamp in DOUBLE format
  std::string time_stamp_temp = std::to_string(depth_feedback->header.stamp.toNSec());
  time_stamp_temp = time_stamp_temp.substr(7, 3) + "." + time_stamp_temp.substr(10, 3) ;
 
  // std::cout<<":: DEPTH MESSAGE :: Testpoint 1"  << std::endl;

  // increase the ring buffer's index
  cyclic_index_depth = cyclic_index_depth + 1;
  cyclic_index_depth = cyclic_index_depth % buffer_size;

  // std::cout<<":: DEPTH MESSAGE :: Testpoint 2"  << std::endl;

  // register the image to the last (int time) position of the buffer
  // TODO: Search for the 32FC1 type. Is it correct?
  depth_buffer[cyclic_index_depth] = (cv_bridge::toCvShare(depth_feedback, "32FC1")->image).clone() ;
  //depth_buffer[cyclic_index_depth] = cv_bridge::toCvShare(depth_feedback, "mono8")->image ;
  
  //  std::cout<<":: DEPTH MESSAGE :: Testpoint 2.2"  << std::endl;
  depth_stamps[cyclic_index_depth] = std::stod(time_stamp_temp);
  //std::cout<<"Depth Stamp: " <<  depth_stamps[cyclic_index_depth]<< std::endl;

  // std::cout<<"------------------------------------- DEPTH ::::::: "<<std::endl;
  std::cout << "Depth------> " <<depth_buffer[cyclic_index_depth].at<float>(cv::Point(2208/2, 1242/2))<<std::endl;
  std::cout << "Size depth -> " <<depth_buffer[cyclic_index_depth].size()<<std::endl;

  ///////////////////// UNCOMMEND TO PUBLISH POINT CLOUD ///////////////////////
  // pcl::PointCloud<pcl::PointXYZ> whole_pc;
  // whole_pc.points.resize (2208 * 1242);

  // for(int i = 0; i<1242; i++){
  //   for(int j = 0; j<2208; j++){

  //     Eigen::Vector2d p2d;
  //     p2d << j, i;

  //     Eigen::Vector3d point_temp = phC.backProject( p2d , depth_buffer[cyclic_index_depth].at<float>(cv::Point(j, i)));
  //     Eigen::Vector3f point_temp2;
      
  //     for(int ll= 0; ll<3; ll++){
  //       point_temp2(ll) = (float)point_temp(ll);
  //     }

  //    whole_pc.points[i*2208+ j].getVector3fMap() = point_temp2;

  //   }
  // }

  // sensor_msgs::PointCloud2 msg_out;
  // pcl::toROSMsg(whole_pc, msg_out);
  // msg_out.header.frame_id = "zedB_left_camera_optical_frame";

  // whole_pc_pub.publish(msg_out);
  // std::cout << "Whole point cloud published. " << std::endl;
  /////////////////////////////////////////////////////////////////////////



      //ros::spinOnce();


  // cv::Mat temp_image;

  //   // uncomment to plot Depth IMAGE

  //   //cv::circle(temp_image,cv::Point(2208/2, 1242/2),400/32,cv::Scalar(0,0,255),cv::FILLED,cv::LINE_8); 

  //   cv::resize(depth_buffer[cyclic_index_depth], temp_image, cv::Size(depth_buffer[cyclic_index_depth].cols/2,depth_buffer[cyclic_index_depth].rows/2));
    
   
  //   // uncommet to show image
  //   cv::imshow("view", temp_image);
  //   cv::waitKey(1);
   
    

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

  nh.getParam("kinematic_chain", kinematic_chain);

  nh.getParam("show_image", show_image);




  stem_pc_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("stem_pointcloud", 1);
  //whole_pc_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("whole_pointcloud", 1);
  

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
  depth_buffer.resize(buffer_size);
  image_stamps.resize(buffer_size);
  depth_stamps.resize(buffer_size);

  count_images = 0;

  // get camera pose
  Eigen::Affine3d tool_pose= robot->getTaskPose(kinematic_chain);

  // initialization of the center of ROI
  p_camera = tool_pose.translation();
  R_camera = tool_pose.rotation();
  p_Target_world = p_camera + 0.5 * R_camera.col(2);
  // p_Target_world(0) = p_Target_world(0) + 0.15;
  // p_Target_world(1) = p_Target_world(1) + 0.25;
  p_Target_vision = Eigen::Vector3d::Zero(3);

   std::cout << "p_camera: " << p_camera <<std::endl;

  // set the radius of ROI
  r_region = 0.3;

  // get the end-effector Jacobian
  J_ee = robot->getJacobian(kinematic_chain);
  J_ee.topRows(3) = R_camera.transpose() * J_ee.topRows(3);
  J_ee.bottomRows(3) = R_camera.transpose() * J_ee.bottomRows(3);


  

  // initialize filter velocity
  v_filter = Eigen::VectorXd::Zero(6);

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
  vision_target = false;


  // start the subscription 
  stem_subscriber = nh.subscribe("/stem_coords", 1, &ActivePerceptionController::stemIdentCallback, this, ros::TransportHints().tcpNoDelay(true)); 
  rgb_subscriber = nh.subscribe("/zed_node_B/left/image_rect_color", 1, &ActivePerceptionController::rgbCallback, this, ros::TransportHints().tcpNoDelay(true)); 
  depth_subscriber = nh.subscribe("/zed_node_B/depth/depth_registered", 1, &ActivePerceptionController::depthCallback, this, ros::TransportHints().tcpNoDelay(true)); 


  // printouts
  std::cout<<"Ending init. "  << std::endl;
}

// this is the measure function
void ActivePerceptionController::measure()
{

  // printout
  //  std::cout<<"Staring measure. "  << std::endl;

  //if the identification found a stem
  if(valid_ident){
    // then we switch to relative target
    vision_target = true;
     
   
  }

  // get camera pose
  Eigen::Affine3d tool_pose= robot->getTaskPose(kinematic_chain);
  p_camera = tool_pose.translation();
  R_camera = tool_pose.rotation();

  Eigen::VectorXd q= robot->getJointPosition(kinematic_chain);
  // std::cout<<"q " << q.transpose() <<std::endl;

  // std::cout << "p_camera: " << p_camera <<std::endl;
  // std::cout << "R_camera: " << arl::math::rotToQuat(R_camera.toArma()) <<std::endl;

  //Eigen::VectorXd q_current = robot->getJointPosition(kinematic_chain);
  //std::cout << "q_current: " << q_current <<std::endl;


  // get the end-effector Jacobian
  J_ee = robot->getJacobian(kinematic_chain);
  J_ee.topRows(3) = R_camera.transpose() * J_ee.topRows(3);
  J_ee.bottomRows(3) = R_camera.transpose() * J_ee.bottomRows(3);

  // std::cout<<"J_ee: " << J_ee <<std::endl;
  // std::cout<<"q: " <<  robot->getJointPosition(kinematic_chain).transpose() <<std::endl;

  // std::cout<<"Ending measure. "  << std::endl;
}

// this is the update function
void ActivePerceptionController::update()
{


// printout
//  std::cout<<"Staring update. "  << std::endl;

  // general purpose z vector
  Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
  z(2) = 1;
  

  // Uncomment this to use vision
  // vision_target = false;


  // region reaching signal
  Eigen::Vector3d e_p;
  if(vision_target){
    e_p = p_Target_vision;
  }else{
    e_p = R_camera.transpose() * (p_Target_world - p_camera);
  }

  // std::cout<<"p_Target_world " << p_Target_world.transpose() <<std::endl;
  // std::cout<<"p_camera " << p_camera.transpose() <<std::endl;
  // std::cout<<"e_p " << e_p.transpose() <<std::endl;
  // std::cout<<"p_Target_vision: " << p_Target_vision.transpose() <<std::endl;

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
  Eigen::VectorXd vddot = Eigen::VectorXd::Zero(6);
  
  vddot = ( k_filter * u - d_filter * v_filter ) / m_filter;
  // integrate dynamical system -----
  v_filter = v_filter + vddot * Tc;


  //  std::cout<<"Ending update. "  << std::endl;
}

//this is the command function
void ActivePerceptionController::command()
{

  // printout
  //  std::cout<<"Staring command. "  << std::endl;

  // Map velocity to the joint space
  Eigen::VectorXd qr =  J_ee.pinv() * v_filter;

  // std::cout<<"qr= "  << qr.transpose() << std::endl;
  //  std::cout<<"v_filter= "  << v_filter.transpose() << std::endl;

  // set joint velocity reference
  robot->setJointVelocity(qr.setZero(), kinematic_chain);

  // spin the ROS callbacks
  ros::spinOnce();
  // std::cout<<"Ending command. "  << std::endl;
}

}  // namespace controller
} // namespace arl