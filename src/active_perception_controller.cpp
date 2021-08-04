#include <active_perception/active_perception_controller.h>




namespace arl
{
namespace controller
{


void ActivePerceptionController::stemIdentCallback(const std_msgs::Int32MultiArray::ConstPtr feedback)
{
  
  int num_stems = feedback->layout.dim[0].size; 
 
   
  std::cout<<"-----------------===== STEM MESSAGE ======---------------------- "  << std::endl;
  std::string label = feedback->layout.dim[0].label;
  label = label.substr(13, 3) + "." + label.substr(16, 3) ;
  double current_stamp = std::stod(label);
  std::cout<<"Label: " << current_stamp << std::endl;
  std::cout<<"Stems identified: " << num_stems << std::endl;

  cv::Vec3b color;
  color[0] = 0;
  color[1] = 0;
  color[2] = 255;

  double min_dur = 10000000000;
    int closest_image_index = -1;
    for(int i = 0; i<buffer_size; i++){
      double dur = current_stamp - image_stamps[i] ;
      if(dur>=0){
        if(dur < min_dur){
          min_dur = dur;
          closest_image_index = i;
        }
      }
    }
  

    std::cout << "Min dur: " << min_dur << std::endl;
    std::cout << "closest_image_index:  " << closest_image_index << std::endl;
   

  if(num_stems>=1 && closest_image_index>=0){

    stem_mask_npoints = 0;

    int downsampling = 80; 

    for (int i = 0; i<25000/downsampling; i++){

        int ds_index = (i*downsampling);
        if(feedback->data[ds_index]!=-1){

          stem_mask_npoints = stem_mask_npoints + 1 ;

          for (int j = 0; j<2; j++){

                
            stem_mask(i,j) = feedback->data[ds_index*2 + j];

          }

          image_buffer[closest_image_index].at<cv::Vec3b>(cv::Point(592 + stem_mask(i,1), 109 +  stem_mask(i,0))) = color;

        }


      

    }


    
    if(stem_mask(9999,0) - stem_mask(0,0) == 99 &&  stem_mask(9999,1) - stem_mask(0,1) == 99){
      
      std::cout<<"-------------------------------------------[!WARNING!] Identification is rejected ... Reason: 100x100 square!!!" <<std::endl; 
      valid_ident = false;
    }else{
      valid_ident = true;
    }

    std::cout << "stem_mask_npoints: " << stem_mask_npoints << std::endl;
    std::cout << "closest image timestamp:  " << image_stamps[closest_image_index] << std::endl;
    cv::Mat temp_image;
    cv::resize(image_buffer[closest_image_index], temp_image, cv::Size(image_buffer[closest_image_index].cols/2, image_buffer[closest_image_index].rows/2));
    cv::imshow("view", temp_image);
    cv::waitKey(1);
    
  
  }



}

void ActivePerceptionController::rgbCallback(const sensor_msgs::Image::ConstPtr rgb_feedback)
{
  
 
  std::cout<<":: RGB LEFT MESSAGE :: "  << std::endl;
  std::string time_stamp_temp = std::to_string(rgb_feedback->header.stamp.toNSec());
  time_stamp_temp = time_stamp_temp.substr(7, 3) + "." + time_stamp_temp.substr(10, 3) ;
 

  cyclic_index = cyclic_index + 1;
  cyclic_index = cyclic_index % buffer_size;

  image_buffer[cyclic_index] = cv_bridge::toCvShare(rgb_feedback, "bgr8")->image ;
  image_stamps[cyclic_index] = std::stod(time_stamp_temp);
  std::cout<<"Stamp: " <<  image_stamps[cyclic_index]<< std::endl;



}



ActivePerceptionController::ActivePerceptionController(const std::shared_ptr<arl::robot::Robot>& robot) :
  arl::robot::Controller(robot, "Active Perception Controller")
{
  nh = ros::NodeHandle("~");

  nh.getParam("k", k);


  stem_mask.resize(25000,2);
  cyclic_index = -1;
  buffer_size = 3;

  valid_ident = false;

  image_buffer.resize(buffer_size);
  image_stamps.resize(buffer_size);

  cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
  //cv::startWindowThread();



  stem_subscriber = nh.subscribe("/stem_coords", 1, &ActivePerceptionController::stemIdentCallback, this, ros::TransportHints().tcpNoDelay(true)); 
  rgb_subscriber = nh.subscribe("/zed_node_D/left/image_rect_color", 1, &ActivePerceptionController::rgbCallback, this, ros::TransportHints().tcpNoDelay(true)); 

  std::cout<<"Ending constructor. "  << std::endl;
}

void ActivePerceptionController::init()
{
  std::cout<<"Ending init. "  << std::endl;
}

void ActivePerceptionController::measure()
{
  // std::cout<<"Ending measure. "  << std::endl;
}

void ActivePerceptionController::update()
{
  //  std::cout<<"Ending update. "  << std::endl;
}

void ActivePerceptionController::command()
{
  ros::spinOnce();
  // std::cout<<"Ending command. "  << std::endl;
}

}  // namespace controller
} // namespace arl