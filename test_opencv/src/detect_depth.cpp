  #include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>
#include <sensor_msgs/LaserScan.h>

static const std::string OPENCV_WINDOW = "Image window";

class DepthDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_depth_;
  ros::Subscriber image_sub_loc_;
  ros::Publisher image_pub_depth_;
  std_msgs::String depth;

public:
  DepthDetector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_depth_ = it_.subscribe("/camera/depth/image_raw", 1,
    	boost::bind(DepthDetector::imageDepth, _1), this);
    image_sub_loc_ = nh_.subscribe("/image_converter/localization",1,
     boost::bind(DepthDetector::imageDepth, _2), this);
    image_pub_depth_ = nh_.advertise<std_msgs::String>("/image_converter/depth", 1);
  }
  void imageDepth(const sensor_msgs::ImageConstPtr& msg, const std_msgs::Float64MultiArray& localization_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Calcular media de profundidad de los puntos
    /*float sum = 0;
    for (int i = 0; i<locations.rows; i++){
    	sum += (float)cv_ptr->image.at<uchar>(locations.at<int>(i,0),locations.at<int>(i,1));
    }*/
    
    float depth_value = (float)cv_ptr->image.at<uchar>(localization_msg->localization.data[0],localization_msg->localization.data[1]);
    
    depth.data = "";
    
  	ROS_INFO("Depth mean: %.2f",depth_value);
	  if (depth_value <= 63.75){
	  	depth.data += "MUY CERCA";
	  }else if (depth_value > 63.75 and depth_value <= 127.5){
	  	depth.data += "CERCA";
	  }else if(depth_value > 191.25){
	  	depth.data += "MUY LEJOS";
	  }else if(depth_value != depth_value){ //nan values
	  	depth.data += "SIN DATOS";
	  }else{
	  	depth.data += "LEJOS";
	  }
    
    // Modificar datos de salida
    image_pub_depth_.publish(depth);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_depth");
  DepthDetector dd;
  ros::spin();
  return 0;
}
