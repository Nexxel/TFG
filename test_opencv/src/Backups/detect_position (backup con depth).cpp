#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

static const std::string OPENCV_WINDOW = "Image window";

class PositionDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber scan_sub_depth_;
  ros::Publisher image_pub_position_;
  ros::Publisher scan_pub_depth_;
  std_msgs::String position;
  std_msgs::String depth;
  cv::Mat locations;
  
   double mean_x;
	 double mean_y;

public:
  PositionDetector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
      &PositionDetector::imageCb, this);
    scan_sub_depth_ = it_.subscribe("/camera/depth/image_raw", 1,
    	&PositionDetector::imageDepth, this);
    image_pub_position_ = nh_.advertise<std_msgs::String>("/image_converter/position", 1);
    scan_pub_depth_ = nh_.advertise<std_msgs::String>("/image_converter/depth", 1);
  }
	
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Transformar imagen de color a blanco y negro, siendo blanco los objetos rojos de la imagen
    cv_ptr->image = ~(cv_ptr->image);

    cv::Mat3b imageHSV;
    cv::cvtColor(cv_ptr->image, imageHSV, cv::COLOR_BGR2HSV);
    cv::inRange(imageHSV, cv::Scalar(90 - 10, 70, 50), cv::Scalar(90 + 10, 255, 255), cv_ptr->image);
    // Coger coordenadas de los puntos blancos de la imagen

    cv::findNonZero(cv_ptr->image, locations);
    // Calcular media de los puntos
    int sum_x = 0; int sum_y = 0;
    for (int i = 0; i<locations.rows; i++){
    	sum_x += locations.at<int>(i,0);
    	sum_y += locations.at<int>(i,1);
    }
    
    int height = cv_ptr->image.rows; 
    int width = cv_ptr->image.cols;
    position.data = ""; 
    
    if(locations.rows != 0){
		  mean_x = sum_x/locations.rows;
		  mean_y = sum_y/locations.rows;
		  if (mean_x < floor(width/2)-1){
		  	position.data += "IZQUIERDA ";
		  }else if (mean_x > ceil(width/2)+1){
		  	position.data += "DERECHA ";
		  }else{
		  	position.data += "CENTRO ";
		  }
		  if (mean_y < floor(height/2)-1){
		  	position.data += "ARRIBA";
		  }else if (mean_y > ceil(height/2)+1){
		  	position.data += "ABAJO";
		  }else{
		  	position.data += "CENTRO";
		  }
	  }else{
	  	position.data = "SIN OBJETOS ROJOS";
	  	ROS_INFO("No hay objetos rojos en la imagen");
	  }

    // Modificar datos de salida
    image_pub_position_.publish(position);
  }
  
  void imageDepth(const sensor_msgs::ImageConstPtr& msg)
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
    
    float sum = (float)cv_ptr->image.at<uchar>(mean_x,mean_y);
    
    depth.data = "";
    
    if (locations.rows != 0){
    	double mean = sum;
    	ROS_INFO("Depth mean: %.2f",mean);
		  if (mean <= 63.75){
		  	depth.data += "MUY CERCA";
		  }else if (mean > 63.75 and mean <= 127.5){
		  	depth.data += "CERCA";
		  }else if(mean > 191.25){
		  	depth.data += "MUY LEJOS";
		  }else if(mean != mean){ //nan values
		  	depth.data += "SIN DATOS";
		  }else{
		  	depth.data += "LEJOS";
		  }
    }else{
    	depth.data = "SIN OBJETOS ROJOS";
    }
    
    // Modificar datos de salida
    scan_pub_depth_.publish(depth);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_position");
  PositionDetector pd;
  ros::spin();
  return 0;
}
