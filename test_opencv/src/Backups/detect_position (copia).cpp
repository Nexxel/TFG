#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";

class PositionDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber scan_sub_depth_;
  ros::Publisher image_pub_position_;
  ros::Publisher image_pub_localization_;
  std_msgs::String position;
  std_msgs::Int64MultiArray localization;
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
    image_pub_position_ = nh_.advertise<std_msgs::String>("/image_converter/position", 1); // Posición relativa
    image_pub_localization_ = nh_.advertise<std_msgs::Int64MultiArray>("/image_converter/localization", 1); // Posición central
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
    localization = std_msgs::Int64MultiArray();
    
    if(locations.rows != 0){
		  float mean_x = sum_x/locations.rows;
		  float mean_y = sum_y/locations.rows;
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
		  localization.data[0] = round(mean_x);
		  localization.data[1] = round(mean_y);
	  }else{
	  	position.data = "SIN OBJETOS ROJOS";
	  	ROS_INFO("No hay objetos rojos en la imagen");
	  }

    // Modificar datos de salida
    image_pub_position_.publish(position);
    image_pub_localization_.publish(localization);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_position");
  PositionDetector pd;
  ros::spin();
  return 0;
}
