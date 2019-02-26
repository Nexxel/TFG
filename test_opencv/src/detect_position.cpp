#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Red objects";

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
  Mat locations;
  
   double mean_x;
	 double mean_y;

public:
  PositionDetector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 10000,
      &PositionDetector::imageCb, this);
    scan_sub_depth_ = it_.subscribe("/camera/depth/image_raw", 15000,
    	&PositionDetector::imageDepth, this);
    image_pub_position_ = nh_.advertise<std_msgs::String>("/image_converter/position", 12500);
    scan_pub_depth_ = nh_.advertise<std_msgs::String>("/image_converter/depth", 17500);
    
    namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
    namedWindow("Depth image", CV_WINDOW_AUTOSIZE);
  }
  
  ~PositionDetector()
  {
    destroyWindow(OPENCV_WINDOW);
    destroyWindow("Depth image");
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

    // Invertimos la imagen para poder trabajar más cómodamente con el rojo
    cv_ptr->image = ~(cv_ptr->image);

    // Aplicamos un filtro gaussiano para eliminar gran parte de los reflejos
    cv::GaussianBlur(cv_ptr->image, cv_ptr->image, Size(9,9), 4, 4);
    
    // Aplicamos un umbral para escoger los elementos turquesa de la imagen invertida (Rojos en la original) 
    Mat3b imageHSV;
    cvtColor(cv_ptr->image, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, Scalar(90 - 10, 100, 100), Scalar(90 + 10, 255, 255), cv_ptr->image);
    
    // Volvemos a aplicar un filtro gaussiano para evitar posibles falsos positivos
    GaussianBlur(cv_ptr->image, cv_ptr->image, Size(9,9), 6, 6);
    
    // Actualizar ventana
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::imshow("Depth image", cv_ptr->image);
    
    float depth_center = (float)cv_ptr->image.at<uchar>(mean_x,mean_y);
    
    depth.data = "";
    
    if (locations.rows != 0){
    	ROS_INFO("Depth in center of the object: %.2f",depth_center);
		  if (depth_center < 1.00){
		  	depth.data += "MUY CERCA";
		  }else if (depth_center >= 1.00 and depth_center < 2.00){
		  	depth.data += "CERCA";
		  }else if(depth_center >= 2.00){
		  	depth.data += "LEJOS";
		  }else if(depth_center != depth_center){ //nan values
		  	depth.data += "SIN DATOS";
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
