/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for detecting the object
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Gaussian image";

class ObjectDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::master::V_TopicInfo topic_info;
  bool is_simulation;

public:
  ObjectDetector()
    : it_(nh_)
  {

    ros::master::getTopics(topic_info);
    isSimulation();
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 20000,
      &ObjectDetector::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/bw/image", 30000);

    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
  }

  ~ObjectDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow("Original image");
    cv::destroyWindow("Inverted image");
  }

/*------------------------------------
 Check if we are executing a gazebo simulation:
-----------------------------------*/
void isSimulation(){
    int i = 0;
    is_simulation = false;
    while(i < topic_info.size() and !is_simulation){
        if(topic_info[i].name == "/simulation/pose"){
            is_simulation = true;
        }
        i++;
    }
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
		imshow("Original image", cv_ptr->image);
		waitKey(3);
		
    cv_ptr->image = ~(cv_ptr->image);
		imshow("Inverted image", cv_ptr->image);
		waitKey(3);
		
    Mat3b imageHSV;
    if(!is_simulation){
      GaussianBlur(cv_ptr->image, cv_ptr->image, Size(15,15), 7, 7);
    }
    cvtColor(cv_ptr->image, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, Scalar(90 - 10, 100, 100), Scalar(90 + 10, 255, 255), cv_ptr->image);
    
    GaussianBlur(cv_ptr->image, cv_ptr->image, Size(3,3), 3, 3);
   	
    // Actualizar ventana
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_object");
  ObjectDetector od;
  ros::spin();
  return 0;
}
