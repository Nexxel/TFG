/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Q-Learning implementation header file
*/

/*------------------------------------
 Libraries
-----------------------------------*/
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <time.h>
// Ros library
#include <ros/ros.h>
#include <ros/package.h>
// Using matrices easily
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>
// Image processing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Image filters
#include <opencv2/imgproc/imgproc.hpp>
// Image visualization
#include <opencv2/highgui/highgui.hpp>
// Ros messages
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

/*------------------------------------
 Definitions:
-----------------------------------*/
#define L3 0.1423
#define d2 0.1528
#define L45 0.1221 //(0.07585 + 0.04625)
#define beta 1.2496

#define MAX_EFFORT 0.03   //Max effort of the idle gripper ( totally opened or closed)

#define ROS_RATE 8

#define MAX_DISTANCE 1.0   //Max distance to perceive the object

#define OBJECT_HEIGHT 100.0  // Height of the object in mm
#define OBJECT_WIDTH 24.0 // Width of the object in mm
#define OBJECT_AREA (OBJECT_HEIGHT * OBJECT_WIDTH)

#define SENSOR_WIDTH 2.30 // Width of the sensor in mm
#define SENSOR_HEIGHT 1.73 // Height of the sensor in mm

#define CAMERA_WIDTH 640 // Width of the image in pixels
#define CAMERA_HEIGHT 480 // Height of the image in pixels

#define HEIGHT_PX_2_M (SENSOR_HEIGHT/CAMERA_HEIGHT)
#define WIDTH_PX_2_M (SENSOR_WIDTH/CAMERA_WIDTH)

// Min and max positions of the objects
#define MAX_X 1.5 //1.5
#define MIN_X 1.5 //0.85
#define MAX_Y 0.8 //0.8
#define MIN_Y 0.8 //-0.8
// Number of box.urdf to use (box_1 z-size=0.1, box_2 z-size=0.2...)
#define MIN_BOX 3
#define MAX_BOX 5

#define EXPLORATION_RATE 30
#define GAMMA 0.99

#define N_ACTIONS 5 // Number of actions

/*------------------------------------
 Some useful namespaces:
-----------------------------------*/
using namespace std;
using namespace arma;
using namespace cv;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
/*------------------------------------
 Variables
 -----------------------------------*/
 struct state{
   int height_d;   // Discretized height
   double height_c;    // Continuos height
   int distance_d;  // Discretized distance
   double distance_c;  // Continuous distance
   int angle_d;     // Discretized angle in horizontal
   double angle_c;     // Continuous angle in horizontal
   bool folded;        // Is the arm folded?
   bool object_picked; // Is the object picked?
 } robot_state;

int sa;                 // Actual state
int sp;                 // Posterior state
double reward;
string exploit;         // If you want just to exploit

double dist;            // Real distance
double d;               // Distance between V and V'
double e;               // Min absolute distance between V and V'

vec3 ang_or;            // Angle orientation of gripper //3x1
vec3 home_pos;          // Home position
vec3 n;                 // Direction of gripper
int discr_level = 9;    // Discretization level
double distance_per_level = (double)MAX_DISTANCE/(double)discr_level;   // Distance per discretization level
double angle_per_level = ((double)48.6/(double)discr_level) * (M_PI/180); // Angle (rad) per discretization level

mat44 T05;       // Direct kinematic model
mat44 TSB;       // Transformation matrix from sensor to arm base
mat33 I2P;       // Tranformation from Image frame to P2

ros::master::V_TopicInfo topic_info;
bool is_simulation;
bool update_pose = false;
int counter_continuous_low_distance = 0;   // Counter of continuous times the distance is really low

bool gripper_opened;    // Is the gripper opened?
vec3 gripper_position;
double gripper_effort;

bool seeing_table = false;
bool object_reachable = false;
vec2 object_center;    // Object center in pixels

vec5 joint_angles;

int action;                 // Learning action
int num_states = (discr_level+2) * pow(discr_level+1, 2) * pow(2,2); // Really 13 * 12² * 2² = 7488 
arma::mat q_matrix = arma::zeros<arma::mat>(num_states, N_ACTIONS);        // Q matrix
vec prev_V = arma::zeros<vec>(num_states);             // Prev value function
vec V = arma::zeros<vec>(num_states);                  // Value function
vec policy_matrix = arma::zeros<vec>(num_states);      // Policy matrix
arma::mat visit_matrix = arma::zeros<arma::mat>(num_states, N_ACTIONS);    // Matrix of visits

double alpha;
int steps = 0;
int simulations = 0;
bool gui = false;

// Logs
ofstream log_file;              // Log file
string log_name;
stringstream complete_log_name;
stringstream complete_simplified_log_name;
stringstream complete_exploitation_log_name;
stringstream complete_distance_log_name;
stringstream complete_iteration_distance_log_name;
stringstream complete_position_log_name;
stringstream complete_object_position_log_name;


// Elements useful for object detection
cv_bridge::CvImagePtr cv_ptr; // Pointer to the cv image
cv::Mat pixel_locations;          // Location of the pixels of the object
arma::mat P = arma::zeros<arma::mat>(3,4);               // Projection/camera matrix of 3 x 4

// To get object position
int sum_x = 0; 
int sum_y = 0; 
double max_u = INFINITY; 
double max_v = INFINITY;
double min_u = -INFINITY;
double min_v = -INFINITY;
set<int> x_values = set<int>();
set<int> y_values = set<int>();

// Node handlers
class Handlers{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
public:
    Handlers(): it(nh){}
    ~Handlers(){}
    ros::NodeHandle getNH(){
        return nh;
    }
    image_transport::ImageTransport getIT(){
        return it;
    }
};

// Subscribers
image_transport::Subscriber color_image_sub;
ros::Subscriber camera_info_sub;
ros::Subscriber joint_states_sub;
ros::Subscriber discr_level_sub;
ros::Subscriber sim_pose_sub;

// Publishers
ros::Publisher joints[5];
ros::Publisher gripper;
ros::Publisher base;

/*------------------------------------
 Callbacks
 -----------------------------------*/
 /*------------------------------------
 Get the rgb image and process it:
 -----------------------------------*/
 void callbackImage(const ImageConstPtr& image_msg);
 /*------------------------------------
 Get the rgb camera info and get the projection matrix:
 -----------------------------------*/
 void callbackCameraInfo(const CameraInfoConstPtr& camera_info_msg);
/*------------------------------------
 Get gripper effort:
 -----------------------------------*/
 void getGripperEffortCallback(const JointStateConstPtr& joint_states_msg);
 /*------------------------------------
 Get simulation pose and actualize it:
 -----------------------------------*/
 void getSimulationPoseCallback(const Vector3ConstPtr& sim_pose_msg);
/*------------------------------------
 Methods
 -----------------------------------*/
int main(int argc, char** argv);
/*------------------------------------
 Initialize vectors and matrices
 -----------------------------------*/
 void initializeVecMat();
/*------------------------------------
 Initialize learning elements reading from a log file
 -----------------------------------*/
 void readLog();
/*------------------------------------
 Initialize transformation matrix from Sensor frame to Widow-X arm base frame:
 -----------------------------------*/
 void initializeTSB();
 /*------------------------------------
 Initialize transformation matrix from image frame to P2:
 -----------------------------------*/
 void initializeI2P();
/*------------------------------------
 Process messages:
 -----------------------------------*/
 void processMessages();
/*------------------------------------
 Make all the process of learning:
 -----------------------------------*/
void learning(Handlers handlers);
/*------------------------------------
 Update state:
-----------------------------------*/
void updateState();
/*------------------------------------
 Get the location in pixels of the object:
-----------------------------------*/
void getLocation();
/*------------------------------------
 Calculate the real position of the object with respect to the robot:
-----------------------------------*/
void calculateRealPos();
/*------------------------------------
Get object real position with respect to the sensor frame where:
    max_u: Max X coordinate of the center of the object (Pixels)
    max_v: Max Y coordinate of the center of the object (Pixels)
    min_u: Min X coordinate of the center of the object (Pixels)
    min_v: Min Y coordinate of the center of the object (Pixels)
-----------------------------------*/
void getObjectPosition();
/*------------------------------------
 Discretize values:
-----------------------------------*/
void discretizeValues();
/*------------------------------------
 Discretize values auxiliar:
    Inputs:
        selector:
            0: angle
            1: height
            2: depth
        step: difference between each level of discretization
-----------------------------------*/
void discretizeValuesAux(int selector, double step);
/*------------------------------------
 Check if we are executing a gazebo simulation:
-----------------------------------*/
void isSimulation();
/*------------------------------------
 Get the direct kinematic model of the widowX-arm
-----------------------------------*/
void mcd();
/*------------------------------------
 Get the position of the gripper by means of the Direct kinematic model:
 -----------------------------------*/
void getGripperPosition();
/*------------------------------------
 Get the angle of each joint in order to reach the desired position
 by means of the inverse kinematic model:
    Inputs:
        next_position: Desired position 
-----------------------------------*/
void mci(vec3 next_position);
/*------------------------------------
 Move Widow-X arm to object:
 -----------------------------------*/
void moveArmToObject();
/*------------------------------------
 Open gripper:
 -----------------------------------*/
void openGripper();
/*------------------------------------
 Close gripper:
 -----------------------------------*/
 void closeGripper();
/*------------------------------------
 Detect picked object:
-----------------------------------*/
void isObjectPicked();
/*------------------------------------
Transform a point in the image to the sensor frame
-----------------------------------*/
vec4 image2sensor(vec3 pixel_pos);
/*------------------------------------
 Detect if object is reachable:
    Object is centered in x and near
-----------------------------------*/
void isObjectReachable();
/*------------------------------------
 Set next position:
   Inputs:
      x, y, z: The coordinates of the next position
-----------------------------------*/
vec3 setNextPosition(double x, double y, double z);
/*------------------------------------
 Fold arm:
-----------------------------------*/
void foldArm();
/*------------------------------------
 Start a new random simulation:
-----------------------------------*/
void startRandomSimulation();
/*------------------------------------
 Kill the current simulation:
-----------------------------------*/
void killSimulation();
/*------------------------------------
 Get the matrix row index from the robot state:
-----------------------------------*/
int getIndexFromState();
/*------------------------------------
 Modify the state of the robot from the row index:
-----------------------------------*/
void getStateFromIndex(int index);
/*------------------------------------
 Select action (Exploitation/exploration strategy):
-----------------------------------*/
void selectAction();
/*------------------------------------
 Update V and policy matrix:
-----------------------------------*/
void updateVPolicy(int s);
/*------------------------------------
 Give reward?:
-----------------------------------*/
bool giveReward();
/*------------------------------------
 Calculate reward:
-----------------------------------*/
void calculateReward();
/*------------------------------------
 Actualize log:
-----------------------------------*/
void actualizeLog();
/*------------------------------------
 Actualize simplified log:
-----------------------------------*/
void actualizeSimplifiedLog();
/*------------------------------------
 Actualize log for exploitation:
-----------------------------------*/
void actualizeExploitationLog();
/*------------------------------------
 Actualize log for distances:
-----------------------------------*/
void actualizedistanceLog();
/*------------------------------------
 Actualize log for distances each iteration:
-----------------------------------*/
void actualizeIterationDistanceLog();
/*------------------------------------
 Actualize log for position of robot each iteration:
    Inputs:
        x, y, z: Position of the robot
-----------------------------------*/
void actualizePositionLog(double x, double y, double z);
/*------------------------------------
 Actualize log for position of object on each simulation:
    Inputs:
        x, y, z: Position of the object
-----------------------------------*/
void actualizeObjectPositionLog(double x, double y, double z);
/*------------------------------------
 Print debug:
    Inputs:
        function: Function where you are
        line: Line in which you are
-----------------------------------*/
void printDebug(string function, int line);
/*------------------------------------
 Get random number following an uniform distribution:
    Inputs:
        min, max: Min and max values
-----------------------------------*/
double unifRnd(double min, double max);
