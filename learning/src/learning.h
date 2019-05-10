/*------------------------------------
 Libraries
-----------------------------------*/
#include <fstream>
#include <math.h>
// Ros library
#include <ros/ros.h>
// Image processing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Image filters
#include <opencv2/imgproc/imgproc.hpp>
// Image visualization
#include <opencv2/highgui/highgui.hpp>
// Ros messages
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

#define MAX_DISTANCE 2.0   //Max distance to perceive the object

#define OBJECT_HEIGHT 0.080  //Medir
#define OBJECT_WIDTH 0.028 //Medir
#define OBJECT_AREA (OBJECT_HEIGHT * OBJECT_WEIGHT)

#define SENSOR_HEIGHT 0.00230 // Height of the sensor in meters
#define SENSOR_WIDTH 0.00173 // Width of the sensor in pixels

#define CAMERA_HEIGHT 640 // Height of the image in pixels
#define CAMERA_WIDTH 480 // Width of the image in pixels

#define HEIGHT_PX_2_M (SENSOR_HEIGHT/CAMERA_HEIGHT)
#define WIDTH_PX_2_M (SENSOR_WIDTH/CAMERA_WIDTH)

// Min and max positions of the objects
#define MAX_X 2.5
#define MIN_X 2.0
#define MAX_Y 2.0
#define MIN_Y -2.0
// Number of box.urdf to use (box_1 z-size=0.1, box_2 z-size=0.2...)
#define MIN_BOX 4
#define MAX_BOX 6

#define ALPHA 0.02
#define GAMMA 0.9

#define N_ACTIONS 5 // Number of actions

#define MAX_EXPLORATION 100
#define MIN_EXPLORATION 30
#define DECAY 1e-3

/*------------------------------------
 Some useful namespaces:
-----------------------------------*/
using namespace std;
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

bool inside_learning = false; // Boolean to control the callbacks

double a[3];            // Orientation of gripper
int discr_level = 5;    // Discretization level

double T05[4][4];       // Direct kinematic model

ros::master::V_TopicInfo topic_info;
bool is_simulation;

bool gripper_opened;    // Is the gripper opened?
double gripper_position[3];
double gripper_effort;

bool seeing_table = false;
bool object_reachable = false;
double object_center[2];    // Object center in pixels

double joint_angles[5];

int action;                 // Learning action
//const size_t num_states = pow(discr_level+1, 3) * pow(2,2); // Really 6³*2² = 864 
double q_matrix[864][5];        // Q matrix
double V[864];                  // Value function
int policy_matrix[864];      // Policy matrix
int visit_matrix[864][5];    // Matrix of visits
int counter = 0;            // Total number of steps

double exploration_rate = MAX_EXPLORATION;
int steps = 0;
int simulations = 0;
bool gui = false;
ofstream log_file;              // Log file

// Elements useful for object detection
cv_bridge::CvImagePtr cv_ptr; // Pointer to the cv image
Mat pixel_locations;          // Location of the pixels of the object
double P[3][4];               // Projection/camera matrix of 3 x 4
double P_inv[4][3]; // Pseudo-inverse Projection matrix of 4 x 3

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
 Change discretization level:
 -----------------------------------*/
 void setDiscretizationLevel(const Int16ConstPtr& new_discr_level);
/*------------------------------------
 Methods
 -----------------------------------*/
int main(int argc, char** argv);
/*------------------------------------
 Process messages:
 -----------------------------------*/
 void processMessages();
/*------------------------------------
 Make all the process of learning:
    While node is running:
        1. Get state
        2. Detect if it is reachable
        3. Move arm if correct, else move base
        4. Fold arm
        5. Check reward
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
 Discretize values:
-----------------------------------*/
void discretizeValues();
/*------------------------------------
 Discretize values auxiliar:
    Inputs:
        - selector:
            0: angle
            1: height
            2: depth
        - step
-----------------------------------*/
void discretizeValuesAux(int selector, double step);
/*------------------------------------
 Check if we are executing a gazebo simulation:
-----------------------------------*/
void isSimulation();
/*------------------------------------
 Multiply P_inv and pixel_pos
-----------------------------------*/
void multiplyP_Inv(double result[4][1], double P_inv[4][3], double pixel_pos[3][1]);
/*------------------------------------
 Multiply 2 transformation matrices
-----------------------------------*/
void multiplyTransformations(double result[4][4], double first[4][4], double second[4][4]);
/*------------------------------------
 Update the matrix T05 of the direct kinematic model copying aux into T05
-----------------------------------*/
void updateT05(double T05[4][4], double aux[4][4]);
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
        - next_position: Desired position
        - a: Desired angle orientation of the wrisp
        - n: Desired orientation of the wrisp 
 -----------------------------------*/
void mci(double next_position[3], double n[3]);
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
 Get object real position with respect to the robot:
    [X Y Z 1] = P^(-1) * [u v 1]
    Inputs:
        top_u: X coordinate of the top center of the object (Pixels)
        top_v: Y coordinate of the top center of the object (Pixels)
        bottom_u: X coordinate of the bottom center of the object (Pixels)
        bottom_v: Y coordinate of the bottom center of the object (Pixels)
-----------------------------------*/
void getObjectPosition(int top_u, int top_v, int bottom_u, int bottom_v);
/*------------------------------------
 Detect if object is reachable:
    Object is centered in x, near and up
-----------------------------------*/
void isObjectReachable();
/*------------------------------------
 Set next position:
   Inputs:
      next_position: The next position to initialize
      x, y, z: The coordinates of the next position
-----------------------------------*/
void setNextPosition(double next_position[3], double x, double y, double z);
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
    Inputs:
        - sa: actual state
-----------------------------------*/
void selectAction(int sa);
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
double calculateReward(int sa, int sp);
/*------------------------------------
 Actualize log:
-----------------------------------*/
void actualizeLog(int sa, int sp, double reward);
/*------------------------------------
 Actualize simplified log:
-----------------------------------*/
void actualizeSimplifiedLog(int sa, int sp, double reward);
/*------------------------------------
 Print debug:
-----------------------------------*/
void printDebug(string function, int line);
/*------------------------------------
 Get uniform random:
    Inputs:
        - min
        - max
-----------------------------------*/
double unifRnd(double min, double max);
