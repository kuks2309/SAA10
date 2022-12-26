#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>




// maximum angular velocity 
#define MAX_L_STEER -6.0
#define MAX_R_STEER  6.0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 6
#define WayPoint_X_Tor 0.1
#define WayPoint_Y_Tor 0.3

#define SpeedRegion_NO 100

geometry_msgs::Pose unicar_model_pose;

double pos_x = 0.0;
double pos_y = 0.0;
double roll,pitch,yaw;

double r,p,y;

float   car_speed_base = 2.0; //[m/s]
float   car_speed[SpeedRegion_NO] = {car_speed_base,};  // initialize speed


int    no_region = 4;
int    roi_no = -1;

struct Point 
{ 
	float x; 
	float y; 
	float z;
};


struct WayPoints
{
	float x;
	float y;
	
} ;

struct Current_Pos
{
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0 ;
} my_pose;

struct region
{
	float left;
	float right;
	float top;
	float bottom;
} speed_region[SpeedRegion_NO];

void odomCallback(const nav_msgs::Odometry& msg)
{
	
	my_pose.x = msg.pose.pose.position.x;
	my_pose.y = msg.pose.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
      
    m.getRPY(r, p, y);
    my_pose.theta = y;  // radian
	
	//ROS_INFO("odom : %.2f %.2f %.2f", msg.pose.pose.position.x, msg.pose.pose.position.y, my_pose.theta); 
	
}

void pose2DCallback(const geometry_msgs::Pose2D& pose)
{
	
	my_pose.x = pose.x;
	my_pose.y = pose.y;
    my_pose.theta = pose.theta;
	
	ROS_INFO("pose : %.2lf %.2lf %.2lf", my_pose.x, my_pose.y, my_pose.theta); 
}


void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	my_pose.x = (double)msg.pose.position.x;
	my_pose.y = (double)msg.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;		
}

void read_speed_data(void)
{
	FILE *fp;
	int result = 0;
	int i = 0;
		
	fp = fopen("//home//amap//race_data//vision_data/roi_speed_data.txt","r");
	if(fp == NULL) 
	{
		ROS_INFO("Speed Data File does not exit ~~ ");
		ROS_INFO("Speed is set to 120[pwm] ~~ ");
		
		car_speed_base = 120;
	}

	result = fscanf(fp,"%f", &car_speed_base);
	result = fscanf(fp,"%d", &no_region);
	
	for(i=0; i< no_region ; i++)
	{
		result = fscanf(fp,"%f", &speed_region[i].left);
		result = fscanf(fp,"%f", &speed_region[i].top);
		result = fscanf(fp,"%f", &speed_region[i].right);
		result = fscanf(fp,"%f", &speed_region[i].bottom);
		result = fscanf(fp,"%f", &car_speed[i]);
		
		printf("[%2d] : %3.1f %3.1f %3.1f %3.1f %3.1f \n", i, speed_region[i].left, speed_region[i].top, speed_region[i].right , speed_region[i].bottom,car_speed[i] );
	}
	
	fclose(fp);
	
}

void read_abs_roi_speed_data(void)
{
	FILE *fp;
	int result = 0;
	int i = 0;
		
	fp = fopen("//home//amap//race_data//vision_data//abs_roi_speed_data.txt","r");
	if(fp == NULL) 
	{
		ROS_INFO("Absolute Speed Data File does not exit ~~ ");
		ROS_INFO("Absolute  Speed is set to  1.1m/s ~~ ");
		
		car_speed_base = 1.1;
	}

	result = fscanf(fp,"%f", &car_speed_base);
	result = fscanf(fp,"%d", &no_region);
	
	for(i=0; i< no_region ; i++)
	{
		result = fscanf(fp,"%f", &speed_region[i].left);
		result = fscanf(fp,"%f", &speed_region[i].top);
		result = fscanf(fp,"%f", &speed_region[i].right);
		result = fscanf(fp,"%f", &speed_region[i].bottom);
		result = fscanf(fp,"%f", &car_speed[i]);
		
		printf("[%2d] : %3.1f %3.1f %3.1f %3.1f %3.1f \n", i, speed_region[i].left, speed_region[i].top, speed_region[i].right , speed_region[i].bottom,car_speed[i] );
	}
	
	fclose(fp);
}

float speed_check(void)
{
	float min_y, max_y, min_x, max_x;
	float speed; 
	int i;
    roi_no = -1; 
    /*
    min_y = -3.7;     max_y = -3.4;     min_x =  0.5;     max_x =  0.6; 
           
    if( (my_pose.x >= min_x) && (my_pose.x <= max_x)  && (my_pose.y >= min_y) && (my_pose.y  <= max_y ) )
	{
       speed = 0;
       return speed;
    }	
    else 
    {	
      */
	 for(i=0; i< no_region ; i++)
	 {
		
		min_x = (speed_region[i].bottom >= speed_region[i].top) ? speed_region[i].top : speed_region[i].bottom;
        max_x = (speed_region[i].bottom >= speed_region[i].top) ? speed_region[i].bottom : speed_region[i].top;
        min_y= (speed_region[i].left >= speed_region[i].right) ? speed_region[i].right : speed_region[i].left;
        max_y = (speed_region[i].left >= speed_region[i].right) ? speed_region[i].left : speed_region[i].right;
        printf("%3.1f %3.1f %3.1f %3.1f\n", min_x,max_x, min_y,max_y);
		if( (my_pose.x >= min_x) && (my_pose.x <= max_x)  && (my_pose.y >= min_y) && (my_pose.y  <= max_y ) )
		{
		  roi_no = i;	
		  speed = car_speed[i];
		  return speed;
		}		 
		else 
		{
			
			speed = car_speed_base;		
		}
	 //}	 
   }	
     //printf("car_speed %3.1f\n",speed);
	 return speed;	
}
int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}


void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    int unicar_model_index = getIndex(model_states.name, "uni_car");
    unicar_model_pose = model_states.pose[unicar_model_index];
    
    ROS_INFO("%.2f %.2f",unicar_model_pose.position.x,unicar_model_pose.position.y);
    
    my_pose.x = unicar_model_pose.position.x;
    my_pose.y = unicar_model_pose.position.y;
}



int main(int argc, char **argv)
{
  int count = 0;
    
  ros::init(argc, argv, "robot_speed_control_roi_navigation");

  ros::NodeHandle n;

 //std::string odom_sub_topic = "/odom"; // receive odom or pose
  std::string odom_sub_topic = "/ackermann_steering_controller/odom"; // receive odom or pose
  std::string pose2D_sub_topic = "/pose2d"; // receive odom
  std::string speed_topic = "/Car_Control_cmd/Speed_Float32";
  std::string roi_no_topic = "roi_no";
  std::string pose_hector_sub_topic = "/slam_out_pose" ; // receive odom from hector slam
  
 // ros::Subscriber model_states_subscriber = n.subscribe("/gazebo/model_states", 100, model_states_callback);
   
    ros::Subscriber sub1         = n.subscribe(odom_sub_topic, 10, &odomCallback);
 // ros::Subscriber sub2         = n.subscribe(pose2D_sub_topic, 10, &pose2DCallback);
 // ros::Subscriber sub3         = n.subscribe(pose_hector_sub_topic,10, &poseCallback);              // from hector slam
  ros::Publisher roi_no_pub    = n.advertise<std_msgs::Int8>(roi_no_topic,10);
  ros::Publisher car_speed_pub = n.advertise<std_msgs::Float32>(speed_topic,10);
 
  read_speed_data();
  //read_abs_roi_speed_data();
  
  //return -1;  
 
  ros::Rate loop_rate(30);  // 10
  
  std_msgs::Int8 speed_roi_no;  
  speed_roi_no.data=-1;
  std_msgs::Float32 car_speed_current;
  car_speed_current.data = car_speed_base;  
  
   
  while (ros::ok())
  {
	car_speed_current.data = speed_check();
	//printf("car_speed_current.data :  %6.3lf  %6.3lf \n",car_speed_current.data, speed_check());	   
    //speed_roi_no.data = roi_no;
    //roi_no_pub.publish(speed_roi_no);   
    car_speed_pub.publish(car_speed_current);
	
	ROS_INFO("ROI : %3d X : %6.3lf  Y : %6.3lf  A : %6.3lf Speed : %3.1f ",roi_no, my_pose.x, my_pose.y, my_pose.theta, car_speed_current.data);
	//printf("loop test %d \n",count);
	
	loop_rate.sleep();
    ros::spinOnce();
    
    ++count;
  }
  return 0;
}
