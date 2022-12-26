/******************************************************************

###motor params:
speed_ratio:          (unit: m/encode)    
wheel_distance:       (unit: m)
encode_sampling_time: (unit: s)
*******************************************************************/
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// opencv
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//string
#include <string>
//cos,sin
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define NUM_OF_LASER_POINT 759


#define MAP_Width     2048
#define MAP_Height    2048
#define NO_LINE       40
int image_save_flag=1;


using namespace cv;
using namespace std;

Mat mat_map_org_gray   = Mat::zeros(MAP_Height,MAP_Width,CV_8UC1);
Mat mat_map_line_color = Mat::zeros(MAP_Height,MAP_Width,CV_8UC3);
Mat mat_image_canny_edge;

float range_data[NUM_OF_LASER_POINT] = {0,};
float roll_d,pitch_d, yaw_d;
double roll,pitch, yaw;
float imu_yaw;
bool yaw_topic_received_flag =  0;

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));	
   Canny(mat_blur_img,mat_canny_img, 70,150,3);
	
   return mat_canny_img;	
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float  c[NO_LINE] = {0.0, };
    float  d[NO_LINE] = {0.0, };
    
    int count = (int)( 360. / RAD2DEG(scan->angle_increment));
    int sum=0; 
    float x=0, y=0;
    int img_x=0,img_y=0;
    int i = 0;
    
    memset(range_data, 0, sizeof(float)*NUM_OF_LASER_POINT);
    mat_map_org_gray = Mat::zeros(MAP_Height,MAP_Width,CV_8UC1);
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("%f %f",scan->scan_time , scan->time_increment);
    //ROS_INFO("angle_range, %f, %f %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));
  
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        x = scan->ranges[i]*cos(scan->angle_min + scan->angle_increment * i);
        y = scan->ranges[i]*sin(scan->angle_min + scan->angle_increment * i);
       // ROS_INFO(": [%d %f, %f]", i, x, y);
        range_data[i] = scan->ranges[i];
        //ROS_INFO(": [%d %f, %f]", i, degree, scan->ranges[i]);
       
        img_y = MAP_Width  /2  - int(x*100+0.5);  
        img_x = MAP_Height /2  - int(y*100+0.5);
        
        if( (img_x<MAP_Width)&& (img_x>=0) && (img_y<MAP_Height)&& (img_y>=0))
        {
          //printf("%4d %4d \n",img_x,img_y);
          
          mat_map_org_gray.at<uchar>(img_y,img_x) = 255;
        }
    }
    
     
     vector<Vec4i> linesP;
	  
     
      
       mat_image_canny_edge = Canny_Edge_Detection(mat_map_org_gray);
       HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI / 180, 10, 200, 30);
       printf("lines number %d\n ",(int)linesP.size());
       for(int i=0; i<linesP.size();i++)
       {
		  if(i>=NO_LINE) break;
		  
		  Vec4i L= linesP[i];
		  
		  if(fabs(L[3]-L[1])>1.0e-7)
		      c[i] =  (float)(L[2]-L[0])/(float)(L[3]-L[1]);
          else 
              c[i] = 1.0e7;
              
		  printf("%3d %3d %3d %3d %6.3lf \n", L[0],L[1],L[2],L[3],RAD2DEG(atan(c[i])) );
		  line(mat_map_line_color,Point(L[0],L[1]),Point(L[2],L[3]), Scalar(0,255,0),2, LINE_AA);
		}
	 if(image_save_flag==1)
     {
    
	   imwrite("/home/amap/vm_catkin_ws/src/lidar_localization/lidar.bmp",mat_map_org_gray);
       imwrite("/home/amap/vm_catkin_ws/src/lidar_localization/lidar_line.bmp",mat_map_line_color);
      
       
       image_save_flag =0;
       
     }
     
    //ROS_INFO("count= %d", count);
     
}
    
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
	
  /*
   *   ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    */        
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 m(q);     
            
      m.getRPY(roll, pitch, yaw);
      imu_yaw = yaw;
      roll_d  = RAD2DEG(roll);
      pitch_d = RAD2DEG(pitch);
      yaw_d   = RAD2DEG(yaw);        
      yaw_topic_received_flag = 1;      
}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "laser_scan_localilzatoin");
  ros::NodeHandle n;

  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "base_footprint";
   std::string imu_topic = "imu";
  ros::param::get("~imu_topic", imu_topic);    
  
  ros::Subscriber sub_laser = n.subscribe("/scan", 20, scanCallback);
  //ros::Subscriber sub_rpy_angle = n.subscribe("/rpy_degree", 20, callback2);
  ros::Subscriber subIMU = n.subscribe(imu_topic, 20, &imuCallback);  // imu
  
  
  ros::Rate loop_rate(1.0); //10.0HZ
  
  ////////////////  image display window ///////////////////////////
   
  namedWindow("view", WINDOW_NORMAL);
  resizeWindow("view", MAP_Width/4,MAP_Height/4);
  moveWindow("view", 10, 10);
  
  namedWindow("lidar", WINDOW_NORMAL);
  resizeWindow("lidar", MAP_Width/4,MAP_Height/4);
  moveWindow("lidar", 500, 10);
   
  while(ros::ok())
  {
	  if(yaw_topic_received_flag==1)       ROS_INFO("Imu Yaw : %6.3lf", yaw_d);
	  cv::imshow("lidar", mat_map_org_gray);
      cv::imshow("view", mat_map_line_color);
      cv::waitKey(30);
       
      ros::spinOnce();      
      loop_rate.sleep();
  }
  cv::destroyWindow("view");
  return 0;
}
