#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Range.h"
#include <warehouse_quad/markerInfo.h>
#include <warehouse_quad/line.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>

float z, c1, c2;
bool _detected=false;
warehouse_quad::line line;
double yaw, yaw_set, yaw_init;

using namespace std;

void linecb(const warehouse_quad::line::ConstPtr& msg)
{
  line.mode = msg->mode;
  yaw_set = msg->slope + yaw;
 cout << msg->slope*180/3.14 << "\t" << msg->mode << endl;
	
}
void posecb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  z = msg->pose.position.z;
}
void markercb(const warehouse_quad::markerInfo::ConstPtr& msg)
{
  _detected = msg->detect;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	tf::Quaternion q(0, 0, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	double r, p;
	m.getRPY(r,p,yaw);

}

void statecb(const mavros_msgs::State::ConstPtr& msg){
	if(msg->mode != "OFFBOARD"){
		yaw_init = yaw;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mavros/vision_pose/pose", 10, posecb);
  ros::Subscriber sub_marker = n.subscribe("/marker", 10, markercb);
  ros::Subscriber sub_line = n.subscribe("/line",10, linecb);
  ros::Publisher setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Subscriber sub1 = n.subscribe("/mavros/imu/data",100, imuCallback);
  ros::Publisher distance_pub = n.advertise<sensor_msgs::Range>("/mavros/distance_sensor/sonar_1_sub", 10);
  ros::Subscriber sub2 = n.subscribe("/mavros/state",10, statecb);
  ros::Rate loop_rate(100);
// secs1 = ros::Time::now().toSec();
  int _flag=1;
  while (ros::ok())
  {
    sensor_msgs::Range distance;
    geometry_msgs::PoseStamped setpoint;

    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "quad";
    distance.header.stamp = ros::Time::now();


    setpoint.pose.position.x = 0.0f;
    setpoint.pose.position.y = 0.0f;
    tf::Quaternion q;
    tf::Quaternion q1;

    switch (_flag) {

      case 1:  //takeoff
        setpoint.pose.position.z = 1.0f;
        distance.radiation_type = 1;
	distance.min_range = 0.2;
        distance.max_range = 5;
	distance.range = 3.0;
        distance.field_of_view = 10.0;
	
	q.setRPY(0, 0, yaw_init);
	setpoint.pose.orientation.z = q.z();
	setpoint.pose.orientation.w = q.w();
	 if (fabsf(z-1.0f)<=0.2f)
        {
          _flag = 2;

        }
        break;

      case 2:  //forward
  	setpoint.pose.position.z = 1.0f;
        distance.radiation_type = 0;
	//	cout << ros::Time::now().toSec()<<endl;
//	cout << _detected << endl;
	q1.setRPY(0, 0, yaw_set);
	setpoint.pose.orientation.z = q1.z();
	setpoint.pose.orientation.w = q1.w();
	if (line.mode == 0)
        {
          _flag = 3;
        }
	break;
      case 3:
        setpoint.pose.position.z = 0.0f;
	q.setRPY(0, 0, yaw_init);
	setpoint.pose.orientation.z = q.z();
	setpoint.pose.orientation.w = q.w();
	
        break;

      default:
        //setpoint.pose.position.z = 0.0f;
      break;
    }

    //cout<< ros::Time::now().toSec()-secs1-secs<<endl;
    distance_pub.publish(distance);
    setpoint_pub.publish(setpoint);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
