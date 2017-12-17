#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Range.h"
#include <warehouse_quad/markerInfo.h>
#include <warehouse_quad/line.h>
float z;
bool _detected=false;
warehouse_quad::line line;
using namespace std;

void linecb(const warehouse_quad::line::ConstPtr& msg)
{
  line.mode = msg->mode;
}
void posecb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  z = msg->pose.position.z;
}
void markercb(const warehouse_quad::markerInfo::ConstPtr& msg)
{
  _detected = msg->detect;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mavros/vision_pose/pose", 10, posecb);
  ros::Subscriber sub_marker = n.subscribe("/marker", 10, markercb);
  ros::Subscriber sub_line = n.subscribe("/line",10, linecb);
  ros::Publisher setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Publisher distance_pub = n.advertise<sensor_msgs::Range>("/mavros/distance_sensor/sonar_1_sub", 10);
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
    switch (_flag) {

      case 1:
        setpoint.pose.position.z = 1.0f;
        distance.radiation_type = 1;
        if (fabsf(z-1.0f)<=0.2f)
        {
          _flag = 2;

        }
        break;

      case 2:
  	    setpoint.pose.position.z = 1.0f;
        distance.radiation_type = 0;
	//	cout << ros::Time::now().toSec()<<endl;
		cout << _detected << endl;
        if (0)
        {
          _flag = 3;
        }
	break;
      case 3:
        setpoint.pose.position.z = 0.0f;
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
