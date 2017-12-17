#include "ros/ros.h"
#include "std_msgs/String.h"
#include  "geometry_msgs/PoseStamped.h"
#include <warehouse_quad/markerInfo.h>
float z;
bool _detected=false;

using namespace std;

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
  ros::Publisher setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Rate loop_rate(100);
// secs1 = ros::Time::now().toSec();
  int _flag=1;
  while (ros::ok())
  {

    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "quad";
    setpoint.pose.position.x = 0.0f;
    setpoint.pose.position.y = 0.0f;
    switch (_flag) {

      case 1:
        setpoint.pose.position.z = 1.0f;
        if (fabsf(z-1.0f)<=0.2f)
        {
          _flag = 2;
	   
        }
        break;

      case 2:
  	    setpoint.pose.position.z = 1.0f;
	//	cout << ros::Time::now().toSec()<<endl;
		cout << _detected << endl;
        if (_detected)
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

    setpoint_pub.publish(setpoint);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
