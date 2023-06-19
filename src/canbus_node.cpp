#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "BUSCAN.hpp"
#include "canbus/can_msg.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "canbus_node");
  ros::NodeHandle n;

  ros::Publisher can_pub = n.advertise<canbus::can_msg>("/canbus/data", 1);

  sem_t can_sem;
  sem_init(&can_sem, 0, 0);
  
  BUSCAN busCAN(&can_sem);

  bool init_buscan = busCAN.init_BUSCAN(BAUD_500K); //NO COFIGURABLE

    if(init_buscan)
    {
      ROS_INFO("BUSCAN inicializado");

      C4speed_t *p;
      C4steer_t *s;
      C4brake_t *b;

      while (ros::ok())
      {

        sem_wait(&can_sem);

        canbus::can_msg msg;
        msg.speed = busCAN.getSpeedMPS_Unblocking(&p);
        msg.steer = busCAN.getSteeringWheelPosition_Unblocking(&s);
        msg.brake = busCAN.getBrakeForce_Unblocking(&b);

        ROS_DEBUG("%lf %lf %lf", msg.speed, msg.steer, msg.brake);
        //speed_pub.publish(msg);
        can_pub.publish(msg);

        ros::spinOnce();
      }
      return 1;
    }
    else
    {
      ROS_ERROR("No se pudo inicializar el BUSCAN ");
      return 0;
    }

}
