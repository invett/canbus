#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "BUSCAN.hpp"
#include "canbus/can_msg.h"
#include "canbus/can_raw.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "canbus_node");
  ros::NodeHandle n;

  ros::Publisher can_pub = n.advertise<canbus::can_msg>("/canbus/data", 1);
  ros::Publisher can_raw_pub = n.advertise<canbus::can_raw>("/canbus/raw", 1);

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

      canbus::can_raw raw_msg;
      canbus::can_msg msg;

      while (ros::ok())
      {

        sem_wait(&can_sem); // new can raw message
        
        raw_msg.id = busCAN.get_raw_msg_id();
        busCAN.get_raw_msg(&(raw_msg.raw[0]));
        can_raw_pub.publish(raw_msg);

        // Hbaria que crear otro nodo que se subscriba a los mensajes en raw y los parsee
        // mientras tanto los parseamos aqui...

        if(raw_msg.id == 781) // new speed message, pack info and share it
        {
          msg.speed = busCAN.getSpeedMPS_Unblocking(&p);
          msg.steer = busCAN.getSteeringWheelPosition_Unblocking(&s);
          msg.brake = busCAN.getBrakeForce_Unblocking(&b);
          msg.throttle = busCAN.getThrottle_Unblocking();
          msg.gear = busCAN.getGear_Unblocking();

          if((estado_marchas)msg.gear == estado_marchas::R)
              msg.speed = -msg.speed;

          can_pub.publish(msg);
          ROS_DEBUG("%lf %lf %lf %lf", msg.speed, msg.steer, msg.brake, msg.gear);
        }

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