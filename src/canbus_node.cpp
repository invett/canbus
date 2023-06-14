#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "BUSCAN.hpp"
#include "canbus/can_msg.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "canbus_driver");
  ros::NodeHandle n;

  ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("speed", 1);
  ros::Publisher can_pub = n.advertise<canbus::can_msg>("canbus", 1);

  sem_t can_sem;
  sem_init(&can_sem, 0, 0);
  
  BUSCAN busCAN(&can_sem);

  bool init_buscan = busCAN.init_BUSCAN(BAUD_500K); //NO COFIGURABLE

    if(init_buscan)
    {
      ROS_INFO("BUSCAN inicializado");

      C4speed_t *p;

      while (ros::ok())
      {

        sem_wait(&can_sem);
        std_msgs::Float64 msg;
        msg.data = busCAN.getSpeedMPS_Unblocking(&p);

        ROS_DEBUG("%lf", msg.data);
        speed_pub.publish(msg);

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
