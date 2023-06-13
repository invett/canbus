#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "BUSCAN.hpp"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("speed", 1000);

  ros::Rate loop_rate(10);

  sem_t can_sem;
  sem_init(&can_sem, 0, 0);
  
  BUSCAN busCAN(&can_sem);

  bool init_buscan = busCAN.init_BUSCAN(BAUD_500K); //NO COFIGURABLE

    if(init_buscan)
    {
        std::cerr << "\033[1;32m"  << "BUSCAN inicializado " << "\033[0m" << std::endl;
    }
    else
    {
        std::cerr << "\033[1;31m" << "No se pudo inicializar el BUSCAN " << "\033[0m" <<std::endl;
        ros::shutdown();
    }

  double speed = 0;
  C4speed_t *p;

  int count = 0;
  while (ros::ok())
  {

    sem_wait(&can_sem);
    std_msgs::Float64 msg;
    msg.data = busCAN.getSpeedMPS_Unblocking(&p);

    ROS_INFO("%lf", msg.data);
    chatter_pub.publish(msg);

    ros::spinOnce();

    //loop_rate.sleep();
    ++count;
  }


  return 0;
}





// //-- BUSCAN --
//     m_busCAN_initialized = false;
//     //--------------------------------------------------------------------------------------------------------------
//     //---------- BUSCAN Initialization -----------------------------------------------------------------------------
//     {
//         m_busCAN = new BUSCAN(HLC_sem);
//         m_busCAN_initialized = m_busCAN->init_BUSCAN(BAUD_500K); //NO COFIGURABLE

//         if(m_busCAN_initialized)
//         {
//             std::cerr << "\033[1;32m"  << "BUSCAN inicializado " << "\033[0m" << std::endl;
//         }
//         else
//         {
//             std::cerr << "\033[1;31m" << "No se pudo inicializar el BUSCAN " << "\033[0m" <<std::endl;
//         }
//     }