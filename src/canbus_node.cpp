#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "BUSCAN.hpp"
#include "canbus/can_raw.h"


typedef struct
{
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
}raw_can_data_t;


typedef struct{
  pthread_t task;
  pthread_mutex_t acces_mutex;
  raw_can_data_t data[100];
  int put_ind = 0;
  int get_ind = 0;
  int num_dat = 0;
}can_thread_t;

void* read_BUSCAN(void *p)
{
  can_thread_t *can = (can_thread_t *)p;

  CanHandle h;
  bool init_buscan = false;

  /* Localizar Canal KVASER */
  int chanCount = 0;
  //int stat;
  int channel = -1;

  //stat = canGetNumberOfChannels(&chanCount);
  canGetNumberOfChannels(&chanCount);

  if (chanCount < 0 || chanCount > 64)
  {
      ROS_ERROR("ChannelCount = %d but I don't believe it.\n", chanCount);
      exit(1);
  }
  else
  {
      channel=chanCount - 1; //Abre el ultimo canal que ha detectado

      /* Open channels, parameters and go on bus */
      h = canOpenChannel(channel, canWANT_EXCLUSIVE | canWANT_EXTENDED);

      if (h < 0)
      {
          ROS_ERROR("canOpenChannel %d failed\n", channel);
          init_buscan = 0;
      }
      else
      {
          canSetBusParams(h, BAUD_500K, 4, 3, 1, 1, 0);
          canSetBusOutputControl(h, canDRIVER_NORMAL);
          canBusOn(h);
          init_buscan = 1;
      }
  }

  if(init_buscan)
  {
    ROS_INFO("BUSCAN inicializado");
   
    raw_can_data_t data;

    while (ros::ok())
    {
      
      int ret = canReadWait(h, &(data.id), &(data.msg), &(data.dlc), &(data.flag), &(data.time), -1);

      if(ret == canOK) //comando valido (ret=canOk=0)
      {
        can->data[can->put_ind] = data;
        can->put_ind++;
        if(can->put_ind == 100)
          can->put_ind = 0;
        
        can->num_dat++;
        if(can->num_dat == 100)
          printf("ERROR BUFFER ALCANZA LIMITE\n");
      }
    }
    canClose(h);
    //return 1;
  }
  else
  {
    ROS_ERROR("No se pudo inicializar el BUSCAN ");
    //return 0;
  }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "canbus_node");
  ros::NodeHandle n;
  ros::Publisher can_raw_pub = n.advertise<canbus::can_raw>("/canbus/raw", 1000);

  can_thread_t can;
  pthread_mutex_init(&(can.acces_mutex), NULL);
  pthread_create(&(can.task), NULL, read_BUSCAN, &can);

  raw_can_data_t data;
  canbus::can_raw raw_msg;
  
  while (ros::ok())
  {
    if(can.num_dat > 0)
    {
      data = can.data[can.get_ind];
      can.get_ind++;
      can.num_dat--;
      if(can.get_ind == 100)
        can.get_ind = 0;

      raw_msg.id = data.id;
      raw_msg.time = data.time;
      memcpy(&(raw_msg.raw[0]), data.msg, sizeof(data.msg));
      can_raw_pub.publish(raw_msg);
    }
    ros::spinOnce();
  }
}