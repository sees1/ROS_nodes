
#ifndef RELAY_TOPIC_H
#define	RELAY_TOPIC_H


#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
 #include <sstream>
#include <fstream>
#include <ros/package.h>

#include <cstdio>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"


class relayTopic{
public:
      relayTopic();
     ~relayTopic();
      void subscribe(std::string g_input_topic,ros::NodeHandle nh);//generic subscriber

      void callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event);//generic callback function which also create generic publisher

    ros::TransportHints g_th;
     bool g_advertised = false;
    ros::Publisher g_pub;
    ros::Subscriber* g_sub;
    std::string g_output_topic;


private:
     ros::NodeHandle *g_node = NULL;
     ros::NodeHandle n;
};

#endif	/* RELAY_TOPIC_H */

