
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

      void subscribe(std::string g_input_topic,std::string namesp, ros::NodeHandle nh);//generic subscriber
      void callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event, std::string& topic);//generic callback function which also create generic publisher
     ros::Publisher getPublisher(const std::string& topic,  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg, boost::shared_ptr<const ros::M_string> const& connection_header );

private:
      std::map<std::string, ros::Publisher> mPublishers;
      std::vector<ros::Subscriber> subs; 
      std::vector<std::string> topics; 
      ros::NodeHandle *g_node = NULL;
      ros::NodeHandle n;
      std::string namesp_;

};

#endif	/* RELAY_TOPIC_H */

