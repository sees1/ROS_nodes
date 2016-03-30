
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


class foreignTopic{
public:
      foreignTopic();
     ~foreignTopic();
     void callback(const std_msgs::String::ConstPtr& msg);    
     ros::Publisher chat_;

private:
     ros::NodeHandle n;
};

#endif	/* RELAY_TOPIC_H */

