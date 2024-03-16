#ifndef RELAY_TOPIC_H
#define RELAY_TOPIC_H

#include <fstream>
#include <iostream>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <string>

#include <cstdio>
#include <topic_tools/parse.h>
#include <topic_tools/shape_shifter.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class relayTopic
{
public:
  relayTopic(){};
  ~relayTopic(){};

  // tf listener
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform listen(ros::Time time, std::string from, std::string to);

  // generic subscriber
  void subscribe(std::string g_input_topic, std::string namesp, ros::NodeHandle nh);

  // generic callback function which also create generic publisher
  void callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event, std::string& topic);

  ros::Publisher getPublisher(const std::string& topic, boost::shared_ptr<topic_tools::ShapeShifter const> const& msg,
                              boost::shared_ptr<const ros::M_string> const& connection_header);

private:
  std::map<std::string, ros::Publisher> mPublishers;
  std::vector<ros::Subscriber> subs;
  std::vector<std::string> topics;
  ros::NodeHandle* g_node = NULL;
  ros::NodeHandle n;
  std::string namesp_;
};

#endif /* RELAY_TOPIC_H */