/*
 *  File :multimaster.cpp
 *
 *  Created on: Februar 23.02.2017
 *      Author: Denis Tananaev
 */

#include <multimaster/multimaster.h>

using std::string;
using std::vector;

std::string multimaster::foreign_master_uri()
{
  return foreign_master;
}

bool multimaster::getHostTopicsList()
{
  if (!pnh.getParam("local_pubs",
                    hostTopicsList) || hostTopicsList.size() == 0)  // get list of publisher from host
  {
    ROS_ERROR("List of local publisher is not present in config or empty!");
    return false;
  }

  vector<string> local_tf_copy;
  if (!pnh.getParam("local_tf",
                           local_tf_copy) || local_tf_copy.size() == 0)  // get list of transforms from host
  {
    ROS_WARN("List of local tf transformation is not present in config or empty! Ignore if you dont need it!");
  }
  else
  {
    for (auto& transform_name : local_tf_copy)
    {
      string delim = "->";
      size_t split = transform_name.find(delim);
      hostTfList.push_back({ transform_name.substr(0, split), transform_name.substr(split + delim.length()) });
    }
    ROS_INFO("List of local TF is set!");
  }

  return true;
}

bool multimaster::getForeignTopicsList()
{
  if (!pnh.getParam("foreign_pubs",
                    foreignTopicsList) ||
      foreignTopicsList.size() == 0)  // get list of publisher from foreign master
  {
    ROS_ERROR("List of foreign publisher is not present in config or empty!");
    return false;
  }

  vector<string> foreign_tf_copy;
  if (!pnh.getParam("foreign_tf",
                           foreign_tf_copy) ||
      foreign_tf_copy.size() == 0)  // get list of transforms from foreign master
  {
    ROS_WARN("List of foreign tf transformation is not present in config or empty! Ignore if you dont need it!");
  }
  else
  {
    for (auto& transform_name : foreign_tf_copy)
    {
      string delim = "->";
      size_t split = transform_name.find(delim);
      foreignTfList.push_back({ transform_name.substr(0, split), transform_name.substr(split + delim.length()) });
    }
    ROS_INFO("List of foreign TF is set!");
  }

  return true;
}

bool multimaster::getParam()
{
  // Read ip adress and port of foreign master from the launch file
  if (!pnh.getParam("foreign_ip", foreign_ip) || foreign_ip == "")
  {
    ROS_ERROR("PLEASE SPECIFY THE IP ADRESS OF FOREIGN MASTER IN THE LAUNCH FILE");
    return false;
  }
  else if (!pnh.getParam("foreign_port", foreign_port))
  {
    ROS_ERROR("PLEASE SPECIFY THE PORT  OF FOREIGN MASTER IN THE LAUNCH FILE");
    return false;
  }

  // Get the other parameters from the launch file
  pnh.param<double>("msgs_pub_freq", msgs_pub_freq, 10.0);
  pnh.param<double>("observ_freq", observ_freq, 200.0);
  pnh.param<std::string>("namespace", namesp, "multimaster");

  // Get the host and foreign master_uri
  foreign_master = "http://" + foreign_ip + ":" + std::to_string(foreign_port) + "/";
  host_master = ros::master::getURI();

  return true;
}

void multimaster::host2foreign(ros::M_string remappings)
{
  ros::Rate loop_rate(msgs_pub_freq);

  tf::TransformBroadcaster broadcaster;

  ros::Time foreign_time = ros::Time::now();

  remappings["__master"] = host_master;
  ros::master::init(remappings);

  ros::Time host_time = ros::Time::now();
  ros::Duration difference = host_time - foreign_time;

  relayTopic pc;
  for (int i = 0; i < hostTopicsList.size(); ++i)
  {
    // Create subscribers in the host and connect them to the foreign topics
    pc.subscribe(hostTopicsList[i], namesp, nh);
    ROS_INFO("hostTopicsList [%i] = %s", i, hostTopicsList[i].c_str());
  }

  remappings["__master"] = foreign_master;
  ros::master::init(remappings);

  ros::Duration(0.5).sleep();
  while (ros::ok() && ros::master::check() == true)
  {
    ros::spinOnce();
    loop_rate.sleep();

    for (int i = 0; i < hostTfList.size(); ++i)
    {
      broadcaster.sendTransform(
          tf::StampedTransform(pc.listen(ros::Time::now() + difference, hostTfList[i].from, hostTfList[i].to)));
    }
  }
}

void multimaster::foreign2host(ros::M_string remappings)
{
  relayTopic pc2;

  for (int i = 0; i < foreignTopicsList.size(); ++i)
  {
    // Create subscribers in the host and connect them to the foreign topics
    pc2.subscribe(foreignTopicsList[i], namesp, nh);
    ROS_INFO("foreignTopicsList [ %i ] = %s", i, foreignTopicsList[i].c_str());
  }

  ros::Time foreign_time = ros::Time::now();
  remappings["__master"] = host_master;
  ros::master::init(remappings);

  ros::Time host_time = ros::Time::now();
  ros::Duration difference = host_time - foreign_time;

  tf::TransformBroadcaster broadcaster;

  ros::Rate r(msgs_pub_freq);

  ros::Duration(0.5).sleep();
  while (ros::ok() && ros::master::check() == true)
  {
    ros::spinOnce();
    r.sleep();

    for (int i = 0; i < foreignTfList.size(); ++i)
    {
      broadcaster.sendTransform(
          tf::StampedTransform(pc2.listen(ros::Time::now() + difference, foreignTfList[i].from, foreignTfList[i].to)));
    }
  }
}