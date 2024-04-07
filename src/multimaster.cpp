/*
 *  File :multimaster.cpp
 *
 *  Created on: Februar 23.02.2017
 *      Author: Denis Tananaev
 */

#include <multimaster_node/multimaster.h>

HFMultimaster::HFMultimaster() : nh(), pnh("~"), Multimaster()
{
  setupParam();
  manager = new HostRelayTopicManager();
  tf_manager = new HostRelayTFManager();
}

FHMultimaster::FHMultimaster() : nh(), pnh("~"), Multimaster()
{
  setupParam();
  manager = new ForeignRelayTopicManager();
  tf_manager = new ForeignRelayTFManager();
}

HFMultimaster::~HFMultimaster()
{
  delete manager;
  delete tf_manager;
}

FHMultimaster::~FHMultimaster()
{
  delete manager;
  delete tf_manager;
}

string HFMultimaster::get_foreign_master_uri()
{
  return foreign_master;
}

string FHMultimaster::get_foreign_master_uri()
{
  return foreign_master;
}

void HFMultimaster::switch_to_host()
{
  remappings["__master"] = host_master;
  ros::master::init(remappings);
}

void FHMultimaster::switch_to_host()
{
  remappings["__master"] = host_master;
  ros::master::init(remappings);
}

void HFMultimaster::switch_to_foreign()
{
  remappings["__master"] = foreign_master;
  ros::master::init(remappings);
}

void FHMultimaster::switch_to_foreign()
{
  remappings["__master"] = foreign_master;
  ros::master::init(remappings);
}

void HFMultimaster::setupParam()
{
  string foreign_ip_temp;
  int    foreign_port_temp;
  // Read ip adress and port of foreign master from the launch file
  if (!pnh.getParam("foreign_ip", foreign_ip_temp) || foreign_ip_temp == "")
  {
    ROS_ERROR("PLEASE SPECIFY THE IP ADRESS OF FOREIGN MASTER IN THE CONFIG FILE");
    std::terminate();
  }
  else if (!pnh.getParam("foreign_port", foreign_port_temp))
  {
    ROS_ERROR("PLEASE SPECIFY THE PORT OF FOREIGN MASTER IN THE CONFIG FILE");
    std::terminate();
  }

  // Get the other parameters from the launch file
  pnh.param<double>("msgs_pub_freq", msgs_pub_freq, 10.0);

  // Get the host and foreign master_uri
  foreign_master = "http://" + foreign_ip_temp + ":" + std::to_string(foreign_port_temp) + "/";
  host_master = ros::master::getURI();
}

void FHMultimaster::setupParam()
{
  string foreign_ip_temp;
  int    foreign_port_temp;
  // Read ip adress and port of foreign master from the launch file
  if (!pnh.getParam("foreign_ip", foreign_ip_temp) || foreign_ip_temp == "")
  {
    ROS_ERROR("PLEASE SPECIFY THE IP ADRESS OF FOREIGN MASTER IN THE CONFIG FILE");
    std::terminate();
  }
  else if (!pnh.getParam("foreign_port", foreign_port_temp))
  {
    ROS_ERROR("PLEASE SPECIFY THE PORT OF FOREIGN MASTER IN THE CONFIG FILE");
    std::terminate();
  }

  // Get the other parameters from the launch file
  pnh.param<double>("msgs_pub_freq", msgs_pub_freq, 10.0);

  // Get the host and foreign master_uri
  foreign_master = "http://" + foreign_ip_temp + ":" + std::to_string(foreign_port_temp) + "/";
  host_master = ros::master::getURI();
}

void HFMultimaster::establish_connection()
{
  manager->setupConfig(pnh);
  tf_manager->setupConfig(pnh);

  manager->connectToMaster();
  tf_manager->connectToMaster();

  switch_to_foreign();

  tf_manager->spin(msgs_pub_freq);
  manager->spin(msgs_pub_freq);
}

void FHMultimaster::establish_connection()
{
  manager->setupConfig(pnh);
  tf_manager->setupConfig(pnh);

  switch_to_foreign();

  manager->connectToMaster();
  tf_manager->connectToMaster();

  switch_to_host();

  tf_manager->spin(msgs_pub_freq);
  manager->spin(msgs_pub_freq);
}