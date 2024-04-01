/*
 *
 *
 *  Created on: Februar 23.02.2017
 *      Author: Denis Tananaev
 */

#include <multimaster/multimaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "host2foreign");  // init ROS

  Multimaster* mmaster = new HFMultimaster();

  mmaster->switch_to_foreign();

  if (ros::master::check() == false)
  {
    ROS_WARN("DISCONNECTED FROM THE ROS_MASTER_URI := %s", mmaster->get_foreign_master_uri().c_str());
  }
  else
  {
    ROS_INFO("CONNECTED TO THE ROS_MASTER_URI := %s", mmaster->get_foreign_master_uri().c_str());
    mmaster->establish_connection();
  }

  delete mmaster;

  return 0;
}