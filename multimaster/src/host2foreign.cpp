/*
 *
 *
 *  Created on: Februar 23.02.2017
 *      Author: Denis Tananaev
 */

#include <multimaster/multimaster.h>

int main(int argc, char** argv)
{
  ros::M_string remappings;

  ros::init(argc, argv, "host2foreign");  // init ROS

  float foreign_master_works = false;  // set by default that foreign master is turned off

  multimaster mmaster;

  remappings["__master"] = mmaster.get_foreign_master_uri();  // remap to the foreign master
  ros::master::init(remappings);

  if (ros::master::check() == false)  // first check
  {
    ROS_WARN("DISCONNECTED FROM THE ROS_MASTER_URI := %s", mmaster.get_foreign_master_uri().c_str());
  }

  ros::Rate main_rate(mmaster.get_observ_frequency());

  while (ros::ok())
  {
    if (ros::master::check() == true && foreign_master_works == false)  // check that master is working
    {
      foreign_master_works = true;
      ROS_INFO("CONNECTED TO THE ROS_MASTER_URI := %s", mmaster.get_foreign_master_uri().c_str());
      mmaster.host2foreign(remappings);
    }
    else if (ros::master::check() == false && foreign_master_works == true)
    {
      foreign_master_works = false;
      ROS_WARN("DISCONNECTED FROM THE ROS_MASTER_URI := %s", mmaster.get_foreign_master_uri().c_str());
    }

    main_rate.sleep();
  }

  return 0;
}