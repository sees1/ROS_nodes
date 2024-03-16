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

  ros::init(argc, argv, "foreign2host");//init ROS

  float foreign_master_works = false;//set by default that foreign master is turned off

  multimaster mmaster;
  if(mmaster.getParam() == false || mmaster.getForeignTopicsList() == false)
  {
    return 0;
  }

  remappings["__master"] = mmaster.foreign_master_uri();//remap to the foreign master
  ros::master::init(remappings);

  if (ros::master::check() == false)//first check
  {
    ROS_ERROR_STREAM("DISCONNECTED FROM THE ROS_MASTER_URI := " << mmaster.foreign_master_uri());
  }

  ros::Rate main_rate(mmaster.observ_freq);

  while(ros::ok())
  {
    if(ros::master::check() == true && foreign_master_works == false)//check that master is working
    {
      foreign_master_works = true;
      ROS_INFO_STREAM("CONNECTED TO THE ROS_MASTER_URI := " << mmaster.foreign_master_uri());
      mmaster.foreign2host(remappings);
    }
    else if(ros::master::check() == false && foreign_master_works == true)
    {
      foreign_master_works = false;
      ROS_ERROR_STREAM("DISCONNECTED FROM THE ROS_MASTER_URI := " << mmaster.foreign_master_uri());
    }

    main_rate.sleep();
  }

  return 0;
}