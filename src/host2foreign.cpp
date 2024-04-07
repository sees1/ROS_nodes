/*
 *
 *
 *  Created on: Februar 23.02.2017
 *      Author: Denis Tananaev
 */

#include <multimaster_node/multimaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "host2foreign");  // init ROS

  Multimaster* mmaster = new HFMultimaster();

  mmaster->establish_connection();

  delete mmaster;

  return 0;
}