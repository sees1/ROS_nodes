/*
 *
 *
 *  Created on: Februar 23.02.2017
 *      Author: Denis Tananaev
 */

#include <multimaster_node/multimaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "foreign2host");  // init ROS

  Multimaster* mmaster = new FHMultimaster();

  mmaster->establish_connection();

  delete mmaster;

  return 0;
}