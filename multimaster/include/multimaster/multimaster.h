/*
 * File: multimaster
 * Author: Denis Tananaev
 * Date: 23.02.2017
 *
 */

#ifndef MULTIMASTER_H
#define MULTIMASTER_H

#include <multimaster/relay_topic.h>

using std::string;

namespace ros
{
namespace master
{
void init(const M_string& remappings);
}
}  // namespace ros

typedef struct
{
  std::string from;
  std::string to;
} tfTransform;


// The class with functions which read the parameters from launch file and subscribe topics
class multimaster
{
public:
  multimaster() : nh(), pnh("~"){};
  ~multimaster(){};

  bool getParam();
  std::string foreign_master_uri();
  void init(ros::M_string remappings);

  bool getHostTopicsList();
  bool getForeignTopicsList();
  void host2foreign(ros::M_string remappings);
  void foreign2host(ros::M_string remappings);

  std::vector<std::string> hostTopicsList;
  std::vector<std::string> foreignTopicsList;
  std::vector<tfTransform> hostTfList;
  std::vector<tfTransform> foreignTfList;

  std::string namesp;
  std::string foreign_master, host_master;
  std::string foreign_ip;
  int foreign_port;
  double msgs_pub_freq, observ_freq;

private:
  ros::NodeHandle nh, pnh;
};

#endif /* MULTIMASTER_H */