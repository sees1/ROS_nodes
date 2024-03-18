#include <multimaster/relay_config.h>

HostRelayTopicConfig::HostRelayTopicConfig(ros::NodeHandle& multimaster_nh) : config_pnh(multimaster_nh), RelayTopicConfig()
{
  if(!config_pnh.getParam("namespace", topic_ns))
  {
    ROS_ERROR("namespace field not present in config file!");
    std::terminate();
  }
  setupTopicsList();
}

HostRelayTopicConfig::HostRelayTopicConfig() : config_pnh(), RelayTopicConfig()
{
  if(!config_pnh.getParam("namespace", topic_ns))
  {
    ROS_ERROR("namespace field not present in config file!");
    std::terminate();
  }
  setupTopicsList();
}


ForeignRelayTopicConfig::ForeignRelayTopicConfig(ros::NodeHandle& multimaster_nh) : config_pnh(multimaster_nh), RelayTopicConfig()
{
  if(!config_pnh.getParam("namespace", topic_ns))
  {
    ROS_ERROR("namespace field not present in config file!");
    std::terminate();
  }
  setupTopicsList();
}

ForeignRelayTopicConfig::ForeignRelayTopicConfig() : config_pnh(), RelayTopicConfig()
{
  if(!config_pnh.getParam("namespace", topic_ns))
  {
    ROS_ERROR("namespace field not present in config file!");
    std::terminate();
  }
  setupTopicsList();
}

HostRelayTFConfig::HostRelayTFConfig(ros::NodeHandle& multimaster_nh) : config_pnh(multimaster_nh), RelayTFConfig()
{
  setupTFList();
}

HostRelayTFConfig::HostRelayTFConfig() : config_pnh(), RelayTFConfig()
{
  setupTFList();
}

ForeignRelayTFConfig::ForeignRelayTFConfig(ros::NodeHandle& multimaster_nh) : config_pnh(multimaster_nh), RelayTFConfig()
{
  setupTFList();
}

ForeignRelayTFConfig::ForeignRelayTFConfig() : config_pnh(), RelayTFConfig()
{
  setupTFList();
}

const vector<string>& HostRelayTopicConfig::getTopicsList()
{
  return hostTopicsList;
}

const vector<string>& ForeignRelayTopicConfig::getTopicsList()
{
  return foreignTopicsList;
}

string HostRelayTopicConfig::getNamespace()
{
  return topic_ns;
}

string ForeignRelayTopicConfig::getNamespace()
{
  return topic_ns;
}

const vector<tfTransform>& HostRelayTFConfig::getTFList()
{
  return hostTFList;
}

const vector<tfTransform>& ForeignRelayTFConfig::getTFList()
{
  return foreignTFList;
}

void HostRelayTopicConfig::setupTopicsList()
{
  if (!config_pnh.getParam("local_pubs",
                           hostTopicsList) ||
      hostTopicsList.size() == 0)  // get list of publisher from host
  {
    ROS_ERROR("List of local publisher is not present in config or empty!");
    std::terminate();
  }
  else
  {
    ROS_INFO("List of local publisher is setup properly!");
  }
}

void ForeignRelayTopicConfig::setupTopicsList()
{
  if (!config_pnh.getParam("foreign_pubs",
                    foreignTopicsList) ||
      foreignTopicsList.size() == 0)  // get list of publisher from foreign master
  {
    ROS_ERROR("List of foreign publisher is not present in config or empty!");
    std::terminate();
  }
  else
  {
    ROS_INFO("List of local publisher is setup properly!");
  }
}

void HostRelayTFConfig::setupTFList()
{
  vector<string> local_tf_copy;
  if (!config_pnh.getParam("local_tf",
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
      hostTFList.push_back({ transform_name.substr(0, split), transform_name.substr(split + delim.length()) });
    }
    ROS_INFO("List of local TF is set!");
  }
}

void ForeignRelayTFConfig::setupTFList()
{
  vector<string> foreign_tf_copy;
  if (!config_pnh.getParam("foreign_tf",
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
      foreignTFList.push_back({ transform_name.substr(0, split), transform_name.substr(split + delim.length()) });
    }
    ROS_INFO("List of foreign TF is set!");
  }
}