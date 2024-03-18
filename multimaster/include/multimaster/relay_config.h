#ifndef RELAY_CONFIG_H
#define RELAY_CONFIG_H

#include <string>
#include <vector>
#include <map>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <topic_tools/parse.h>
#include <topic_tools/shape_shifter.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Empty.h>

using std::string;
using std::vector;

typedef struct
{
  string from;
  string to;
} tfTransform;

class RelayTopicConfig
{
public:
  RelayTopicConfig(){};
  virtual ~RelayTopicConfig(){};

  virtual const vector<string>& getTopicsList() = 0;
  virtual string getNamespace() = 0;

private:
  virtual void setupTopicsList() = 0;
};

class HostRelayTopicConfig : public RelayTopicConfig
{
public:
  HostRelayTopicConfig();
  HostRelayTopicConfig(ros::NodeHandle& multimaster_nh);
  virtual ~HostRelayTopicConfig(){};

  virtual const vector<string>& getTopicsList() override;
  virtual string getNamespace() override;

private:
  virtual void setupTopicsList() override;

private:
  vector<string> hostTopicsList;
  string topic_ns;
  ros::NodeHandle config_pnh;
};

class ForeignRelayTopicConfig : public RelayTopicConfig
{
public:
  ForeignRelayTopicConfig();
  ForeignRelayTopicConfig(ros::NodeHandle& multimaster_nh);
  virtual ~ForeignRelayTopicConfig(){};

  virtual const vector<string>& getTopicsList() override;
  virtual string getNamespace() override;

private:
  virtual void setupTopicsList() override;

private:
  vector<string> foreignTopicsList;
  string topic_ns;
  ros::NodeHandle config_pnh;
};

class RelayTFConfig
{
public:
  RelayTFConfig(){};
  virtual ~RelayTFConfig(){};

  virtual const vector<tfTransform>& getTFList() = 0;

private:
  virtual void setupTFList() = 0;
};

class HostRelayTFConfig : public RelayTFConfig
{
public:
  HostRelayTFConfig();
  HostRelayTFConfig(ros::NodeHandle& multimaster_nh);
  virtual ~HostRelayTFConfig(){};

  virtual const vector<tfTransform>& getTFList() override;

private:
  virtual void setupTFList() override;

private:
  vector<tfTransform> hostTFList;
  ros::NodeHandle config_pnh;
};

class ForeignRelayTFConfig : public RelayTFConfig
{
public:
  ForeignRelayTFConfig();
  ForeignRelayTFConfig(ros::NodeHandle& multimaster_nh);
  virtual ~ForeignRelayTFConfig(){};

  virtual const vector<tfTransform>& getTFList() override;

private:
  virtual void setupTFList() override;

private:
  vector<tfTransform> foreignTFList;
  ros::NodeHandle config_pnh;
};

#endif /* CONFIG_RELAY_H */