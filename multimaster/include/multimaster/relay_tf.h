#ifndef RELAY_TF_H
#define RELAY_TF_H

#include <multimaster/relay_config.h>

using boost::shared_ptr;
using std::map;

class RelayTF
{
public:
  RelayTF(RelayTFConfig* cfg) : relay_nh_(), cfg_(cfg){};
  ~RelayTF();

  void listen(double rate, ros::Duration time);
  void setListener(tf::TransformListener* listener);
  void setBroadcaster(tf::TransformBroadcaster* broadcaster);

private:
  tf::TransformListener* listener_;
  tf::TransformBroadcaster* broadcaster_;
  RelayTFConfig* cfg_;
  ros::NodeHandle relay_nh_;
};

#endif /* RELAY_TF_H */