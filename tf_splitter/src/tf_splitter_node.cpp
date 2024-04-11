#include <tf_splitter/tf_splitter_node.h>

class TfSplitter
{
public:
  TfSplitter();
  virtual ~TfSplitter(){};

  void callback(const std::vector<geometry_msgs::TransformStamped> & msgtf);

private:
  ros::NodeHandle nh, pnh;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
}

TfSplitter::TfSplitter() : nh(), pnh("~"), buffer_(5.0), tf_listener_(buffer_) {}

void TfSplitter::callback(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  
}