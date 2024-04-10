#include <thread>
#include <mutex>
#include <multimaster_node/relay_tf.h>

RelayTF::~RelayTF()
{
  delete buffer_;
  delete listener_;
  delete broadcaster_;
}

void RelayTF::setListener(tf2_ros::Buffer* buffer, tf2_ros::TransformListener* listener)
{
  buffer_ = buffer;
  listener_ = listener;
}

void RelayTF::setBroadcaster(tf2_ros::TransformBroadcaster* broadcaster)
{
  broadcaster_ = broadcaster;
}

void RelayTF::listen(double rate)
{
  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
    const std::vector<TfTransform>& transforms = cfg_->getTFList();
    for (const auto& tf_name : transforms)
    {
      try
      {
        auto transform = buffer_->lookupTransform(tf_name.from, tf_name.to, ros::Time(0));
        transform.header.stamp = ros::Time::now();
        broadcaster_->sendTransform(transform);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
    loop_rate.sleep();
  }
}
