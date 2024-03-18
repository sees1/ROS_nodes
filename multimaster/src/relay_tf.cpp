#include <thread>
#include <mutex>
#include <multimaster/relay_tf.h>

RelayTF::~RelayTF()
{
  delete listener_;
  delete broadcaster_;
}

void RelayTF::setListener(tf::TransformListener* listener)
{
  listener_ = listener;
}

void RelayTF::setBroadcaster(tf::TransformBroadcaster* broadcaster)
{
  broadcaster_ = broadcaster;
}

void RelayTF::listen(double rate, ros::Duration time)
{
  // ros::Duration(5).sleep();
  ros::Rate loop_rate(rate);
  while(ros::ok())
  {
    try
    {
      for (const auto& tf_name : cfg_->getTFList())
      {
        tf::StampedTransform transform;
        listener_->waitForTransform(tf_name.from, tf_name.to, ros::Time::now() - time - ros::Duration(1), ros::Duration(1.0));
        listener_->lookupTransform(tf_name.from, tf_name.to, ros::Time::now() - time - ros::Duration(1), transform);
        transform.stamp_ = ros::Time::now();
        broadcaster_->sendTransform(transform);
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    loop_rate.sleep();
  }
}