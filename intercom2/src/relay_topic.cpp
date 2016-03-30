
#include "intercom2/relay_topic.h"
//class which create  multimaster/chatter on the foreign pc 

foreignTopic::foreignTopic() {
    chat_= n.advertise<std_msgs::String>("multimaster/chatter", 1000);
}

foreignTopic::~foreignTopic(){
}

void foreignTopic::callback(const std_msgs::String::ConstPtr& msg){
    chat_.publish(msg);
}
