
#include "intercom2/relay_topic.h"
//class which create  multimaster/chatter on the foreign pc 
relayTopic::relayTopic() {
     ros::NodeHandle n; 
       //topic_tools::ShapeShifter temp(std_msgs::Empty);
   // ros::Publisher pub=temp.advertise(n,"chat",10,false);
   // pub=n.advertise<std_msgs::Empty>("/relay/chatter2", 10);
  //  std::cout<< mPublishers["/relay/chatter2"]<<"\n";
}

relayTopic::~relayTopic(){
}

void relayTopic::conn_cb(const ros::SingleSubscriberPublisher&)
{
 // ros::Subscriber subscriber=n.subscribe<topic_tools::ShapeShifter>("chat", 10, boost::bind(&relayTopic::callback, this, _1, "chat") );
   // subs.push_back(subscriber);
}


 void relayTopic::callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event, std::string& topic){

     std::string publisher_name = msg_event.getPublisherName(); 

    boost::shared_ptr<topic_tools::ShapeShifter const> const &msg = msg_event.getConstMessage();
    boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();

    ros::Publisher publ = relayTopic::getPublisher(namesp_+"/"+topic, msg,connection_header);
    publ.publish(msg);

}

ros::Publisher relayTopic::getPublisher(const std::string& topic,  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg, boost::shared_ptr<const ros::M_string> const& connection_header){

 if (mPublishers.find(topic) == mPublishers.end()){

        bool latch = false;
       // If the input topic is latched, make the output topic latched.
        if (connection_header){
            ros::M_string::const_iterator it = connection_header->find("latching");

            if((it != connection_header->end()) && (it->second == "1")){
                 ROS_DEBUG("input topic is latched; latching output topic to match");
                 latch = true;
            }
        }

     //advertise new topic
     mPublishers[topic] = msg->advertise(n, topic, 10);
  }

    return mPublishers[topic];
}


void relayTopic::subscribe(std::string g_input_topic,std::string namesp, ros::NodeHandle nh ){
  
     std::cout<<namesp+"/"+ g_input_topic<<"\n";

    ros::Subscriber subscriber=nh.subscribe<topic_tools::ShapeShifter>(g_input_topic, 10, boost::bind(&relayTopic::callback, this, _1, g_input_topic) );
    subs.push_back(subscriber);
    namesp_=namesp;
   
}





