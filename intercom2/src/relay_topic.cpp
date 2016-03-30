
#include "intercom2/relay_topic.h"
//class which create  multimaster/chatter on the foreign pc 
relayTopic::relayTopic() {
     std::cout<<"node handle on!"<<"\n";
     ros::NodeHandle n;
   // g_pub = n.advertise<topic_tools::ShapeShifter>("/chatter", 1000);  
}

relayTopic::~relayTopic(){
}


 void relayTopic::callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event){
    
  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg = msg_event.getConstMessage();
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();

    if (!g_advertised) {
    // If the input topic is latched, make the output topic latched, #3385.
     bool latch = false;
  
        if (connection_header){
            ros::M_string::const_iterator it = connection_header->find("latching");

            if((it != connection_header->end()) && (it->second == "1")){
                 ROS_DEBUG("input topic is latched; latching output topic to match");
                 latch = true;
            }
        }
      
          //create generic publisher and publish mssage
        g_pub = msg->advertise(n, g_output_topic, 10, latch);
        g_advertised = true;
        ROS_INFO("advertised as %s\n", g_output_topic.c_str());
    }

    g_pub.publish(msg);
}


void relayTopic::subscribe(std::string g_input_topic,ros::NodeHandle nh){
  
      g_th.unreliable().reliable();

    g_output_topic=g_input_topic+"_relay";//the name of publisher on foreign pc 

   // g_sub = new ros::Subscriber(nh.subscribe(g_input_topic, 10, &relayTopic::callback,this,g_th));

   
}





