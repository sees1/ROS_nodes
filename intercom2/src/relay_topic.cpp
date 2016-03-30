
#include "intercom2/relay_topic.h"
//class which create  multimaster/chatter on the foreign pc 
relayTopic::relayTopic() {

     ros::NodeHandle n;
   
}

relayTopic::~relayTopic(){
}


 void relayTopic::callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event){

     std::string publisher_name = msg_event.getPublisherName(); 
   publisher_name= namesp_+"/"+publisher_name;  
  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg = msg_event.getConstMessage();
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();
/*
    if (!g_advertised) {
    // If the input topic is latched, make the output topic latched.
     bool latch = false;
  
        if (connection_header){
            ros::M_string::const_iterator it = connection_header->find("latching");

            if((it != connection_header->end()) && (it->second == "1")){
                 ROS_DEBUG("input topic is latched; latching output topic to match");
                 latch = true;
            }
        }
      
          //create generic publisher and publish mssage
         ros::Publisher g_pub;

        //g_pub = msg->advertise(n, g_output_topic, 10, latch);
 
       g_publish =new ros::Publisher(msg->advertise(n, g_output_topic, 10, latch));


        g_advertised = true;
        ROS_INFO("advertised as %s\n", g_output_topic.c_str());
        //std::cin.get();
    }

*/
   // g_pub.publish(msg);
ros::Publisher pub = relayTopic::getPublisher(publisher_name, msg);
pub.publish(msg);
   //g_publish->publish(msg);
}

ros::Publisher relayTopic::getPublisher(const std::string& topic,  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg ){
 if (mPublishers.find(topic) == mPublishers.end()){
    //ros::AdvertiseOptions opts = createAdvertiseOptions(msg,10);
 mPublishers[topic] = msg->advertise(n, topic, 10);
   // ros::NodeHandle mNh;
   // mPublishers[topic] = mNh.advertise(opts);
  }

    return mPublishers[topic];
}


void relayTopic::subscribe(topic g_input_topic,std::string namesp){
  
     // g_th.unreliable().reliable();
    namesp_=namesp;
    g_output_topic=namesp+"/" + g_input_topic.topicName;//the name of publisher on foreign pc 

    g_advertised=g_input_topic.is_advertised;
    std::cout<<"topic_name= "<<g_output_topic<<"\n";
    std::cout<<"advertise= "<<(bool)g_advertised<<"\n";

    //ros::Publisher(advertise(const std::string& topic, uint32_t queue_size, bool latch = false);

   // g_sub = new ros::Subscriber(nh.subscribe(g_input_topic, 10, &relayTopic::callback,this,g_th));

   
}





