/*
 * intercom2.cpp
 *
 *  Created on: march 30, 2016
 *      Author: Denis Tananaev
 */



#include "intercom2/relay_topic.h"

namespace ros {
    namespace master {
        void init(const M_string& remappings);
    }
}


//The class with functions which read the parameters from launch file and subscribe to the multimaster/chatter
class multimaster {
public:
    multimaster();
    ~multimaster();
     bool getParam();
     std::string foreign_master_uri();
     void init(ros::M_string remappings);

     std::string input_topic;
    std::string foreign_master, host_master;
     std::string foreign_ip;
     int foreign_port, msgsFrequency_Hz;
     ros::NodeHandle nh;
private:

};
multimaster::multimaster(){
   
}
multimaster::~multimaster(){
}
std::string multimaster::foreign_master_uri(){
return foreign_master;
}

bool multimaster::getParam(){

     //Read ip adress and port of foreign master from the launch file
    if(!nh.getParam("foreign_ip",foreign_ip)|| foreign_ip==""){
      ROS_ERROR_STREAM("PLEASE SPECIFY THE IP ADRESS OF FOREIGN MASTER IN THE LAUNCH FILE");
     return false;    
    }else if(! nh.getParam("foreign_port",foreign_port)){
      ROS_ERROR_STREAM("PLEASE SPECIFY THE PORT  OF FOREIGN MASTER IN THE LAUNCH FILE");
      return false;    
    }

    // Get the other parameters from the launch file     
    nh.param<int>("msgsFrequency_Hz", msgsFrequency_Hz, 10);
    nh.getParam("topic_name",input_topic);
    //Get the host and foreign master_uri
        std::stringstream ss;
        ss <<"http://"<<foreign_ip<<":"<<foreign_port<<"/";
        foreign_master=ss.str();
        host_master=ros::master::getURI();
       std::cout<<"topic name is: "<<input_topic.c_str()<<"\n"; 
    return true;
}


void multimaster::init(ros::M_string remappings) {
                  
    ros::Rate loop_rate(msgsFrequency_Hz);     
                 relayTopic pc; 
    //Create subscribers in the host and connect them to the foreign topics 

             
//ros::spinOnce(); 
   // ros::Subscriber subscriberFeedback = nh.subscribe("/chatter", 1, &foreignTopic::callback, &pc);  
    //remappings["__master"] =  foreign_master;
   //ros::master::init(remappings);
    pc.subscribe(input_topic,nh);
        remappings["__master"] = host_master;
    ros::master::init(remappings);
     pc.subscribe(input_topic,nh);
    while(ros::ok() && ros::master::check()==true){
        ros::spinOnce(); 
         std::cout<<"spin go"<<"\n";
        loop_rate.sleep();
    }

}




int main(int argc, char **argv){
    ros::M_string     remappings;

    //init ROS    
    ros::init(argc, argv,"multimaster_example");
          

     ros::NodeHandle nh;  
    ros::Rate loop_rate_main(200);//check the connection to the master every 200 mc
     float foreign_master_works= false;//set by default that foreign master is turned off


    multimaster mmaster;
   if(mmaster.getParam()==false){
    return 0;
    }
 
    //remap to the foreign master 
    remappings["__master"] = mmaster.foreign_master_uri();
    ros::master::init(remappings);
   //ros::NodeHandle nn; 

    //first check
if (ros::master::check()==false){
    ROS_ERROR_STREAM("DISCONNECTED FROM THE ROS_MASTER_URI:= "<< mmaster.foreign_master_uri());
}

    while(ros::ok()){
        //check that master is working
        if(ros::master::check()==true && foreign_master_works==false){
            foreign_master_works=true;   
     ROS_INFO_STREAM("CONNECTED TO THE ROS_MASTER_URI:= "<<mmaster.foreign_master_uri());      
               
            mmaster.init(remappings);                
         } else if(ros::master::check()==false && foreign_master_works==true){
                foreign_master_works=false;
               ROS_ERROR_STREAM("DISCONNECTED FROM THE ROS_MASTER_URI:= "<<mmaster.foreign_master_uri());                    
               
           }
 ros::spinOnce(); 
   loop_rate_main.sleep();
    }

    return 0;
}
