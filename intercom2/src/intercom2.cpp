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
    bool getTopicsList();

    std::vector<topic> hostTopicsList;
    std::vector<topic> foreignTopicsList;

     std::string input_topic, namesp;
    std::string foreign_master, host_master;
     std::string foreign_ip;
     int foreign_port, msgsFrequency_Hz;
     ros::NodeHandle nh;
private:

    std::string  config_name_;
    std::string  folder_path_;

};
multimaster::multimaster(){

    config_name_ = "config"; 
    folder_path_ = ros::package::getPath("intercom2"); 
 
}
multimaster::~multimaster(){
}
std::string multimaster::foreign_master_uri(){
    return foreign_master;
}

bool multimaster::getTopicsList(){
    
 std::ifstream file_stream_topics(folder_path_+"/"+ config_name_ + ".yaml");
    if (file_stream_topics.is_open()) { // check if file exsist
         std::string line;
        std::string parser;
  
         while (std::getline(file_stream_topics, line)) {
            std::istringstream iss(line);
            getline(iss, parser, ':');
            std::cout<<"parser= "<<parser.c_str()<<"\n";
           //get the list of topics from local computer
            if(parser=="local_pubs"){
                        std::string delim =",";
                        auto start=line.find(":")+3;
                        auto end = line.find(delim);
                   while(end != std::string::npos){

                        topic temp;
                        temp.is_advertised=false;
                        temp.topicName=line.substr(start, end - start);
                        hostTopicsList.push_back(temp);

                        std::cout << line.substr(start, end - start) << std::endl;
                        start = end + delim.length();
                         end = line.find(delim, start);
                    } 
                }

                 //get the list of topics from foreign computer
                 if(parser=="foreign_pubs"){
                        std::string delim =",";
                        auto start=line.find(":")+3;
                        auto end = line.find(delim);
                   while(end != std::string::npos){
                        getline(iss, parser, ',');
                        topic temp;
                        temp.is_advertised=false;
                        temp.topicName=line.substr(start, end - start);
                        foreignTopicsList.push_back(temp);

                        std::cout << line.substr(start, end - start) << std::endl;
                        start = end + delim.length();
                        end = line.find(delim, start);
                    } 
                }
         }
         file_stream_topics.close();
    }else{

       return false;
    }
   
   return true;

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
     nh.param<std::string>("namespace", namesp, "intercom2");
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
        for(int i=0; i<hostTopicsList.size();i++){
    
        pc.subscribe(hostTopicsList[i],namesp);
    //Create subscribers in the host and connect them to the foreign topics 
   remappings["__master"] = host_master;
    ros::master::init(remappings);
     //g_sub = new ros::Subscriber(nh.subscribe(g_input_topic, 10, &relayTopic::callback,this,g_th));
    ros::Subscriber *subscriberFeedback = new ros::Subscriber(nh.subscribe(hostTopicsList[i].topicName, 1, &relayTopic::callback, &pc));  
    remappings["__master"] =  foreign_master;
    ros::master::init(remappings);
  std::cout<<"hostTopicsList"<<"["<<i<<"]= "<<hostTopicsList[i].topicName.c_str()<<"\n";
     ros::spinOnce();
    }         
   // pc.subscribe(input_topic,nh);
    //ros::spinOnce(); 

 
    while(ros::ok() && ros::master::check()==true){
        ros::spinOnce(); 
         //std::cout<<"spin go"<<"\n";
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
      //   ROS_ERROR_STREAM("CAN'T READ THE PARAMETERS FROM LAUNCH FILE"<<"\n";
        return 0;
    }


    if(mmaster.getTopicsList()==false){
       // ROS_ERROR_STREAM("CAN'T READ THE CONFIGURATION FILE WITH THE LIST OF TOPICS"<<"\n";
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
   loop_rate_main.sleep();
    }

    return 0;
}
