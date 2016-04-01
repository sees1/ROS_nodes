/*
 * intercom2.cpp
 *
 *  Created on: march 30, 2016
 *      Author: Denis Tananaev
 */

#include "intercom2/intercom2.h"


multimaster::multimaster(){

    config_name_ = "config"; 
    folder_path_ = ros::package::getPath("intercom2"); 
 
}

multimaster::~multimaster(){
}

std::string multimaster::foreign_master_uri(){
    return foreign_master;
}

bool multimaster::getHostTopicsList(){
    
 std::ifstream file_stream_topics(folder_path_+"/"+ config_name_ + ".yaml");
    if (file_stream_topics.is_open()) { // check if file exsist
         std::string line;
        std::string parser;
  
         while (std::getline(file_stream_topics, line)) {
            std::istringstream iss(line);
            getline(iss, parser, ':');

           //get the list of topics from local computer
            if(parser=="local_pubs"){
                        std::string delim =",";
                        auto start=line.find(":")+3;
                        auto end = line.find(delim);
                   while(end != std::string::npos){

                      
                       std::string temp=line.substr(start, end - start);
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
                   
                        std::string temp=line.substr(start, end - start);
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

bool multimaster::getForeignTopicsList(){
    
 std::ifstream file_stream_topics(folder_path_+"/"+ config_name_ + ".yaml");
    if (file_stream_topics.is_open()) { // check if file exsist
         std::string line;
        std::string parser;
  
         while (std::getline(file_stream_topics, line)) {
            std::istringstream iss(line);
            getline(iss, parser, ':');
    
                 //get the list of topics from foreign computer
                 if(parser=="foreign_pubs"){
                        std::string delim =",";
                        auto start=line.find(":")+3;
                        auto end = line.find(delim);
                   while(end != std::string::npos){
                        getline(iss, parser, ',');
                   
                        std::string temp=line.substr(start, end - start);
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


    
    //Get the host and foreign master_uri
        std::stringstream ss;
        ss <<"http://"<<foreign_ip<<":"<<foreign_port<<"/";
        foreign_master=ss.str();
        host_master=ros::master::getURI();
   

    return true;
}


void multimaster::host2foreign(ros::M_string remappings) {
                  
       ros::Rate loop_rate(msgsFrequency_Hz);     
           
    relayTopic pc;//foreign node handle
     remappings["__master"] = host_master;
     ros::master::init(remappings);
   

    for(int i=0; i<hostTopicsList.size();i++){         
        //Create subscribers in the host and connect them to the foreign topics 
        pc.subscribe(hostTopicsList[i],namesp, nh); 
        std::cout<<"hostTopicsList"<<"["<<i<<"]= "<<hostTopicsList[i].c_str()<<"\n";
    
    } 
    remappings["__master"] =  foreign_master;
    ros::master::init(remappings);

    while(ros::ok() && ros::master::check()==true){
        ros::spinOnce();
        loop_rate.sleep();
    }
 }  


void multimaster::foreign2host(ros::M_string remappings) {
          ros::Rate loop_rate(msgsFrequency_Hz); 
    relayTopic pc2; //host node handle

    remappings["__master"] =  foreign_master;
    ros::master::init(remappings);

     for(int i=0; i<foreignTopicsList.size();i++){         
    //Create subscribers in the host and connect them to the foreign topics 
         pc2.subscribe(foreignTopicsList[i],namesp, nh);   
        std::cout<<"foreignTopicsList"<<"["<<i<<"]= "<<foreignTopicsList[i].c_str()<<"\n";
    } 
remappings["__master"] = host_master;
     ros::master::init(remappings);
   
    while(ros::ok() && ros::master::check()==true){
        ros::spinOnce();
        loop_rate.sleep();
    }
}



