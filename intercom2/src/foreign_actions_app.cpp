/*
 * 
 *
 *  Created on: March 30, 2016
 *      Author: Denis Tananaev
 */


#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <navigation_actions/waypointAction.h>
#include <navigation_actions/NavGoalAction.h>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <iostream>
 #include <sstream>
#include <fstream>
#include <memory>
#include <ros/package.h>

namespace ros {
    namespace master {
        void init(const M_string& remappings);
    }
}
class connect{
public:
        connect(std::string nav_pc);
        ~connect();
    //waypoint action 
    void callbackFeedback(const navigation_actions::waypointActionFeedback::ConstPtr& msg);
    void callbackStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void callbackResult(const navigation_actions::waypointActionResult::ConstPtr& msg);
    //NavGoal action
    void callbackFeedbackNG(const navigation_actions::NavGoalActionFeedback::ConstPtr& msg);
    void callbackStatusNG(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void callbackResultNG(const navigation_actions::NavGoalActionResult::ConstPtr& msg);
    
    ros::NodeHandle n_;
    //waypoint publishers
    ros::Publisher feedback_, status_,result_;
    //NavGoal publishers
     ros::Publisher feedbackNG_, statusNG_,resultNG_;
   
private:

};

connect::connect(std::string nav_pc) {
    //waypoint topics
    feedback_= n_.advertise<navigation_actions::waypointActionFeedback>(nav_pc+"/waypoint_goal/feedback", 1);
    status_=n_.advertise<actionlib_msgs::GoalStatusArray>(nav_pc+"/waypoint_goal/status",1);
    result_=n_.advertise<navigation_actions::waypointActionResult>(nav_pc+"/waypoint_goal/result",1);
    //NavGoal topics
    feedbackNG_= n_.advertise<navigation_actions::NavGoalActionFeedback>(nav_pc+"/nav_goal/feedback", 1);
    statusNG_=n_.advertise<actionlib_msgs::GoalStatusArray>(nav_pc+"/nav_goal/status",1);
    resultNG_=n_.advertise<navigation_actions::NavGoalActionResult>(nav_pc+"/nav_goal/result",1);
    
}

connect::~connect(){
}

//Waypoint callback functions
void connect::callbackFeedback(const navigation_actions::waypointActionFeedback::ConstPtr& msg) {
    feedback_.publish(msg);   
}
void connect::callbackStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    status_.publish(msg);   
}
void connect::callbackResult(const navigation_actions::waypointActionResult::ConstPtr& msg) {
    result_.publish(msg);   
}
//NavGoal callback functions
void connect::callbackFeedbackNG(const navigation_actions::NavGoalActionFeedback::ConstPtr& msg) {
    feedbackNG_.publish(msg);    
}
void connect::callbackStatusNG(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    statusNG_.publish(msg);    
}
void connect::callbackResultNG(const navigation_actions::NavGoalActionResult::ConstPtr& msg) {
    resultNG_.publish(msg);   
}


class connectPC2{
public:
        connectPC2();
        ~connectPC2();
    


  
    //waypoint 
    void callbackGoal(const navigation_actions::waypointActionGoal::ConstPtr& msg);
    void callbackCancel(const actionlib_msgs::GoalID::ConstPtr& msg);
    //NavGoal 
    void callbackGoalNG(const navigation_actions::NavGoalActionGoal::ConstPtr& msg);
    void callbackCancelNG(const actionlib_msgs::GoalID::ConstPtr& msg);          

    ros::NodeHandle nh_;
    //waypoint
    ros::Publisher goal_, cancel_;
    //NavGoal
    ros::Publisher goalNG_, cancelNG_;
private:
};

connectPC2::connectPC2(): nh_() {
    //waypoint topics
    goal_= nh_.advertise<navigation_actions::waypointActionGoal>("waypoint_goal/goal", 1);
    cancel_= nh_.advertise<actionlib_msgs::GoalID>("waypoint_goal/cancel", 1);
    //NavGoal topics
    goalNG_= nh_.advertise<navigation_actions::NavGoalActionGoal>("nav_goal/goal", 1);
    cancelNG_= nh_.advertise<actionlib_msgs::GoalID>("nav_goal/cancel", 1);
}
connectPC2::~connectPC2(){
}
//waypoint feedback 
void connectPC2::callbackGoal(const navigation_actions::waypointActionGoal::ConstPtr& msg) {
   goal_.publish(msg);    
}
void connectPC2::callbackCancel(const actionlib_msgs::GoalID::ConstPtr& msg) {
  cancel_.publish(msg);   
}
//NavGoal feedback 
void connectPC2::callbackGoalNG(const navigation_actions::NavGoalActionGoal::ConstPtr& msg) {
   goalNG_.publish(msg);   
}
void connectPC2::callbackCancelNG(const actionlib_msgs::GoalID::ConstPtr& msg) {
  cancelNG_.publish(msg);   
}



void init(ros::M_string remappings, std::string foreign_master_uri,std::string host_master_uri, std::string nav_pc, int msgsFrequency_Hz) {
    

    std::string foreign_master=foreign_master_uri;
    std::string host_master=host_master_uri;
    
    ros::NodeHandle n;             
              
    ros::Rate loop_rate(msgsFrequency_Hz);     
     //Create subscribers in the host and connect them to the foreign intercom topics 
 connectPC2 pc2;
    remappings["__master"] = host_master;
    ros::master::init(remappings);
   
    connect pc(nav_pc); 
    //waypoint    
    ros::Subscriber goal = n.subscribe(nav_pc+"/waypoint_goal/goal", 1, &connectPC2::callbackGoal, &pc2);
    ros::Subscriber cancel = n.subscribe(nav_pc+"/waypoint_goal/cancel", 1, &connectPC2::callbackCancel, &pc2);
    //NavGoal
    ros::Subscriber goalNG = n.subscribe(nav_pc+"/nav_goal/goal", 1, &connectPC2::callbackGoalNG, &pc2);
    ros::Subscriber cancelNG = n.subscribe(nav_pc+"/nav_goal/cancel", 1, &connectPC2::callbackCancelNG, &pc2);
   
   
    remappings["__master"] = foreign_master;
    ros::master::init(remappings);
       
     //waypoint
    ros::Subscriber subscriberFeedback = n.subscribe("waypoint_goal/feedback", 1, &connect::callbackFeedback, &pc);  
    ros::Subscriber subscriberStatus = n.subscribe("waypoint_goal/status", 1, &connect::callbackStatus, &pc); 
    ros::Subscriber subscriberResult = n.subscribe("waypoint_goal/result", 1, &connect::callbackResult, &pc);
    //NavGoal
    ros::Subscriber subscriberFeedbackNG = n.subscribe("nav_goal/feedback", 1, &connect::callbackFeedbackNG, &pc);  
    ros::Subscriber subscriberStatusNG = n.subscribe("nav_goal/status", 1, &connect::callbackStatusNG, &pc); 
    ros::Subscriber subscriberResultNG = n.subscribe("nav_goal/result", 1, &connect::callbackResultNG, &pc);

    remappings["__master"] = host_master;
    ros::master::init(remappings);
    while(ros::ok() && ros::master::check()==true){
        ros::spinOnce(); 
        loop_rate.sleep();
     
   
    }

}


int main(int argc, char **argv){
    const std::string node_name  = "host_actions_app";
    ros::M_string     remappings;
    float foreign_master_works= false;
 
    //init ROS    
    ros::init(argc, argv, node_name);
    //Init NodeHandle
    ros:: NodeHandle nh;
    
    std::string foreign_ip,  nav_pc;
    int foreign_port, connectCheck_Hz, msgsFrequency_Hz;

    //Read ip adress and port of foreign master from the launch file
    if(!nh.getParam("foreign_ip",foreign_ip)|| foreign_ip==""){
      ROS_ERROR_STREAM("PLEASE SPECIFY THE IP ADRESS OF FOREIGN MASTER IN THE LAUNCH FILE");
     return 0;    
    }else if(! nh.getParam("foreign_port",foreign_port)){
      ROS_ERROR_STREAM("PLEASE SPECIFY THE PORT  OF FOREIGN MASTER IN THE LAUNCH FILE");
      return 0;    
    }
    // Get the other parameters from the launch file    
   
    nh.param<std::string>("namespace", nav_pc, "intercom2");
    nh.param<int>("msgsFrequency_Hz", msgsFrequency_Hz, 10);

    //Check the foreign master core
     ros::Rate loop_rate_main(200);  

    //Get the host and foreign master_uri
        std::stringstream ss;
        ss <<"http://"<<foreign_ip<<":"<<foreign_port<<"/";
    std::string foreign_master=ss.str();
    std::string host_master=ros::master::getURI(); 

    //remap to the foreign master 
    remappings["__master"] = foreign_master;
    ros::master::init(remappings);
   
    //first check
if (ros::master::check()==false){
   // ROS_ERROR_STREAM("DISCONNECTED FROM THE ROS_MASTER_URI:= "<<foreign_master.c_str());
}

    while(ros::ok()){
        //check that master is working
        if(ros::master::check()==true && foreign_master_works==false){
            foreign_master_works=true;   
            // ROS_INFO_STREAM("CONNECTED TO THE ROS_MASTER_URI:= "<<foreign_master.c_str());                                                    
            init(remappings,foreign_master, host_master,nav_pc,msgsFrequency_Hz);                
         } else if(ros::master::check()==false && foreign_master_works==true){
                foreign_master_works=false;
              //  ROS_ERROR_STREAM("DISCONNECTED FROM THE ROS_MASTER_URI:= "<<foreign_master.c_str());                    
               
           }

   loop_rate_main.sleep();
    }

    return 0;
}
