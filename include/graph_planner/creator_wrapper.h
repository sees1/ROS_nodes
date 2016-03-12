/* 
 * File:   creator_wrapper.h
 * Author: Vladislav Tananaev
 *
 * Created on February 17, 2016, 8:29 AM
 */

#ifndef CREATORWRAPPER_H
#define	CREATORWRAPPER_H

#include "creator.h"
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

//Add messages
#include "graph_planner/Point.h"
#include "graph_planner/Edge.h"
#include "graph_planner/PointCmd.h"
#include "graph_planner/EdgeCmd.h"
#include "graph_planner/GraphStructure.h"
//Add enum variables
#include "graph_planner/point_cmd_def.h"
#include "graph_planner/edge_cmd_def.h"

//Add ROS messages
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <interactive_markers/interactive_marker_server.h>

#include <cmath>
#include <stdlib.h> 
#include <math.h>
#include <eigen3/Eigen/Dense>



/*! Class CreatorWrapper is a connection between GUI and ROS functions*/
class CreatorWrapper : public Creator {
public:

    //! A constructor.
     /*!
      The constructor creates ROS topics: "/points"; "/graph"; 
      It subscribes to the next ROS topics:
      "/graph_viz"; "/query_viz"; "/path_viz"; "/point_cmd"; "/edge_cmd";
      It also creates interactive marker server "point_marker";
     */
    CreatorWrapper();

    //! A destructor.
    virtual ~CreatorWrapper();

    //! init function. 
    /*!
       The function loads the graph (if it is exist).
     */
    void init();

    //! pointCmdCallback function. 
    /*!
        /param cmd is a message which includes the information about point, see ../graph_planner/msgs/VertexCmd.msg.
        The function gets the information from the buttons of the gui and implements the necessary operations with points (add, delete, update) in rviz.
     */
    void pointCmdCallback(const graph_planner::PointCmd::ConstPtr &cmd);

    //! edgeCmdCallback function. 
    /*!
        /param cmd is a message which includes the information about edge, see ../graph_planner/msgs/EdgeCmd.msg.
        The function gets the information from the buttons of the gui in rail_creator mode and implements the necessary operations with edges (add, update, delete) in rviz.
     */
    void edgeCmdCallback(const graph_planner::EdgeCmd::ConstPtr& cmd);
 
    //! pubGraph function. 
    /*!
        The function publishes visualisation markers for the graph at the "/graph" topic for rviz.
     */

    void pubGraph();

    //RVIZ visualisation functions

    //! pubRvizGraph function. 
    /*!
        The function publishes graph to the "/graph_viz" topic.
     */
    void pubRvizGraph();


    //! point2msg function. 
    /*!
        /param in is a pointer on the Point information (x,y, name).
        /param key_id is a unique number for each point
        /param out is a pointer with Point information  
        The function gets the information from the points list of <map> and rewrites this information to the output pointer. 
     */
    void point2msg(Point* in, int32_t key_id, graph_planner::Point* out);

    //! pointUpdateFromMsg function. 
    /*!
        /param v  is the Point information (x,y, name ).
        The function updates currently existed vertex with new information from the topic "/point_cmd".
     */
    void pointUpdateFromMsg(graph_planner::Point v);

    //! edge2msg function. 
    /*!
         /param in is a pointer on the Edge information (from, to).
        /param out is a pointer with Edge information.  
         The function gets the information from the edge list of <map> and rewrites this information to the output pointer.
      */
    void edge2msg(Edge* in, graph_planner::Edge* out);

    //! edgeUpateFromMsg function. 
    /*!
        /param e  is the edge information (from, to, motion).
        The function updates currently existed edge with new information from the topic "/edge_cmd".
     */
    void edgeUpateFromMsg(graph_planner::Edge e);

     //! spinOnes function. 
    /*!
       The function visualise the goal and the path through the graph . 
     */
    void spinOnes();

    //! addInteractiveMarker function. 
    /*!
        /param id is a point key_id.
        The function publishes interactive marker at the position of the waypoint with input key_id.
     */
    void addInteractiveMarker(PointKey id);
    void processMarkerFedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    
private:
    std::string name_;
    std::string folder_path_ ;
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher graph_pub_;
    ros::Publisher graph_viz_pub_;

    ros::Subscriber point_cmd_sub_, edge_cmd_sub_, planner_cmd_sub_, goal_sub_;

    ros::Publisher vis_pub_;

    interactive_markers::InteractiveMarkerServer* marker_server_;
    std::map<std::string, PointKey> marker_name_2_pointkey_;

};

#endif	/* CREATORWRAPPER_H */

