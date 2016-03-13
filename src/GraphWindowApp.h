/*
 * GraphWindowApp.h
 *
 *  Created on: February 11, 2016
 *      Author: Denis Tananaev
 */

#ifndef GRAPHWINDOWAPP_H
#define	GRAPHWINDOWAPP_H


//Common libraries
#include <iostream>
#include <cstdlib>
#include <map>
#include <vector>

//QT libraries
#include <QApplication>
#include <QComboBox>
#include <QKeyEvent>
#include <QMenuBar>
#include <QFileDialog>
#include <QString>
#include <QScrollBar>
#include <QMainWindow>
#include "ui_graph_window.h"
//ROS libraries
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

//Add messages
#include "graph_planner/Point.h"
#include "graph_planner/Edge.h"
#include "graph_planner/PointCmd.h"
#include "graph_planner/EdgeCmd.h"
#include "graph_planner/GraphStructure.h"
#include "graph_planner/Plan.h"
//Add enum variables
#include "graph_planner/point_cmd_def.h"
#include "graph_planner/edge_cmd_def.h"


/*! Class GraphWindow is a GUI window for 2d graph planing */
class GraphWindowApp : public QMainWindow, private Ui_GraphWindow {
    Q_OBJECT
  public slots:

   //! addNewPoint function. 
    /*!
       Send message command "new_point" with (0,0) position on map to "/point_cmd" ROS topic  
       after pushing "newPointButton". 
     */
    void addNewPoint();

    //! deleteSelectedPoint function. 
    /*!
        Read waypoint key_id from "pointBox".
        Send message command "del_point" and key_id of waypoint to "/point_cmd" ROS topic.
     */
    void deleteSelectedPoint();

   //! addUpdate function. 
     /*!
       Read from "edgeBox".
       If edgeBox set "add new edge" send command "add_edge" and key_id of points from "fromPointBox" "toPointBox" and 
        to "/edge_cmd" ROS topic after pushing "addUpdateButton".
       Else send command "update_edge" and key_id of edge from "edgeBox" and key_id of points from "fromPointBox" "toPointBox" and 
       to "/edge_cmd" ROS topic after pushing "addUpdateButton".
      */
      void addUpdate();

     //! updateEdge function. 
     /*!
        \param index an integer argument.
        Read index from "edgeBox".
        Set "fromPointBox", "toPointBox" and "weightBox" with information of edge with index from "edgeBox".
      */
    void updateEdge(int);

      //! saveGraph function. 
     /*!
        Send command "save_graph" to "/edge_cmd" ROS topic after pushing "saveButton".
      */
    void saveGraph();
      //! deleteEdge function. 
     /*!
       Read from "edgeBox" key_id of edge.
       Send command "del_edge" and key_id of edge to "/edge_cmd" ROS topic after pushing "deleteEdgeButton".
      */
    void deleteEdge();

    void findPath();
public:

    //! A constructor.
    /*!
      Create buttons connection.
    */
  GraphWindowApp(QWidget *parent = 0);
    //! A destructor.
  ~GraphWindowApp();

    //! init function.
    /*!
      Create topics "/point_cmd" and "/edge_cmd". Subscribes to topic "/graph".
    */
    void init();

     //! graphCallback function.
    /*!
      Read information from "/graph" ROS topic.
      Fill with this information "pointBox", "edgeBox", "fromPointBox", "toPointBox".  
    */
    void graphCallback(const  graph_planner::GraphStructure::ConstPtr& cmd);
private:

    ros::NodeHandle nh_; /*!< ROS NodeHandle. */

    ros::Publisher cmd_point_pub_, cmd_edge_pub_,plan_pub_;/*!<ROS publishers for "/point_cmd", "/edge_cmd"  and "plan_cmd"  topics.*/
    ros::Subscriber graph_sub_;/*!<ROS subscribe for "/graph" topic.*/

    graph_planner::GraphStructure list_;/*!<List with information of points and edges. Updates from "/graph" topic.*/
     
    std::vector<int> indx_from_;  /*!<vector with key_id of points ( points "from" for edges) */
    std::vector<int> indx_to_;  /*!<vector with key_id of points ( points "to" for edges) */ 
};

#endif /* GRAPHWINDOWAPP_H */

