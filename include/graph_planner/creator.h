/* 
 * File:   creator.h
 * Author: Vladislav Tananaev
 *
 * Created on February 20, 2016, 1:21 PM
 */

#ifndef CREATOR_H
#define CREATOR_H

#include "graph.hpp"

/*! Class GraphPoint is a point class*/
class GraphPoint : public Graph<int32_t>::Point {
public:
    
    //! A constructor.
    GraphPoint(int32_t point_key_id, double x, double y,  std::string name);
    
     //! A destructor.
    virtual ~GraphPoint() {

    }

    double x_, y_;
    std::string name_;
};

/*! Class GraphEdge is an edge class*/
class GraphEdge : public Graph<int32_t>::Edge {
public:
     //! A constructor.
    GraphEdge() {
    }

    GraphEdge(Graph<int32_t>::EdgeKey edge_key_id, Graph<int32_t>::PointPtr from_point,
            Graph<int32_t>::PointPtr to_point, double weight);

     //! A destructor.
    virtual ~GraphEdge() {
    }

};

/*! Class Creator is a graph class*/
class Creator : public Graph<int> {
public:
     //! A constructor.
    Creator();

     //! A destructor.
    virtual ~Creator();


    //! addPoint function. 
    /*!
        /param x is a x coordinate of the waypoint.
        /param y is a y coordinate of the waypoint.
        /param name is a name of the waypoint.
        The function add point to the graph.
     */
    int32_t addPoint(double x, double y, std::string name);

        //! addEdge function. 
        /*!
        /param from_point is a unique key_id of the point.
        /param to_point is a unique key_id of the point.
        /param weight is weight of the edge which affects on the path planning (Path planning function tryes to find path with the minimum weight).
        The function add edge to the graph.
         */
    int32_t addEdge(int32_t from_point, int32_t to_point, double weight);

    std::shared_ptr<GraphPoint> getPoint(PointKey k);
    std::shared_ptr<GraphPoint> point2GraphPoint(PointPtr p);

   // std::shared_ptr<GraphEdge> getEdge(EdgeKey k);
   // std::shared_ptr<GraphEdge> edge2GraphEdge(EdgePtr p);
    
    //! loadGraphFromFile function. 
        /*!
        /param path is a string which contain path to the folder with graph files.
        /param  name is a name of the graph.
        The function load graph from the file.
         */
   // bool loadGraphFromFile(std::string path, std::string name);

        //! saveGraphToFile function. 
        /*!
        /param path is a string which contain path to the folder with graph files.
        /param  name is a name of the graph.
        The function save graph from the file.
         */
   // bool saveGraphToFile(std::string path, std::string name);


private:

};

#endif	/* CREATOR_H */

