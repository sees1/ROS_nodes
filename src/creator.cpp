/* 
 * File:   creator.cpp
 * Author: Vladislav Tananaev
 * 
 * Created on February 20, 2016, 1:21 PM
 */

#include <map>
#include <memory>
#include <fstream>
#include <iostream>
#include "graph_planner/creator.h"

GraphPoint::GraphPoint(int32_t point_key_id, double x, double y, std::string name ) {
    id_ = point_key_id;
    x_ = x;
    y_ = y;
    name_ = name;
}

GraphEdge::GraphEdge(Graph<int32_t>::EdgeKey edge_key_id, Graph<int32_t>::PointPtr from_point,
        Graph<int32_t>::PointPtr to_point, double weight) {

    id_ = edge_key_id;
    weight_ = weight;
    from_ = from_point;
    to_ = to_point;
}

Creator::Creator() {


}

Creator::~Creator() {
}

int32_t Creator::addPoint(double x, double y, std::string name) {


    // find the lowest unused key
    int32_t point_key_id = 1;
    while (points_->find(point_key_id) != points_->end()) {
        point_key_id++;
    }

    if (name == "") {
        name = "Point_" + std::to_string(point_key_id);
    }

    std::shared_ptr<GraphPoint> point(new GraphPoint(point_key_id, x, y, name));
    Graph<int32_t>::PointPtr point_casted = std::dynamic_pointer_cast<Graph<int32_t>::Point>(point);

    // add to graph
    bool added = Graph<int32_t>::addPoint(point_key_id, point_casted);


    return point_key_id;
}

int32_t Creator::addEdge(int32_t from_point, int32_t to_point, double weight) {


    auto from = points_->find(from_point);
    auto to = points_->find(to_point);

    if (from == points_->end() || to == points_->end()) {
        return 0;
    }

    //find lowest free edge key id
    EdgeKey edge_key_id = 1;
    while (edges_->find(edge_key_id) != edges_->end()) {
        edge_key_id++;
    }

    std::shared_ptr<GraphEdge> e(new GraphEdge(edge_key_id, from->second, to->second, weight));
    Graph<int32_t>::EdgePtr edge_casted = std::dynamic_pointer_cast<Graph<int32_t>::Edge>(e);

    return Graph<int32_t>::addEdge(edge_casted, edge_key_id);
}



std::shared_ptr<GraphPoint> Creator::getPoint(PointKey k) {
    auto it = points_->find(k);
    if (it != points_->end()) {
        return std::dynamic_pointer_cast<GraphPoint> (it->second);
    }

    return nullptr;
}

std::shared_ptr<GraphPoint> Creator::point2GraphPoint(PointPtr p) {
    return std::dynamic_pointer_cast<GraphPoint> (p);
}

std::shared_ptr<GraphEdge> Creator::edge2GraphEdge(EdgePtr p) {
    return std::dynamic_pointer_cast<GraphEdge> (p);
}


/*
std::shared_ptr<GraphEdge> Rail::getEdge(EdgeKey k) {
    auto it = edges_->find(k);
    if (it != edges_->end()) {
        return std::dynamic_pointer_cast<GraphEdge> (it->second);
    }

    return nullptr;
}
*/


bool Creator::loadGraphFromFile(std::string path, std::string name) {

    std::ifstream file_stream_points(path +"/"+ name + "_points.txt");
    if (file_stream_points.is_open()) { // check if file exsist
         std::string line;
         while (std::getline(file_stream_points, line)) {
            std::istringstream file_stream_points(line);
            std::string name;
             double x, y;
             
            file_stream_points>>name>>x>>y;
             addPoint(x,y,name);
         }
         file_stream_points.close();
    }else{
       return false;
    }

    std::ifstream file_stream_edge(path +"/"+ name + "_edge.txt");
    if (file_stream_edge.is_open()) {
        std::string line;
        while (std::getline(file_stream_edge, line)) {
            std::istringstream file_stream_edge(line);
            
             int from, to;
             double weight;

            file_stream_edge>>from>>to>>weight;
             addEdge(from,to,weight);
         }
         file_stream_edge.close();
     

    }
   
   return true;

}

bool Creator::saveGraphToFile(std::string path, std::string name) {

    std::string points_file_name = path + "/"+ name + "_points.txt";
    std::string edge_file_name = path + "/" +name + "_edge.txt";
    //start writing vertices
    std::ofstream GraphFile;
    GraphFile.open (points_file_name, std::ios::out | std::ios::trunc );
    if (GraphFile.is_open()){ 
         for(auto it = points_->begin(); it != points_->end(); it++){
              GraphPoint* v_ptr = dynamic_cast<GraphPoint*> (it->second.get());
              GraphFile<<v_ptr->name_<<" "<<v_ptr->x_<<" "<<v_ptr->y_<<"\n";
          
        } 
    }   
    else{
        return false;
    }
  GraphFile.close();
    //start writing vertices
  GraphFile.open (edge_file_name, std::ios::out | std::ios::trunc );
      if (GraphFile.is_open()){ 
            for(auto it = edges_->begin(); it != edges_->end(); it++){  
              //GraphEdge* e_ptr = dynamic_cast<GraphEdge*> (it->second.get());   
              GraphFile<<it->second.get()->from()->id()<<" "<<it->second.get()->to()->id()<<" "<<it->second.get()->weight()<<"\n";
          }     
       }
    GraphFile.close();
// if saved successfully return ture
    return true;
}
