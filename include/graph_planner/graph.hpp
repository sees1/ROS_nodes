/*
 * File:   graph.hpp
 * Author: Denis Tananaev
 *
 * Created on January 12, 2016
 */

#ifndef GRAPH_H
#define	GRAPH_H

#include <memory>
#include <set>
#include <map>
#include <vector>
#include <array>

#include <iostream>
#include <eigen3/Eigen/Dense>

template<class T>
class Graph {
public:
    class Point;
    class Edge;

    typedef T PointKey; 
    typedef int32_t EdgeKey;

    typedef std::shared_ptr<Edge> EdgePtr; 
    typedef std::shared_ptr<Point> PointPtr;

    typedef std::map<EdgeKey, EdgePtr > EdgeSet;
    typedef std::map<PointKey, PointPtr > PointSet;

    typedef std::shared_ptr<EdgeSet > EdgeSetPtr;
    typedef std::shared_ptr<PointSet > PointSetPtr;

    class Path {
    public:

        std::vector<PointPtr> point_ptrs_;
        std::vector<EdgePtr> edge_ptrs_;
        double value;
    };

    class Point {
    public:

        Point() {

        }

        Point(PointKey id) {
            id_ = id;
            edges_.clear();
        }

        Point(PointKey id, EdgeSet edges) {
            id_ = id;
            edges_ = edges;
        }

        virtual ~Point() {

        }


        PointKey id() {
            return id_;
        }

        const EdgeSet& edges() {
            return edges_;
        }

        void addEdge(EdgePtr edge) {
            if (edges_.find(edge->id()) == edges_.end()) {
                edges_.insert(std::make_pair(edge->id(), edge));
            }
        }

        void removeEdge(EdgeKey id) {
            auto it = edges_.find(id);
            if (it != edges_.end()) {
                edges_.erase(it);
            }
        }

    protected:
        PointKey id_;
        EdgeSet edges_;
    };

    class Edge {
    public:

        Edge() {
            id_ = -1;
            weight_ = 0;
            
        }

        virtual ~Edge() {

        }

        EdgeKey id() {
            return id_;
        }

        void setId(EdgeKey id) {
            id_ = id;
        }

        PointPtr from() {
            return from_;
        }

        void setFrom(PointPtr from) {
            from_ = from;
        }

        PointPtr to() {
            return to_;
        }

        void setTo(PointPtr to) {
            to_ = to;
        }

        double weight() {
            return weight_;
        }

        void setWeight(double weight) {
            weight_ = weight;
        }
       

    protected:
        EdgeKey id_;
        double weight_;
        PointPtr from_;
        PointPtr to_;
    };

    Graph();

    virtual ~Graph();

    virtual void init();

    virtual bool isInitialized() {
        return initialized_;
    };

    virtual void clear();

  //  virtual void printGraphText();

    virtual PointSetPtr points() {
        return points_;
    }

    virtual EdgeSetPtr edges() {
        return edges_;
    }

   virtual bool addPoint(PointKey k, PointPtr v);
    virtual bool removePoint(PointKey k);
///////////////////////////////////////////////////
/*  
    virtual EdgeKey addEdge(EdgePtr e, EdgeKey edge_id);

    virtual bool removeEdge(EdgeKey id);

    virtual bool edgeChangeTo(EdgeKey edge_id, VertexKey to_id);
    virtual bool edgeChangeFrom(EdgeKey edge_id, VertexKey from_id);

 
  

    virtual std::vector< std::pair< VertexPtr, EdgePtr> > getNextVertex(EdgePtr last_edge, VertexPtr current_vertex);

    virtual std::vector< std::pair< VertexPtr, EdgePtr> > getNewVertex(EdgePtr last_edge, VertexPtr current_vertex);//for graph search with motion constraint (forward, backward, forward and backward)

   // virtual std::vector< std::shared_ptr<Path> > getPathes(VertexKey from, VertexKey to, unsigned int n); //has equivalent name with function from line 204 !

    virtual std::shared_ptr<Path> getPathVertex2Vertex(VertexKey from, VertexKey to);
    virtual std::shared_ptr<Path> getPathVertex2Edge(VertexKey from, EdgeKey to);
    virtual std::shared_ptr<Path> getPathEdge2Vertex(EdgeKey from, VertexKey to);
    virtual std::shared_ptr<Path> getPathEdge2Edge(EdgeKey from, EdgeKey to);

    virtual std::vector< std::shared_ptr<Path> > getPathes(unsigned int n, std::set<VertexKey> from, std::set<VertexKey> to);
*/


protected:

    Graph(const Graph<PointKey>& orig) {
    };

    Graph& operator=(const Graph<PointKey>&) {
        return *this;
    }

 //   virtual double computeWeight(EdgePtr last_edge_ptr, VertexPtr via_vertex, EdgePtr next_edge_ptr);

    
    EdgeSetPtr edges_;
    PointSetPtr points_;
    bool initialized_;

};

#include "graph.hxx"

#endif	/* GRAPH_H */

