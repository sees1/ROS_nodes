/* 
 * File:   graph.cpp
 * Author: Tananaev Denis
 * 
 * Created on January 12, 2016
 */

#include "graph.hpp"
#include <algorithm>

//using namespace Eigen;

template<class T>
Graph<T>::Graph() {
    initialized_ = false;
}

template<class T>
Graph<T>::~Graph() {
}

template<class T>
void Graph<T>::init() {
    clear();
    initialized_ = true;
}

template<class T>
void Graph<T>::clear() {

    edges_.reset(new EdgeSet);
    points_.reset(new PointSet);
}

template<class T>
bool Graph<T>::addPoint(PointKey k, PointPtr v) {
    auto it = points_->find(k);
    if (it != points_->end()) {
        return false;
    }

    points_->insert(std::make_pair(k, v));
    return true;
}

template<class T>
bool Graph<T>::removePoint(PointKey k) {
    auto it = points_->find(k);
    if (it ==points_->end()) {
        return false;
    }

    const EdgeSet edges = it->second->edges();
    //it->second->edges().clear();

    for (auto e : edges) {
        removeEdge(e.first);
    }

    points_->erase(it);
}

template<class T>
typename Graph<T>::EdgeKey Graph<T>::addEdge(EdgePtr edge, EdgeKey edge_id) {

    if (edge.use_count() == 0) {
        return -1;
    }

    if (edge->from().use_count() == 0) {
        return -1;
    }

    if (edge->to().use_count() == 0) {
        return -1;
    }

    if (points_->find(edge->from()->id()) == points_->end()) {
        return -1;
    }

    if (points_->find(edge->to()->id()) == points_->end()) {
        return -1;
    }

    if (edges_->find(edge_id) != edges_->end()) {
        return -1;
    }


    edge->setId(edge_id);
    edge->from()->addEdge(edge);
    edge->to()->addEdge(edge);
    edges_->insert(std::make_pair(edge_id, edge));

    return edge->id();
}

template<class T>
bool Graph<T>::removeEdge(EdgeKey id) {

    auto e = edges_->find(id);

    if (e == edges_->end()) {
        return false;
    }

    e->second->from()->removeEdge(id);
    e->second->to()->removeEdge(id);

    edges_->erase(e);

    return true;
}

template<class T>
bool Graph<T>::edgeChangeTo(EdgeKey edge_id, PointKey to_id) {
    auto e = edges_->find(edge_id);

    if (e == edges_->end()) {
        return false;
    }

    auto to = points_->find(to_id);
    if (to == points_->end()) {
        return false;
    }

    e->second->to()->removeEdge(edge_id);
    e->second->setTo(to->second);

    to->second->addEdge(e->second);

    return true;
}

template<class T>
bool Graph<T>::edgeChangeFrom(EdgeKey edge_id, PointKey from_id) {
    auto e = edges_->find(edge_id);

    if (e == edges_->end()) {
        return false;
    }

    auto from = points_->find(from_id);
    if (from == points_->end()) {
        return false;
    }

    e->second->from()->removeEdge(edge_id);
    e->second->setFrom(from->second);

    from->second->addEdge(e->second);

    return true;
}


