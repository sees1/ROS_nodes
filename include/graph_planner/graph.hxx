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


template<class T>
std::vector<typename std::shared_ptr<typename Graph<T>::Path> > Graph<T>::getPathes(unsigned int n, PointKey from, PointKey to) {

    std::multiset< std::pair<double, std::shared_ptr<Path> > > path_multi_set;
    std::vector< std::shared_ptr<Path> > pathes;

    std::map<EdgeKey, int> edge_selected_counter_;

    for (auto edge : *edges_) {
        edge_selected_counter_[edge.first] = 0;
    }


    //initalize
    std::shared_ptr<Path> current_path_ptr;
    current_path_ptr.reset(new Path);
    current_path_ptr->point_ptrs_.push_back(points_->at(from));
    current_path_ptr->edge_ptrs_.push_back(std::make_shared<Edge>());
    current_path_ptr->value = 0;
    path_multi_set.insert(std::make_pair(current_path_ptr->value, current_path_ptr));

    bool first = true;
    while (!path_multi_set.empty() && pathes.size() < n) {
        current_path_ptr = path_multi_set.begin()->second;
        path_multi_set.erase(path_multi_set.begin());

        if ((*current_path_ptr->point_ptrs_.rbegin())->id() == to) {
            current_path_ptr->edge_ptrs_.erase(current_path_ptr->edge_ptrs_.begin());
            pathes.push_back(current_path_ptr);
        } else {

            std::vector< std::pair< Graph::PointPtr, Graph::EdgePtr> > new_points_edges;

            new_points_edges =
                    getNextPoint(*current_path_ptr->edge_ptrs_.rbegin(), *current_path_ptr->point_ptrs_.rbegin());

            for (auto point_egde_pair : new_points_edges) {

                if (edge_selected_counter_[point_egde_pair.second->id()] > n) {
                    continue;
                }

                bool loop = false;

                // check for loops
                for (EdgePtr edge_ptr : current_path_ptr->edge_ptrs_) {

                    if (edge_ptr->id() == point_egde_pair.second->id()) {
                        loop = true;
                        break;
                    }
                }

                if (loop == false) {

                    std::shared_ptr<Path> tmp_path_ptr;
                    tmp_path_ptr.reset(new Path);
                    tmp_path_ptr->edge_ptrs_ = current_path_ptr->edge_ptrs_;
                    tmp_path_ptr->point_ptrs_ = current_path_ptr->point_ptrs_;
                    tmp_path_ptr->value = current_path_ptr->value +
                            computeWeight(point_egde_pair.second);

                    tmp_path_ptr->edge_ptrs_.push_back(point_egde_pair.second);
                    tmp_path_ptr->point_ptrs_.push_back(point_egde_pair.first);

                    path_multi_set.insert(std::make_pair(tmp_path_ptr->value, tmp_path_ptr));
                    edge_selected_counter_[point_egde_pair.second->id()]++;
                }
            }
        }


    }

if(pathes.size()==0){//in case if no path found add first and the last points for visualization

    current_path_ptr.reset(new Path);
    current_path_ptr->point_ptrs_.push_back(points_->at(from));
    current_path_ptr->point_ptrs_.push_back(points_->at(to));
    pathes.push_back(current_path_ptr);
}

    return pathes;
}

template<class T>
std::vector< std::pair< typename Graph<T>::PointPtr, typename Graph<T>::EdgePtr> > Graph<T>::getNextPoint(EdgePtr last_edge, PointPtr current_point) {

    std::vector< std::pair< PointPtr, EdgePtr >  > next_points_via_edge;
//typename EdgeSet::iterator
    for ( auto it = current_point->edges().begin(); it != current_point->edges().end(); it++) {
        if (last_edge.use_count() == 0 || last_edge->id() != it->first) {
            if (it->second->from()->id() != current_point->id()) {
                next_points_via_edge.push_back(std::make_pair(it->second->from(), it->second));
            } else {

                next_points_via_edge.push_back(std::make_pair(it->second->to(), it->second));
            }
        }
    }

    return next_points_via_edge;
}

//template<class T>
//double Graph<T>::computeWeight(EdgePtr last_edge_ptr, VertexPtr via_vertex, EdgePtr next_edge_ptr) {
//    return 1;
//}


template<class T>
double Graph<T>::computeWeight(EdgePtr next_edge_ptr) {

    return next_edge_ptr->weight();

}

