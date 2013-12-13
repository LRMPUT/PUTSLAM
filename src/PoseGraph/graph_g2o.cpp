#include "../include/PoseGraph/graph_g2o.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Pose Graph g2o
PoseGraphG2O::Ptr graph_g2o;

PoseGraphG2O::PoseGraphG2O(void) : Graph("Pose Graph g2o") {
    std::cout << this->name;
}

const std::string& PoseGraphG2O::getName() const {
    return name;
}

putslam::Graph* putslam::createPoseGraphG2O(void) {
    graph_g2o.reset(new PoseGraphG2O());
    return graph_g2o.get();
}

/// Removes a vertex from the graph. Returns true on success
bool PoseGraphG2O::removeVertex(Vertex* v){

}

/// removes an edge from the graph. Returns true on success
bool PoseGraphG2O::removeEdge(Edge* e){

}

/// clears the graph and empties all structures.
void PoseGraphG2O::clear(){

}

/// @returns the map <i>id -> vertex</i> where the vertices are stored
const PoseGraph::VertexSet& PoseGraphG2O::vertices() const{

}

/// @returns the set of edges of the hyper graph
const PoseGraph::EdgeSet& PoseGraphG2O::edges() const{

}

/**
 * adds a vertex to the graph.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertex(Vertex* v){

}

/**
 * Adds an edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(Edge* e){

}
