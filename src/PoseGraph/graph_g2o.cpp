#include "../include/PoseGraph/graph_g2o.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Pose Graph g2o
PoseGraphG2O::Ptr graph_g2o;

putslam::Graph* putslam::createPoseGraphG2O(void) {
    graph_g2o.reset(new PoseGraphG2O());
    return graph_g2o.get();
}

PoseGraphG2O::PoseGraphG2O(void) : Graph("Pose Graph g2o") {
}

const std::string& PoseGraphG2O::getName() const {
    return name;
}

/// Removes a vertex from the graph. Returns true on success
bool PoseGraphG2O::removeVertex(Vertex* v){
    return true;
}

/// removes an edge from the graph. Returns true on success
bool PoseGraphG2O::removeEdge(Edge* e){
    return true;
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
bool PoseGraphG2O::addVertex(Vertex& v){
    return true;
}

/**
 * Adds an edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(Edge& e){
    return true;
}

/**
 * update graph: adds a vertexes and edges to the graph.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::updateGraph(const Vertex7D& v){
    std::cout << "update local graph \n";
    return true;
}

/// Optimize graph
void PoseGraphG2O::optimize(void) {
    std::cout << "start local graph optimization (t = 0s)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    std::cout << "finish local graph optimization (t = 0.7s)\n";
}
