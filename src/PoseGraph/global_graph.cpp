#include "../include/PoseGraph/global_graph.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Global Graph
GlobalGraph::Ptr global_graph;


putslam::Graph* putslam::createGlobalGraph(void) {
    global_graph.reset(new GlobalGraph());
    return global_graph.get();
}

GlobalGraph::GlobalGraph(void) : Graph("Pose Graph g2o") {
}

const std::string& GlobalGraph::getName() const {
    return name;
}

/// Removes a vertex from the graph. Returns true on success
bool GlobalGraph::removeVertex(Vertex* v){
    return true;
}

/// removes an edge from the graph. Returns true on success
bool GlobalGraph::removeEdge(Edge* e){
    return true;
}

/// clears the graph and empties all structures.
void GlobalGraph::clear(){

}

/// @returns the map <i>id -> vertex</i> where the vertices are stored
const PoseGraph::VertexSet& GlobalGraph::vertices() const{
	return graph.vertices;
}

/// @returns the set of edges of the hyper graph
const PoseGraph::EdgeSet& GlobalGraph::edges() const{
	return graph.edges;
}

/**
 * adds a vertex to the graph.
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::addVertex(Vertex& v){
    return true;
}

/**
 * Adds an edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdge(Edge& e){
    return true;
}

/**
 * update graph: adds vertices and edges to the graph.
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::updateGraph(const VertexSE3& v){
    std::cout << "update global graph \n";
    return true;
}

/// Optimize graph
void GlobalGraph::optimize(void) {
    std::cout << "start global graph optimization (t = 0s)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "finish global graph optimization (t = 0.2s)\n";
}
