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
 * adds a vertex to the graph feature
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::addVertexFeature(const Vertex3D& v){
    return true;
}

/**
 * adds a vertex to the graph - pose
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::addVertexPose(const VertexSE3& v){
    return true;
}

/**
 * Adds an SE3 edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdgeSE3(const EdgeSE3& e){
    return true;
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdge3D(const Edge3D& e){
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

/// Return trajectory (set of SE3 poses)
std::vector<Mat34> GlobalGraph::getTrajectory(void) const{

}

/// Save graph to file
void GlobalGraph::save2file(const std::string filename) const {

}

/// Export camera path to file (RGB-D SLAM format)
void GlobalGraph::export2RGBDSLAM(const std::string filename) const{

}

/// Import camera path from file (RGB-D SLAM format)
bool GlobalGraph::importRGBDSLAM(const std::string filename){
    return false;
}

/// Optimize graph
bool GlobalGraph::optimize(uint_fast32_t maxIterations) {
    std::cout << "start global graph optimization (t = 0s)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "finish global graph optimization (t = 0.2s)\n";
}

/// Removes weak edes (with error bigger than threshold
bool GlobalGraph::optimizeAndPrune(float_type threshold, unsigned int singleIteration){

}

/// Removes weak edes (with error bigger than threshold
bool GlobalGraph::optimizeAndPrune2(float_type threshold, unsigned int singleIteration){

}
