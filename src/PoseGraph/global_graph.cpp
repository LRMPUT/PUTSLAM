/** @file globalGraph.h
 *
 * Global Graph
 * \author Dominik Belter
 */

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
 * adds a vertex to the graph - x,y,theta
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::addVertexSE2(const VertexSE2& v){
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
 * Adds an 3D reproj edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdge3DReproj(const Edge3DReproj& e){
    return true;
}

/**
 * Adds an SE2 edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdgeSE2(const EdgeSE2& e){
    return true;
}

/**
 * update a vertex of the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::updateVertex(const putslam::VertexSE3& v){

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

/// Load graph from file
void GlobalGraph::load(const std::string filename){

}

/// Export camera path to file (RGB-D SLAM format)
void GlobalGraph::export2RGBDSLAM(const std::string filename) const{

}

/// Import camera path from file (RGB-D SLAM format)
bool GlobalGraph::importRGBDSLAM(const std::string filename){
    return false;
}

/// Optimize graph
bool GlobalGraph::optimize(int_fast32_t maxIterations, int verbose, double minimalChi2Ratio) {
    std::cout << "start global graph optimization (t = 0s)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "finish global graph optimization (t = 0.2s)\n";
}

/// Removes weak edes (with error bigger than threshold
bool GlobalGraph::optimizeAndPrune(float_type threshold, unsigned int singleIteration, int verbose){

}

/// Removes weak edes (with error bigger than threshold
bool GlobalGraph::optimizeAndPrune2(float_type threshold, unsigned int singleIteration, int verbose){

}

/// Returns set of graph vertices
PoseGraph::VertexSet GlobalGraph::getVertices(void){

}

/// Returns set of graph edges
PoseGraph::EdgeSet GlobalGraph::getEdges(void){

}

/// find all neighboring vertices for which distance is smaller than threshold
bool GlobalGraph::findNearestNeighbors(int vertexId, int depth, std::vector<int>& neighborsIds){

}

/// marginalize measurements (pose-feature)
bool GlobalGraph::marginalize(const std::vector<int>& keyframes, const std::set<int>& features2remove){

}
