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
    std::cout << v->vertexId << "\n";
    throw std::runtime_error("Remove vertex not implemented\n");
    return true;
}

/// removes an edge from the graph. Returns true on success
bool GlobalGraph::removeEdge(Edge* e){
    std::cout << e->id << "\n";
    throw std::runtime_error("Remove edge not implemented\n");
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
    std::cout << v.vertexId << "\n";
    throw std::runtime_error("Add vertex feature not implemented\n");
    return true;
}

/**
 * adds a vertex to the graph - pose
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::addVertexPose(const VertexSE3& v){
    std::cout << v.vertexId << "\n";
    throw std::runtime_error("Add vertex pose not implemented\n");
    return true;
}

/**
 * Adds an SE3 edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdgeSE3(const EdgeSE3& e){
    std::cout << e.id << "\n";
    throw std::runtime_error("Add edge se3 not implemented\n");
    return true;
}

/**
 * adds a vertex to the graph - x,y,theta
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::addVertexSE2(const VertexSE2& v){
    std::cout << v.vertexId << "\n";
    throw std::runtime_error("Add v se2 not implemented\n");
    return true;
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdge3D(const Edge3D& e){
    std::cout << e.id << "\n";
    throw std::runtime_error("Add edge 3d not implemented\n");
    return true;
}

/**
 * Adds an 3D reproj edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdge3DReproj(const Edge3DReproj& e){
    std::cout << e.id << "\n";
    throw std::runtime_error("Add edge reproj not implemented\n");
    return true;
}

/**
 * Adds an SE2 edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool GlobalGraph::addEdgeSE2(const EdgeSE2& e){
    std::cout << e.id << "\n";
    throw std::runtime_error("Add edge not implemented\n");
    return true;
}

/**
 * update a vertex of the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::updateVertex(const putslam::VertexSE3& v){
    std::cout << v.vertexId << "\n";
    throw std::runtime_error("Update vertex not implemented\n");
	return true;
}

/**
 * update graph: adds vertices and edges to the graph.
 * returns true, on success, or false on failure.
 */
bool GlobalGraph::updateGraph(const VertexSE3& v){
    std::cout << v.vertexId << "\n";
    throw std::runtime_error("Update graph not implemented\n");
    return true;
}

/// Return trajectory (set of SE3 poses)
std::vector<Mat34> GlobalGraph::getTrajectory(void) const{
	return std::vector<Mat34>();
}

/// Save graph to file
void GlobalGraph::save2file(const std::string filename) const {
    std::cout << filename << "\n";
    throw std::runtime_error("Save not implemented\n");
}

/// Load graph from file
void GlobalGraph::load(const std::string filename){
    std::cout << filename << "\n";
    throw std::runtime_error("Load not implemented\n");
}

/// Export camera path to file (RGB-D SLAM format)
void GlobalGraph::export2RGBDSLAM(const std::string filename) const{
    std::cout << filename << "\n";
    throw std::runtime_error("Export not implemented\n");
}

/// Import camera path from file (RGB-D SLAM format)
bool GlobalGraph::importRGBDSLAM(const std::string filename){
    std::cout << filename << "\n";
    throw std::runtime_error("Import RGBD SLAM not implemented\n");
    return false;
}

/// Optimize graph
bool GlobalGraph::optimize(int_fast32_t maxIterations, int verbose, double minimalChi2Ratio) {
    std::cout << maxIterations << verbose << minimalChi2Ratio;
    throw std::runtime_error("Optimize not implemented\n");
    return true;
}

/// Removes weak edes (with error bigger than threshold
bool GlobalGraph::optimizeAndPrune(double threshold, unsigned int singleIteration, int verbose){
    std::cout << threshold << singleIteration << verbose << "\n";
    throw std::runtime_error("Opt and prune not implemented\n");
	return true;
}

/// Removes weak edes (with error bigger than threshold
bool GlobalGraph::optimizeAndPrune2(double threshold, unsigned int singleIteration, int verbose){
    std::cout << threshold << singleIteration << verbose << "\n";
    throw std::runtime_error("Oopt and prune not implemented\n");
	return true;
}

/// Returns set of graph vertices
PoseGraph::VertexSet GlobalGraph::getVertices(void){
	return PoseGraph::VertexSet();
}

/// Returns set of graph edges
PoseGraph::EdgeSet GlobalGraph::getEdges(void){
	return PoseGraph::EdgeSet();
}

/// find all neighboring vertices for which distance is smaller than threshold
bool GlobalGraph::findNearestNeighbors(int vertexId, int depth, std::vector<int>& neighborsIds){
    std::cout << vertexId << depth << neighborsIds.size() << "\n";
    throw std::runtime_error("Find nearest neighbours not implemented\n");
	return true;
}

/// marginalize measurements (pose-feature)
bool GlobalGraph::marginalize(const std::vector<int>& keyframes, const std::set<int>& features2remove){
    std::cout << keyframes.size() << features2remove.size() << "\n";
    throw std::runtime_error("Marginalize not implemented\n");
	return true;
}

/// Fix vertex
void GlobalGraph::fixVertex(int vertexId){
    std::cout << vertexId << "\n";
    throw std::runtime_error("Fix vertex not implemented\n");
}

/// unFix vertex
void GlobalGraph::unfixVertex(int vertexId){
    std::cout << vertexId << "\n";
    throw std::runtime_error("unifix vertex not implemented\n");
}
