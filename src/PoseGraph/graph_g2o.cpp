#include "../include/PoseGraph/graph_g2o.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

/// A single instance of Pose Graph g2o
PoseGraphG2O::Ptr graph_g2o;

putslam::Graph* putslam::createPoseGraphG2O(void) {
    graph_g2o.reset(new PoseGraphG2O());
    return graph_g2o.get();
}

PoseGraphG2O::PoseGraphG2O(void) : Graph("Pose Graph g2o") {
    // create the linear solver
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();

    // create the block solver on top of the linear solver
    blockSolver = new g2o::BlockSolverX(linearSolver);

    // create the algorithm to carry out the optimization
    //OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
    optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    factory = g2o::Factory::instance();
}

/// Destructor
PoseGraphG2O::~PoseGraphG2O(void){
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
	return graph.vertices;
}

/// @returns the set of edges of the hyper graph
const PoseGraph::EdgeSet& PoseGraphG2O::edges() const{
	return graph.edges;
}

/**
 * adds a vertex to the graph - feature.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertexFeature(const Vertex3D& v){
    if (graph.vertices.size() > v.vertex_id){//wrong id
        return false;
    }
    else {//add vertex
        graph.vertices.push_back(v);//update putslam structure
        g2o::OptimizableGraph::Vertex* vert = static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(v.vertex_id));
        g2o::HyperGraph::GraphElemBitset elemBitset;
        elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
        elemBitset.flip();
        if (graph.vertices.size()==0){
            vert->setFixed(true);
        }
        g2o::HyperGraph::HyperGraphElement* element = factory->construct("VERTEX_SE3:QUAT", elemBitset);
//         if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(element)) {
             //std::cout << "// it's a vertex type\n";
//         }
         g2o::OptimizableGraph::Vertex* vse3 = static_cast<g2o::OptimizableGraph::Vertex*>(element);
        //g2o::OptimizableGraph::Vertex* v3d = static_cast<g2o::OptimizableGraph::Vertex*>(element);
        std::stringstream currentLine;
        currentLine << v.keypoint.depthFeature.x() << ' ' << v.keypoint.depthFeature.y() << ' ' << v.keypoint.depthFeature.z() << ' ' << 0 << ' ' << 0 << ' ' << 0 << ' ' << 1;
        vse3->read(currentLine);
        vse3->setId(v.vertex_id);
        if (!optimizer.addVertex(vse3)) {
          std::cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex\n";
        }
        delete vert;
        return true;
    }
}

/**
 * adds a vertex to the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertexPose(const VertexSE3& v){
    if (graph.vertices.size() > v.vertex_id){//wrong id
        return false;
    }
    else {//add vertex
        graph.vertices.push_back(v);//update putslam structure
        g2o::OptimizableGraph::Vertex* vert = static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(v.vertex_id));
        g2o::HyperGraph::GraphElemBitset elemBitset;
        elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
        elemBitset.flip();
        if (graph.vertices.size()==0){
            vert->setFixed(true);
        }
        g2o::HyperGraph::HyperGraphElement* element = factory->construct("VERTEX_SE3:QUAT", elemBitset);
//         if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(element)) {
             //std::cout << "// it's a vertex type\n";
//         }
         g2o::OptimizableGraph::Vertex* vse3 = static_cast<g2o::OptimizableGraph::Vertex*>(element);
        //g2o::OptimizableGraph::Vertex* v3d = static_cast<g2o::OptimizableGraph::Vertex*>(element);
        std::stringstream currentLine;
        currentLine << v.nodeSE3.pos.x() << ' ' << v.nodeSE3.pos.y() << ' ' << v.nodeSE3.pos.z() << ' ' << v.nodeSE3.rot.x() << ' ' << v.nodeSE3.rot.y() << ' ' << v.nodeSE3.rot.z() << ' ' << v.nodeSE3.rot.w();
        vse3->read(currentLine);
        vse3->setId(v.vertex_id);
        if (!optimizer.addVertex(vse3)) {
          std::cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex\n";
        }
        return true;
    }
}

/**
 * Adds an SE3 edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdgeSE3(const EdgeSE3& e){
    if ((graph.vertices.size() < e.toVertexId)||(graph.vertices.size() < e.fromVertexId)){
        return false;
    }
    else {
        graph.edges.push_back(e);
        g2o::HyperGraph::GraphElemBitset elemBitset;
        elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
        elemBitset.flip();
        HyperGraph::HyperGraphElement* element = factory->construct("EDGE_SE3:QUAT", elemBitset);
        //if (dynamic_cast<Edge*>(element)) {
        //      cerr << "it is an edge" << endl;
        //}
        g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(element);
        g2o::OptimizableGraph::Vertex* from = optimizer.vertex(e.fromVertexId);
        g2o::OptimizableGraph::Vertex* to = optimizer.vertex(e.toVertexId);
        edge->setVertex(0, from);
        edge->setVertex(1, to);
        std::stringstream currentLine;
        currentLine << e.trans.pos.x() << ' ' << e.trans.pos.y() << ' ' << e.trans.pos.z()
                    << ' ' << e.trans.rot.x() << ' ' << e.trans.rot.y() << ' ' << e.trans.rot.z() << ' ' << e.trans.rot.w()
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2) << ' ' << e.info(0,3) << ' ' << e.info(0,4) << ' ' << e.info(0,5)
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2) << ' ' << e.info(1,3) << ' ' << e.info(1,4) << ' ' << e.info(1,5)
                    << ' ' << e.info(2,2) << ' ' << e.info(2,3) << ' ' << e.info(2,4) << ' ' << e.info(2,5)
                    << ' ' << e.info(3,3) << ' ' << e.info(3,4) << ' ' << e.info(3,5)
                    << ' ' << e.info(4,4) << ' ' << e.info(4,5)
                    << ' ' << e.info(5,5);
        edge->read(currentLine);
        if (!optimizer.addEdge(edge)) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to add edge \n";
            delete edge;
        }
        return true;
    }
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge3D(const Edge3D& e){
    if ((graph.vertices.size() < e.toVertexId)||(graph.vertices.size() < e.fromVertexId)){
        return false;
    }
    else {
        graph.edges.push_back(e);
        g2o::HyperGraph::GraphElemBitset elemBitset;
        elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
        elemBitset.flip();
        HyperGraph::HyperGraphElement* element = factory->construct("EDGE_SE3:QUAT", elemBitset);
        if (dynamic_cast<Edge*>(element)) {
              cerr << "it is an edge" << endl;
        }
        g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(element);
        g2o::OptimizableGraph::Vertex* from = optimizer.vertex(e.fromVertexId);
        g2o::OptimizableGraph::Vertex* to = optimizer.vertex(e.toVertexId);
        edge->setVertex(0, from);
        edge->setVertex(1, to);
        std::stringstream currentLine;
        currentLine << e.trans.x() << ' ' << e.trans.y() << ' ' << e.trans.z()
                    << ' ' << 0 << ' ' << 0 << ' ' << 0 << ' ' << 1
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2) << ' ' << 0 << ' ' << 0 << ' ' << 0
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2) << ' ' << 0 << ' ' << 0 << ' ' << 0
                    << ' ' << e.info(2,2) << ' ' << 0 << ' ' << 0 << ' ' << 0
                    << ' ' << 50 << ' ' << 0 << ' ' << 0
                    << ' ' << 50 << ' ' << 0
                    << ' ' << 50;
        edge->read(currentLine);
        if (!optimizer.addEdge(edge)) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to add edge \n";
            delete edge;
        }
        return true;
    }
}

/**
 * update graph: adds vertices and edges to the graph.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::updateGraph(const VertexSE3& v){
    std::cout << "update local graph \n";
    return true;
}

/// Save graph to file
void PoseGraphG2O::save2file(std::string filename) const{
    optimizer.save(filename.c_str());
}

/// Optimize graph
void PoseGraphG2O::optimize(uint_fast32_t maxIterations) {
    std::cout << "start local graph optimization (t = 0s)\n";
    auto start = std::chrono::system_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
    std::cout << "finish local graph optimization (t = " << elapsed.count() << "ms)\n";
}
