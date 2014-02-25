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
    mtxGraph.lock();
    optimizer.clear();
    graph.edges.clear();
    graph.vertices.clear();
    mtxGraph.unlock();
}

/// @returns the map <i>id -> vertex</i> where the vertices are stored
const PoseGraph::VertexSet& PoseGraphG2O::vertices() const{
	return graph.vertices;
}

/// @returns the set of edges of the hyper graph
const PoseGraph::EdgeSet& PoseGraphG2O::edges() const{
	return graph.edges;
}

/// add vertex to g2o interface
bool PoseGraphG2O::addVertexG2O(uint_fast32_t id, std::stringstream& vertex){
    g2o::OptimizableGraph::Vertex* vert = static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
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
    vse3->read(vertex);
    vse3->setId(id);
    if (!optimizer.addVertex(vse3)) {
      std::cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex\n";
    }
}

/**
 * adds a vertex to the graph - feature.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertexFeature(const Vertex3D& v){
    mtxBuffGraph.lock();
    bufferGraph.vertices.push_back(std::unique_ptr<Vertex>(new Vertex3D(v)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/**
 * adds a vertex to the graph - feature.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertex(const Vertex3D& v){
    mtxGraph.lock();
    if (graph.vertices.size() > v.vertexId){//wrong id
        mtxGraph.unlock();
        return false;
    }
    else {//add vertex
        graph.vertices.push_back(std::unique_ptr<Vertex>(new Vertex3D(v)));//update putslam structure
        std::stringstream currentLine;
        currentLine << v.keypoint.depthFeature.x() << ' ' << v.keypoint.depthFeature.y() << ' ' << v.keypoint.depthFeature.z() << ' ' << 0 << ' ' << 0 << ' ' << 0 << ' ' << 1;
        addVertexG2O(v.vertexId, currentLine);
        mtxGraph.unlock();
        return true;
    }
}

/**
 * adds a vertex to the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertexPose(const VertexSE3& v){
    mtxBuffGraph.lock();
    bufferGraph.vertices.push_back(std::unique_ptr<Vertex>(new VertexSE3(v)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/**
 * adds a vertex to the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertex(const VertexSE3& v){
    mtxGraph.lock();
    if (graph.vertices.size() > v.vertexId){//wrong id
        mtxGraph.unlock();
        return false;
    }
    else {//add vertex
        graph.vertices.push_back(std::unique_ptr<Vertex>(new VertexSE3(v)));//update putslam structure        
        std::stringstream currentLine;
        currentLine << v.nodeSE3.pos.x() << ' ' << v.nodeSE3.pos.y() << ' ' << v.nodeSE3.pos.z() << ' ' << v.nodeSE3.rot.x() << ' ' << v.nodeSE3.rot.y() << ' ' << v.nodeSE3.rot.z() << ' ' << v.nodeSE3.rot.w();
        addVertexG2O(v.vertexId, currentLine);
        mtxGraph.unlock();
        return true;
    }
}

/// add edge to g2o interface
bool PoseGraphG2O::addEdgeG2O(uint_fast32_t fromId, uint_fast32_t toId, std::stringstream& edgeStream){
    g2o::HyperGraph::GraphElemBitset elemBitset;
    elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
    elemBitset.flip();
    HyperGraph::HyperGraphElement* element = factory->construct("EDGE_SE3:QUAT", elemBitset);
    //if (dynamic_cast<Edge*>(element)) {
    //      cerr << "it is an edge" << endl;
    //}
    g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(element);
    g2o::OptimizableGraph::Vertex* from = optimizer.vertex(fromId);
    g2o::OptimizableGraph::Vertex* to = optimizer.vertex(toId);
    edge->setVertex(0, from);
    edge->setVertex(1, to);
    edge->read(edgeStream);
    if (!optimizer.addEdge(edge)) {
        cerr << __PRETTY_FUNCTION__ << ": Unable to add edge \n";
        delete edge;
    }
}

/**
 * Adds an SE3 edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdgeSE3(const EdgeSE3& e){
    mtxBuffGraph.lock();
    bufferGraph.edges.push_back(std::unique_ptr<Edge>(new EdgeSE3(e)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/**
 * Adds an SE3 edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(const EdgeSE3& e){
    mtxGraph.lock();
    if ((graph.vertices.size() < e.toVertexId)||(graph.vertices.size() < e.fromVertexId)){
        mtxGraph.unlock();
        return false;
    }
    else {
        graph.edges.push_back(std::unique_ptr<Edge>(new EdgeSE3(e)));
        std::stringstream currentLine;
        currentLine << e.trans.pos.x() << ' ' << e.trans.pos.y() << ' ' << e.trans.pos.z()
                    << ' ' << e.trans.rot.x() << ' ' << e.trans.rot.y() << ' ' << e.trans.rot.z() << ' ' << e.trans.rot.w()
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2) << ' ' << e.info(0,3) << ' ' << e.info(0,4) << ' ' << e.info(0,5)
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2) << ' ' << e.info(1,3) << ' ' << e.info(1,4) << ' ' << e.info(1,5)
                    << ' ' << e.info(2,2) << ' ' << e.info(2,3) << ' ' << e.info(2,4) << ' ' << e.info(2,5)
                    << ' ' << e.info(3,3) << ' ' << e.info(3,4) << ' ' << e.info(3,5)
                    << ' ' << e.info(4,4) << ' ' << e.info(4,5)
                    << ' ' << e.info(5,5);
        addEdgeG2O(e.fromVertexId, e.toVertexId, currentLine);
        mtxGraph.unlock();
        return true;
    }
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge3D(const Edge3D& e){
    mtxBuffGraph.lock();
    bufferGraph.edges.push_back(std::unique_ptr<Edge>(new Edge3D(e)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(const Edge3D& e){
    mtxGraph.lock();
    if ((graph.vertices.size() < e.toVertexId)||(graph.vertices.size() < e.fromVertexId)){
        mtxGraph.unlock();
        return false;
    }
    else {
        graph.edges.push_back(std::unique_ptr<Edge>(new Edge3D(e)));
        std::stringstream currentLine;
        currentLine << e.trans.x() << ' ' << e.trans.y() << ' ' << e.trans.z()
                    << ' ' << 0 << ' ' << 0 << ' ' << 0 << ' ' << 1
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2) << ' ' << 0 << ' ' << 0 << ' ' << 0
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2) << ' ' << 0 << ' ' << 0 << ' ' << 0
                    << ' ' << e.info(2,2) << ' ' << 0 << ' ' << 0 << ' ' << 0
                    << ' ' << 0.001 << ' ' << 0 << ' ' << 0
                    << ' ' << 0.001 << ' ' << 0
                    << ' ' << 0.001;
        addEdgeG2O(e.fromVertexId, e.toVertexId, currentLine);
        mtxGraph.unlock();
        return true;
    }
}

/**
 * update graph: adds vertices and edges to the graph.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::updateGraph(void){
    if (mtxGraph.try_lock()){//try to lock graph
        //copy buffer graph
        mtxBuffGraph.lock();
        PoseGraph tmpGraph;
        for (putslam::PoseGraph::EdgeSet::iterator it = bufferGraph.edges.begin(); it!=bufferGraph.edges.end();it++){
            tmpGraph.edges.push_back(std::move(*it));
        }
        for (putslam::PoseGraph::VertexSet::iterator it = bufferGraph.vertices.begin(); it!=bufferGraph.vertices.end();it++){
            tmpGraph.vertices.push_back(std::move(*it));
        }
        bufferGraph.edges.clear(); bufferGraph.vertices.clear();
        mtxBuffGraph.unlock();
        for (putslam::PoseGraph::VertexSet::iterator it = tmpGraph.vertices.begin(); it!=tmpGraph.vertices.end();it++){
            if ((*it).get()->type==Vertex::VERTEX3D){
                addVertex(*(Vertex3D*)it->get());
            }
            else if (it->get()->type==Vertex::VERTEXSE3){
                addVertex(*(VertexSE3*)it->get());
            }
        }
        for (putslam::PoseGraph::EdgeSet::iterator it = tmpGraph.edges.begin(); it!=tmpGraph.edges.end();it++){
            if (it->get()->type==Edge::EDGE_3D){
                addEdge(*(Edge3D*)it->get());
            }
            else if (it->get()->type==Edge::EDGE_SE3){
                addEdge(*(EdgeSE3*)it->get());
            }
        }
        //merge buffer and pose graph
        mtxGraph.unlock();
    }
    return true;
}

/// Save graph to file
void PoseGraphG2O::save2file(std::string filename) const{
    optimizer.save(filename.c_str());
}

/// Export camera path to file (RGB-D SLAM format)
void PoseGraphG2O::export2RGBDSLAM(std::string filename) const{
    ofstream file(filename);
    float_type timestamp;
    for (putslam::PoseGraph::VertexSet::const_iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
        if (it->get()->type==Vertex::VERTEXSE3){
            file << std::setprecision (numeric_limits<double>::digits10 + 1) << ((VertexSE3*)it->get())->timestamp << " " << std::setprecision (8) << ((VertexSE3*)it->get())->nodeSE3.pos.x() << " " << ((VertexSE3*)it->get())->nodeSE3.pos.y() << " " << ((VertexSE3*)it->get())->nodeSE3.pos.x() << " " << ((VertexSE3*)it->get())->nodeSE3.rot.x() << " " << ((VertexSE3*)it->get())->nodeSE3.rot.y() << " " << ((VertexSE3*)it->get())->nodeSE3.rot.z() << " " << ((VertexSE3*)it->get())->nodeSE3.rot.w() << std::endl;
        }
    }
    file.close();
}

/// Optimize graph
void PoseGraphG2O::optimize(uint_fast32_t maxIterations) {
    mtxGraph.lock();
    std::cout << "start local graph optimization (t = 0s)\n";
    auto start = std::chrono::system_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);

    //copy optimized graph to putslam dataset
    set<g2o::OptimizableGraph::Vertex*, g2o::OptimizableGraph::VertexIDCompare> verticesToCopy;
    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);
      if (e->level() == 0) {
        for (vector<g2o::HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
          verticesToCopy.insert(static_cast<g2o::OptimizableGraph::Vertex*>(*it));
        }
      }
    }

    int iter = 0;
    for (set<g2o::OptimizableGraph::Vertex*, g2o::OptimizableGraph::VertexIDCompare>::const_iterator it = verticesToCopy.begin(); it != verticesToCopy.end(); ++it){
      OptimizableGraph::Vertex* v = *it;
      std::vector<double> estimate;
      v->getEstimateData(estimate);
      if (graph.vertices[iter].get()->type==Vertex::VERTEX3D){
          ((Vertex3D*)graph.vertices[iter].get())->keypoint.depthFeature.x() = estimate[0];
          ((Vertex3D*)graph.vertices[iter].get())->keypoint.depthFeature.y() = estimate[1];
          ((Vertex3D*)graph.vertices[iter].get())->keypoint.depthFeature.z() = estimate[2];
      }
      else if (graph.vertices[iter].get()->type==Vertex::VERTEXSE3){
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.pos.x() = estimate[0];
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.pos.y() = estimate[1];
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.pos.z() = estimate[2];
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.x() = estimate[3];
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.y() = estimate[4];
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.z() = estimate[5];
          ((VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.w() = estimate[6];
      }
      iter++;
    }
    updateGraph();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
    std::cout << "finish local graph optimization (t = " << elapsed.count() << "ms)\n";
    mtxGraph.unlock();
}
