/** @file graph_g2o.h
 *
 * g2o graph
 * \author Dominik Belter
 */

#include "PoseGraph/graph_g2o.h"
#include <stdexcept>
#include <chrono>

#include "g2o/types/slam3d/parameter_camera.h"

using namespace putslam;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

/// A single instance of Pose Graph g2o
PoseGraphG2O::Ptr graph_g2o;

putslam::Graph* putslam::createPoseGraphG2O(void) {
    graph_g2o.reset(new PoseGraphG2O());
    return graph_g2o.get();
}

putslam::Graph* putslam::createPoseGraphG2O(Mat34 cameraPose) {
    graph_g2o.reset(new PoseGraphG2O(cameraPose));
    return graph_g2o.get();
}

PoseGraphG2O::PoseGraphG2O(void) : Graph("Pose Graph g2o") {
    // create the linear solver
    //linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();

    // create the block solver on top of the linear solver
    //blockSolver = new g2o::BlockSolverX(linearSolver);
    blockSolver = new g2o::BlockSolverX(linearSolver);

    // create the algorithm to carry out the optimization
    g2o::OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    //g2o::OptimizationAlgorithmWithHessian* optimizationAlgorithm = new g2o::OptimizationAlgorithmWithHessian(blockSolver);
    //g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    factory = g2o::Factory::instance();

    cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;  R  << 1,  0,  0,  0,  1,  0,  0,  0,  1;
    //Quaternion q(-0.5, 0.5, -0.5, 0.5);
    cameraPose= R; cameraPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    cameraOffset->setOffset(cameraPose);
    optimizer.addParameter(cameraOffset);

//	  TODO: Uncomment if it is working
//    g2o::ParameterCamera *pc = new g2o::ParameterCamera();
//    pc->setKcam(525.0, 525.0, 320.0, 240.0);
//
//    optimizer.addParameter(pc);
}

PoseGraphG2O::PoseGraphG2O(Mat34& cameraPose) : PoseGraphG2O() {
    Eigen::Isometry3d camPos;
    camPos = cameraPose.matrix();
    camPos.translation() = Eigen::Vector3d (cameraPose(0,3), cameraPose(1,3), cameraPose(2,3));
    cameraOffset->setOffset(camPos);    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);
}

/// Destructor
PoseGraphG2O::~PoseGraphG2O(void){
    //delete cameraOffset;
}

const std::string& PoseGraphG2O::getName() const {
    return name;
}

/// removes vertex from the g2o graph. Returns true on success
bool PoseGraphG2O::removeVertexG2O(unsigned int id){
    g2o::OptimizableGraph::VertexContainer vertices = optimizer.activeVertices();
    for (g2o::OptimizableGraph::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        if ((unsigned int)(*it)->id()==id){
            optimizer.removeVertex(*it);
            return true;
        }
    }
    return false;
}

/// Removes a vertex from the graph. Returns true on success
PoseGraph::VertexSet::iterator PoseGraphG2O::removeVertex(unsigned int id){
    mtxGraph.lock();
    PoseGraph::VertexSet::iterator v = findVertex(id);
    if (v != graph.vertices.end()) {
        v = graph.vertices.erase(v);
    }
    return v;
    mtxGraph.unlock();
}

/// removes an edge from the g2o graph. Returns true on success
bool PoseGraphG2O::removeEdgeG2O(unsigned int id){
    mtxGraph.lock();
    g2o::OptimizableGraph::EdgeContainer edges = optimizer.activeEdges();
    for (g2o::OptimizableGraph::EdgeContainer::iterator it = edges.begin(); it!=edges.end(); it++){
        if ((unsigned int)(*it)->id()==id){
            optimizer.removeEdge(*it);
            mtxGraph.unlock();
            return true;
        }
    }
    mtxGraph.unlock();
    return false;
}

/// removes an edge from the graph. Returns true on success
PoseGraph::EdgeSet::iterator PoseGraphG2O::removeEdge(unsigned int id){
    mtxGraph.lock();
    PoseGraph::EdgeSet::iterator e = findEdge(id);
    if (e != graph.edges.end()) {
        graph.prunedEdges.push_back(std::move(*e));
        e = graph.edges.erase(e);
        mtxGraph.unlock();
        return e;
    }
    mtxGraph.unlock();
    return graph.edges.end();
}

/// clears the graph and empties all structures.
void PoseGraphG2O::clear(){
    mtxGraph.lock();
    mtxBuffGraph.lock();
    optimizer.clear();
    graph.edges.clear();
    graph.vertices.clear();
    graph.prunedEdges.clear();
    bufferGraph.vertices.clear();
    bufferGraph.edges.clear();
    bufferGraph.prunedEdges.clear();
    mtxBuffGraph.unlock();
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
bool PoseGraphG2O::addVertexG2O(uint_fast32_t id, std::stringstream& vertex, Vertex::Type type){
	// g2o::OptimizableGraph::Vertex* vert = static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
    g2o::HyperGraph::GraphElemBitset elemBitset;
    elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
    elemBitset.flip();

    g2o::HyperGraph::HyperGraphElement* element;
    if (type==Vertex::VERTEXSE3)
        element = factory->construct("VERTEX_SE3:QUAT", elemBitset);
    else if (type==Vertex::VERTEX3D)
        element = factory->construct("VERTEX_TRACKXYZ", elemBitset);
    else if (type==Vertex::VERTEXSE2)
        element = factory->construct("VERTEX_SE2", elemBitset);
    else {
        std::cout << "error: unknown vertex type!\n";
        return false;
    }
    g2o::OptimizableGraph::Vertex* vert = static_cast<g2o::OptimizableGraph::Vertex*>(element);
    vert->read(vertex);
    vert->setId((int)id);


    if (graph.vertices.size()==1){
    	vert->setFixed(true);
    }
    newVertices.insert(vert);


    if (!optimizer.addVertex(vert)) {
      std::cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex\n";
    }

    return true; //TODO: DB check that
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
    //add vertex
    if (findVertex((unsigned int)v.vertexId)==graph.vertices.end()){// to vertex does not exist
        //std::cout << "add feature\n";
        graph.vertices.push_back(std::unique_ptr<Vertex>(new Vertex3D(v)));//update putslam structure
        std::stringstream currentLine;
        currentLine << v.keypoint.depthFeature.x() << ' ' << v.keypoint.depthFeature.y() << ' ' << v.keypoint.depthFeature.z();
        addVertexG2O(v.vertexId, currentLine, Vertex::VERTEX3D);
        mtxGraph.unlock();
        return true;
    }
    else {
        mtxGraph.unlock();
        return true;
    }
}

/**
 * adds a vertex to the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertexPose(const putslam::VertexSE3& v){
    mtxBuffGraph.lock();
    bufferGraph.vertices.push_back(std::unique_ptr<Vertex>(new putslam::VertexSE3(v)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/**
 * adds a vertex to the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertex(const putslam::VertexSE3& v){
    mtxGraph.lock();
    //add vertex
    if (findVertex((unsigned int)v.vertexId)==graph.vertices.end()){// to vertex does not exist
        //std::cout << "add pose\n";
        graph.vertices.push_back(std::unique_ptr<Vertex>(new putslam::VertexSE3(v)));//update putslam structure
        std::stringstream currentLine;
        Quaternion quat(v.pose.rotation());
        currentLine << v.pose(0,3) << ' ' << v.pose(1,3) << ' ' << v.pose(2,3) << ' ' << quat.x() << ' ' << quat.y() << ' ' << quat.z() << ' ' << quat.w();
        addVertexG2O(v.vertexId, currentLine, Vertex::VERTEXSE3);
        mtxGraph.unlock();
        return true;
    }
    else {
        mtxGraph.unlock();
        return false;
    }
}

/**
 * update a vertex of the graph - robot pose
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::updateVertex(const putslam::VertexSE3& v){
    mtxGraph.lock();
    //update vertex
    PoseGraph::VertexSet::iterator vertIt = findVertex((unsigned int)v.vertexId);
    if (vertIt!=graph.vertices.end()){
        ((VertexSE3*)(*vertIt).get())->pose = v.pose;
        ///TODO update g2o graph
        mtxGraph.unlock();
        return true;
    }
    else {
        mtxGraph.unlock();
        return false;
    }
}

/**
 * adds a vertex to the graph - x,y,theta.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertex(const putslam::VertexSE2& v){
    mtxGraph.lock();
    //add vertex
    if (findVertex((unsigned int)v.vertexId)==graph.vertices.end()){// to vertex does not exist
        graph.vertices.push_back(std::unique_ptr<Vertex>(new VertexSE2(v)));//update putslam structure
        std::stringstream currentLine;
        currentLine << v.pos.x() << ' ' << v.pos.y() << ' ' << v.theta;
        addVertexG2O(v.vertexId, currentLine, Vertex::VERTEXSE2);
        mtxGraph.unlock();
        return true;
    }
    else {
        mtxGraph.unlock();
        return false;
    }
}

/**
 * adds a vertex to the graph - x,y,theta
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::addVertexSE2(const putslam::VertexSE2& v){
    mtxBuffGraph.lock();
    bufferGraph.vertices.push_back(std::unique_ptr<Vertex>(new putslam::VertexSE2(v)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/// add edge to g2o interface
bool PoseGraphG2O::addEdgeG2O(uint_fast32_t id, uint_fast32_t fromId, uint_fast32_t toId, std::stringstream& edgeStream, Edge::Type type){
    g2o::HyperGraph::GraphElemBitset elemBitset;
    elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
    elemBitset.flip();
    g2o::HyperGraph::HyperGraphElement* element;
    if (type==Edge::EDGE_SE3)
        element = factory->construct("EDGE_SE3:QUAT", elemBitset);
    else if (type==Edge::EDGE_3D)
        element = factory->construct("EDGE_SE3_TRACKXYZ", elemBitset);
    else if (type==Edge::EDGE_3D_REPROJ) {
    	std::cout<<"Trying to add reprojection!" << std::endl;
    	element = new g2o::EdgeSE3PointXYZReprojectionError();//factory->construct("EDGE_SE3_REPROJECTION", elemBitset);
    	std::cout<<"Trying to add reprojection! - succes!!" << std::endl;
    	if ( !element )
    		std::cout<<"NOT CREATED!" << std::endl;
    }
    else if (type==Edge::EDGE_SE2)
        element = factory->construct("EDGE_SE2", elemBitset);
    else {
        std::cout << "error: unknown edge type!\n";
        return false;
    }
    g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(element);
    g2o::OptimizableGraph::Vertex* from = optimizer.vertex((int)fromId);
    g2o::OptimizableGraph::Vertex* to = optimizer.vertex((int)toId);
    edge->setVertex(0, from);
    edge->setVertex(1, to);
    edge->read(edgeStream);
    edge->setId((int)id);


    //g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
    //rk->setDelta(1);

    //g2o::RobustKernelCauchy* rk1 = edge->robustKernel();
    //rk1->

    if (!optimizer.addEdge(edge)) {
        std::cerr << __PRETTY_FUNCTION__ << ": Unable to add edge \n";
        delete edge;
      //  delete rk;
    }

    return true; //TODO: DB check that
}

/// set Robust Kernel
void PoseGraphG2O::setRobustKernel(std::string name, double delta){
    for (g2o::SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
        g2o::SparseOptimizer::Edge* e = dynamic_cast<g2o::SparseOptimizer::Edge*>(*it);
        g2o::AbstractRobustKernelCreator* creatorRK = g2o::RobustKernelFactory::instance()->creator(name);
        e->setRobustKernel(creatorRK->construct());
        g2o::RobustKernel* rk = e->robustKernel();
        rk->setDelta(delta);
    }
}

/// disable Robust Kernel
void PoseGraphG2O::disableRobustKernel(void){
    for (g2o::SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
        g2o::SparseOptimizer::Edge* e = dynamic_cast<g2o::SparseOptimizer::Edge*>(*it);
        e->setRobustKernel(0);
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
bool PoseGraphG2O::addEdge(EdgeSE3& e){
    mtxGraph.lock();
//    if (findVertex(e.fromVertexId)==graph.vertices.end()){// to-vertex does not exist
//        std::cout << "Warning: vertex SE3 from " << e.fromVertexId << "  does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Mat34 pose(Mat34::Identity());
//        addVertexPose(putslam::VertexSE3(e.fromVertexId, pose));
//        mtxGraph.lock();
//    }
//    if (findVertex(e.toVertexId)==graph.vertices.end()){// to vertex does not exist
//        std::cout << "Warning: vertex SE3 to " << e.toVertexId << " does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Mat34 pose(Mat34::Identity());
//        addVertexPose(putslam::VertexSE3(e.toVertexId, pose));
//        mtxGraph.lock();
//    }
    if (findVertex((unsigned int)e.fromVertexId)!=graph.vertices.end()&&findVertex((unsigned int)e.toVertexId)!=graph.vertices.end()){
        //std::cout << "add edge se3\n";
        e.id = (unsigned int)graph.edges.size();
        graph.edges.push_back(std::unique_ptr<Edge>(new EdgeSE3(e)));
        std::stringstream currentLine;
        Quaternion quat(e.trans.rotation());
        currentLine << e.trans(0,3) << ' ' << e.trans(1,3) << ' ' << e.trans(2,3)
                    << ' ' << quat.x() << ' ' << quat.y() << ' ' << quat.z() << ' ' << quat.w()
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2) << ' ' << e.info(0,3) << ' ' << e.info(0,4) << ' ' << e.info(0,5)
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2) << ' ' << e.info(1,3) << ' ' << e.info(1,4) << ' ' << e.info(1,5)
                    << ' ' << e.info(2,2) << ' ' << e.info(2,3) << ' ' << e.info(2,4) << ' ' << e.info(2,5)
                    << ' ' << e.info(3,3) << ' ' << e.info(3,4) << ' ' << e.info(3,5)
                    << ' ' << e.info(4,4) << ' ' << e.info(4,5)
                    << ' ' << e.info(5,5);
        addEdgeG2O(e.id, e.fromVertexId, e.toVertexId, currentLine, Edge::EDGE_SE3);
        mtxGraph.unlock();
        return true;
    }
    else {
        mtxGraph.unlock();
        return false;
    }
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge3D(const Edge3D& e){
	// TODO: fast hack with e.fromVertexId == 0 - DB correct this!
    mtxBuffGraph.lock();
    bufferGraph.edges.push_back(std::unique_ptr<Edge>(new Edge3D(e)));
    mtxBuffGraph.unlock();
        //std::cout << "add edge update graph\n";
    updateGraph();//try to update graph
    return true;
}

bool PoseGraphG2O::addEdge3DReproj(const Edge3DReproj& e){
    mtxBuffGraph.lock();
    bufferGraph.edges.push_back(std::unique_ptr<Edge>(new Edge3DReproj(e)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
}

/**
 * Adds an 3D edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(Edge3D& e){
    mtxGraph.lock();
//    if (findVertex(e.toVertexId)==graph.vertices.end()){// to vertex does not exist
//        std::cout << "Warning: vertex 3D to " << e.toVertexId << " does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Vec3 pos(0.0, 0.0, 0.0);
//        addVertexFeature(putslam::Vertex3D(e.toVertexId, pos));
//        mtxGraph.lock();
//    }
//    if (findVertex(e.fromVertexId)==graph.vertices.end()){// to vertex does not exist
//        std::cout << "Warning: vertex 3D from " << e.fromVertexId << " does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Mat34 pose(Mat34::Identity());
//        addVertexPose(putslam::VertexSE3(e.fromVertexId, pose));
//        mtxGraph.lock();
//    }
    //std::cout << "try add edge 3d\n";
    if (findVertex((unsigned int)e.toVertexId)!=graph.vertices.end()&&findVertex((unsigned int)e.fromVertexId)!=graph.vertices.end()){
        //std::cout << "add edge 3d\n";
        e.id = (unsigned int)graph.edges.size();
        graph.edges.push_back(std::unique_ptr<Edge>(new Edge3D(e)));
        std::stringstream currentLine;
        currentLine <<  0  << ' ' << e.trans.x() << ' ' << e.trans.y() << ' ' << e.trans.z()
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2)
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2)
                    << ' ' << e.info(2,2);
        addEdgeG2O(e.id, e.fromVertexId, e.toVertexId, currentLine, Edge::EDGE_3D);
        mtxGraph.unlock();
        return true;
    }
    else{
        mtxGraph.unlock();
        return false;
    }
}

/**
 * Adds an 3D reproj edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(Edge3DReproj& e){
    mtxGraph.lock();
//    if (findVertex(e.toVertexId)==graph.vertices.end()){// to vertex does not exist
//        std::cout << "Warning: vertex reproj. to " << e.toVertexId << " does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Vec3 pos(0.0, 0.0, 0.0);
//        addVertexFeature(putslam::Vertex3D(e.toVertexId, pos));
//        mtxGraph.lock();
//    }
//    if (findVertex(e.fromVertexId)==graph.vertices.end()){// to vertex does not exist
//        std::cout << "Warning: vertex reproj. from " << e.fromVertexId << " does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Mat34 pose(Mat34::Identity());
//        addVertexPose(putslam::VertexSE3(e.fromVertexId, pose));
//        mtxGraph.lock();
//    }
    if (findVertex((unsigned int)e.toVertexId)!=graph.vertices.end()&&findVertex((unsigned int)e.fromVertexId)!=graph.vertices.end()){
        e.id = (unsigned int)graph.edges.size();
        graph.edges.push_back(std::unique_ptr<Edge>(new Edge3DReproj(e)));
        std::stringstream currentLine;
        currentLine << e.u << ' ' << e.v << ' ' << e.info(0, 0) << ' '
                << e.info(0, 1) << ' ' << e.info(1, 1);
        addEdgeG2O(e.id, e.fromVertexId, e.toVertexId, currentLine, Edge::EDGE_3D_REPROJ);
        mtxGraph.unlock();
        return true;
    }
    else {
        mtxGraph.unlock();
        return false;
    }
}



/**
 * Adds an SE2 edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(EdgeSE2& e){
    mtxGraph.lock();
//    if (findVertex(e.fromVertexId)==graph.vertices.end()){// to-vertex does not exist
//        std::cout << "Warning: vertex SE2 from " << e.fromVertexId << "  does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Eigen::Vector2d pos(0.0, 0.0); double rot(0);
//        addVertexSE2(putslam::VertexSE2(e.fromVertexId, pos, rot));
//        mtxGraph.lock();
//    }
//    if (findVertex(e.toVertexId)==graph.vertices.end()){// to vertex does not exist
//        std::cout << "Warning: vertex SE2 to " << e.toVertexId << " does not exist. adding new vertex...\n";
//        mtxGraph.unlock();
//        Eigen::Vector2d pos(0.0, 0.0); double rot(0);
//        addVertexSE2(putslam::VertexSE2(e.toVertexId, pos, rot));
//        mtxGraph.lock();
//    }
    if (findVertex((unsigned int)e.fromVertexId)!=graph.vertices.end()&&findVertex((unsigned int)e.toVertexId)==graph.vertices.end()){
        e.id = (unsigned int)graph.edges.size();
        graph.edges.push_back(std::unique_ptr<Edge>(new EdgeSE2(e)));
        std::stringstream currentLine;
        currentLine << e.trans.x() << ' ' << e.trans.y() << ' ' << e.theta
                    << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2)
                    << ' ' << e.info(1,1) << ' ' << e.info(1,2)
                    << ' ' << e.info(2,2);
        addEdgeG2O(e.id, e.fromVertexId, e.toVertexId, currentLine, Edge::EDGE_SE2);
        mtxGraph.unlock();
        return true;
    }
    else{
        mtxGraph.unlock();
        return false;
    }
}

/**
 * Adds an SE2 edge to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdgeSE2(const EdgeSE2& e){
    mtxBuffGraph.lock();
    bufferGraph.edges.push_back(std::unique_ptr<Edge>(new EdgeSE2(e)));
    mtxBuffGraph.unlock();
    updateGraph();//try to update graph
    return true;
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
            //std::cout << "add Vert...\n";
            if ((*it).get()->type==Vertex::VERTEX3D){
                //std::cout << "add Vertex 3D\n";
                if (!addVertex(*(Vertex3D*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "could not add vertex 3D id " << (*it).get()->vertexId << "\n";
                    return false;
                }
            }
            else if (it->get()->type==Vertex::VERTEXSE3){
                //std::cout << "add Vertex SE3\n";
                if (!addVertex(*(putslam::VertexSE3*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "could not add vertex SE3 id " << (*it).get()->vertexId << "\n";
                    return false;
                }
            }
            else if (it->get()->type==Vertex::VERTEXSE2){
                //std::cout << "add Vertex SE2\n";
                if (!addVertex(*(putslam::VertexSE2*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "could not add vertex SE2\n";
                    return false;
                }
            }
        }
        for (putslam::PoseGraph::EdgeSet::iterator it = tmpGraph.edges.begin(); it!=tmpGraph.edges.end();it++){
            //std::cout << "try add edges\n";
            if (it->get()->type==Edge::EDGE_3D){
                //std::cout << "add Edge 3D\n";
                if (!addEdge(*(Edge3D*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "could not add edge 3D\n";
                    return false;
                }
            }
			if (it->get()->type == Edge::EDGE_3D_REPROJ) {
                //std::cout << "add Edge 3D reproj\n";
				if (!addEdge(*(Edge3DReproj*) it->get())) {
					mtxGraph.unlock();
					std::cout << "could not add edge 3D Reproj\n";
					return false;
				}
			}
            else if (it->get()->type==Edge::EDGE_SE3){
                //std::cout << "add Edge SE3\n";
                if (!addEdge(*(EdgeSE3*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "could not add edge SE3\n";
                    return false;
                }
            }
            else if (it->get()->type==Edge::EDGE_SE2){
                //std::cout << "add Edge SE2\n";
                if (!addEdge(*(EdgeSE2*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "could not add edge SE2\n";
                    return false;
                }
            }
        }
        //merge buffer and pose graph
        mtxGraph.unlock();
        return true;
    }
    else{
        return false;
    }
}

/// Save graph to file
void PoseGraphG2O::save2file(const std::string filename) const{
    optimizer.save(filename.c_str());
}

/// Load graph from file
void PoseGraphG2O::load(const std::string filename) {
    this->loadG2O(filename);
}

/// Load Graph from g2o file
bool PoseGraphG2O::loadG2O(const std::string filename){
    graph.edges.clear();
    graph.vertices.clear();
    optimizer.clear();
    //optimizer.load(filename.c_str());
    std::ifstream file(filename);
    if (file.is_open()){ // open file
        std::string line;
        clear();
        while ( getline (file,line) ) { // load each line
            std::istringstream is(line);
            std::string lineType;
            //std::cout << line << "\n";
            double pos[3], rot[4];
            is >> lineType;
            if (lineType == "PARAMS_SE3OFFSET"){
                cameraOffset = new g2o::ParameterSE3Offset;
                cameraOffset->setId(0);
                Eigen::Isometry3d cameraPose;
                is >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
                Eigen::Quaternion<double> qrot(rot[3], rot[0], rot[1], rot[2]);
                Eigen::Matrix3d R(qrot);
                cameraPose = R; cameraPose.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
                cameraOffset->setOffset(cameraPose);
                optimizer.addParameter(cameraOffset);
            }
            else if (lineType == "VERTEX_SE3:QUAT"){
                unsigned int id;
                is >> id >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
                Eigen::Quaternion<double> qrot(rot[3], rot[0], rot[1], rot[2]);
                Mat34 pose(qrot); pose(0,3) = pos[0]; pose(1,3) = pos[1]; pose(2,3) = pos[2];
                putslam::VertexSE3 vertex(id, pose);
                if (!addVertexPose(vertex))
                    std::cout << "error: vertex exists!\n";
            }
            else if (lineType == "FIX"){
            }
            else if (lineType == "VERTEX_TRACKXYZ"){
                unsigned int id;
                is >> id >> pos[0] >> pos[1] >> pos[2];
                Vec3 position(pos[0], pos[1], pos[2]);
                Vertex3D vertex(id, position);
                if (!addVertexFeature(vertex))
                    std::cout << "error: vertex exists!\n";
            }
            else if (lineType == "VERTEX_SE2"){
                unsigned int id;
                double theta;
                is >> id >> pos[0] >> pos[1] >> theta;
                Eigen::Vector2d position(pos[0], pos[1]);
                VertexSE2 vertex(id, position, theta);
                if (!addVertexSE2(vertex))
                    std::cout << "error: vertex exists!\n";
            }
            else if (lineType == "EDGE_SE3:QUAT"){
                unsigned int to, from;
                double info[21];
                is >> from >> to >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
                for (int i=0;i<21;i++) is >> info[i];
                Vec3 position(pos[0], pos[1], pos[2]); Eigen::Quaternion<double> qrot(rot[3], rot[0], rot[1], rot[2]);
                Mat34 trans(position * qrot);
                Mat66 infoMat;
                infoMat(0,0) = info[0]; infoMat(0,1) = info[1]; infoMat(0,2) = info[2]; infoMat(0,3) = info[3]; infoMat(0,4) = info[4]; infoMat(0,5) = info[5];
                infoMat(1,0) = info[1]; infoMat(1,1) = info[6]; infoMat(1,2) = info[7]; infoMat(1,3) = info[8]; infoMat(1,4) = info[9]; infoMat(1,5) = info[10];
                infoMat(2,0) = info[2]; infoMat(2,1) = info[7]; infoMat(2,2) = info[11]; infoMat(2,3) = info[12]; infoMat(2,4) = info[13]; infoMat(2,5) = info[14];
                infoMat(3,0) = info[3]; infoMat(3,1) = info[8]; infoMat(3,2) = info[12]; infoMat(3,3) = info[15]; infoMat(3,4) = info[16]; infoMat(3,5) = info[17];
                infoMat(4,0) = info[4]; infoMat(4,1) = info[9]; infoMat(4,2) = info[13]; infoMat(4,3) = info[16]; infoMat(4,4) = info[18]; infoMat(4,5) = info[19];
                infoMat(5,0) = info[5]; infoMat(5,1) = info[10]; infoMat(5,2) = info[14]; infoMat(5,3) = info[17]; infoMat(5,4) = info[19]; infoMat(5,5) = info[20];
                EdgeSE3 edge(trans,infoMat,from, to);
                if (!addEdgeSE3(edge))
                    std::cout << "error: vertex doesn't exist!\n";
            }
            else if (lineType == "EDGE_SE3_TRACKXYZ"){
                unsigned int to, from, sensorFrame;
                double info[6];
                is >> from >> to >> sensorFrame >> pos[0] >> pos[1] >> pos[2];
                for (int i=0;i<6;i++) is >> info[i];
                Vec3 position(pos[0], pos[1], pos[2]);
                Mat33 infoMat;
                infoMat(0,0) = info[0]; infoMat(0,1) = info[1]; infoMat(0,2) = info[2];
                infoMat(1,0) = info[1]; infoMat(1,1) = info[3]; infoMat(1,2) = info[4];
                infoMat(2,0) = info[2]; infoMat(2,1) = info[4]; infoMat(2,2) = info[5];
                Edge3D edge(position, infoMat, from, to);
                if (!addEdge3D(edge))
                    std::cout << "error: vertex doesn't exist!\n";
            }
            else if (lineType == "EDGE_SE2"){
                unsigned int to, from;
                double info[6], theta;
                is >> from >> to >> pos[0] >> pos[1] >> theta;
                for (int i=0;i<6;i++) is >> info[i];
                Eigen::Vector2d position(pos[0], pos[1]);
                Mat33 infoMat;
                infoMat(0,0) = info[0]; infoMat(0,1) = info[1]; infoMat(0,2) = info[2];
                infoMat(1,0) = info[1]; infoMat(1,1) = info[3]; infoMat(1,2) = info[4];
                infoMat(2,0) = info[2]; infoMat(2,1) = info[4]; infoMat(2,2) = info[5];
                EdgeSE2 edge(position, theta, infoMat, from, to);
                if (!addEdgeSE2(edge))
                    std::cout << "error: vertex doesn't exist!\n";
            }
        }
        file.close();
        return true;
    }
    else {
        return false;
    }
}

/// Return trajectory (set of SE3 poses)
std::vector<Mat34> PoseGraphG2O::getTrajectory(void) const{
    std::vector<Mat34> vertices;
    for (putslam::PoseGraph::VertexSet::const_iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
        if (it->get()->type==Vertex::VERTEXSE3){
            vertices.push_back(((putslam::VertexSE3*)it->get())->pose);
        }
        if (it->get()->type==Vertex::VERTEXSE2){
            Mat34 pose; pose.setIdentity();
            pose(0,0) = cos(((putslam::VertexSE2*)it->get())->theta); pose(0,1) = -sin(((putslam::VertexSE2*)it->get())->theta);
            pose(1,0) = sin(((putslam::VertexSE2*)it->get())->theta); pose(1,1) = cos(((putslam::VertexSE2*)it->get())->theta);
            pose(0,3) = ((putslam::VertexSE2*)it->get())->pos(0); pose(1,3) = ((putslam::VertexSE2*)it->get())->pos(1);
            vertices.push_back(pose);
        }
    }
    return vertices;
}

/// Export camera path to file (RGB-D SLAM format)
void PoseGraphG2O::export2RGBDSLAM(const std::string filename) const{
    std::ofstream file(filename);
    for (putslam::PoseGraph::VertexSet::const_iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
        if (it->get()->type==Vertex::VERTEXSE3){
            std::ostringstream ossTimestamp;
            Quaternion quat(((putslam::VertexSE3*)it->get())->pose.rotation());
            ossTimestamp << std::setfill('0') << std::setprecision(17) << ((putslam::VertexSE3*)it->get())->timestamp;
            file << ossTimestamp.str() << " " << ((putslam::VertexSE3*)it->get())->pose(0,3) << " "
                    << ((putslam::VertexSE3*)it->get())->pose(1,3) << " " << ((putslam::VertexSE3*)it->get())->pose(2,3) << " "
                    << quat.x() << " " << quat.y() << " " << quat.z()
                    << " " << quat.w() << std::endl;
        }
    }
    file.close();
}

/// Import camera path from file (RGB-D SLAM format)
bool PoseGraphG2O::importRGBDSLAM(const std::string filename){
    std::ifstream file(filename);
    if (file.is_open()){ // open file
        std::string line;
        clear();
        unsigned int vertexId = 0;
        putslam::VertexSE3 vertexPrev;
        while ( getline (file,line) ) { // load each line
            std::istringstream is(line);
            double timestamp = 0;
            double pos[3], rot[4];
            is >> timestamp >> pos[0] >> pos[1] >> pos[2] >> rot[1] >> rot[2] >> rot[3] >> rot[0];
            Quaternion rot1(rot[0], rot[1], rot[2], rot[3]);
            Mat34 pose(rot1.matrix()); pose(0,3) = pos[0]; pose(1,3) = pos[1]; pose(2,3) = pos[2];
            putslam::VertexSE3 vertex(0, pose);
            vertex.timestamp = timestamp; vertex.vertexId = vertexId;
            if (addVertexPose(vertex)) //add vertex
                vertexId++;
            else
                return false;
            if (vertexId>1){ // add edge
                putslam::Mat34 trans = (vertex.pose).inverse() * vertexPrev.pose;
                Mat66 infoMat; infoMat.setIdentity();
                EdgeSE3 edge(trans,infoMat,vertexId-2,vertexId-1);
                if (!addEdgeSE3(edge))
                    return false;
            }
            vertexPrev = vertex;
        }
        file.close();
        return true;
    }
    else {
        return false;
    }
}

/// Optimize graph
bool PoseGraphG2O::optimize(int_fast32_t maxIterations, int verbose, double minimalChi2Ratio) {
    optimizer.setVerbose(verbose);
    if (verbose>0)
        std::cout << "start local graph optimization (t = 0s)\n";
    auto start = std::chrono::system_clock::now();

    // Lock the graph
    mtxGraph.lock();
    HessianInv.resize(0,0);

    optimizer.initializeOptimization();
    //optimizer.computeInitialGuess();

    if (maxIterations >= 0)
        optimizer.optimize((int)maxIterations);
    else {
		double prevChi2 = -1.0, chi2 = -1.0;
		int iterationCounter = 0;
		verbose = 1;
		int iteration = 150;
		while (std::isfinite(chi2) && (iteration > 0 || prevChi2 < 0 || fabs(prevChi2-chi2)/chi2 > minimalChi2Ratio) )
    	{
			prevChi2 = chi2;
    		optimizer.optimize(1);
    		chi2 = optimizer.activeRobustChi2();
    		iterationCounter ++;
    		if ( verbose>0 ){
    			std::cout << "Comparing chi2s : " << prevChi2 << " " << chi2 << std::endl;
    			std::cout << "chi2 ratio = "
						<< (prevChi2 - chi2)/chi2 << std::endl;
    		}
    		iteration--;
    	}
		if ( verbose > 0)
			std::cout<<"Final optimization iteration counter = " << iterationCounter << std::endl;
    }

    newOptimizedVertices.insert(newVertices.begin(), newVertices.end());
    newVertices.clear();

    /// delete features
    for (const auto& featureId : features2remove){
        removeVertex(featureId);
    }
    features2remove.clear();
    // Unlock the graph
    mtxGraph.unlock();

    if (!std::isfinite(optimizer.chi2()))
        return false;
    updateEstimate();
    updateGraph();//try to update graph
    if (verbose>0) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
        std::cout << "finish local graph optimization (t = " << elapsed.count() << "ms)\n";
    }
//    std::string result1 = "graphTmpOpt" + std::to_string (graph.vertices.size()) + ".g2o";
//    while(!updateGraph()){}
//    save2file(result1);
    return true;
}

/// copy g2o optimization result to to putslam graph
void PoseGraphG2O::updateEstimate(void){

	//copy optimized graph to putslam dataset
    std::set<g2o::OptimizableGraph::Vertex*, g2o::OptimizableGraph::VertexIDCompare> verticesToCopy;
    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);
      if (e->level() == 0) {
        for (std::vector<g2o::HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(*it);

            if (!v->fixed())
                verticesToCopy.insert(static_cast<g2o::OptimizableGraph::Vertex*>(*it));
        }
      }
    }
    mtxGraph.lock();
    for (std::set<g2o::OptimizableGraph::Vertex*, g2o::OptimizableGraph::VertexIDCompare>::const_iterator it = verticesToCopy.begin(); it != verticesToCopy.end(); ++it){
      g2o::OptimizableGraph::Vertex* v = *it;
      std::vector<double> estimate;
      v->getEstimateData(estimate);
      PoseGraph::VertexSet::iterator itVertex = findVertex(v->id());
      if (itVertex!=graph.vertices.end()){
          if (itVertex->get()->type==Vertex::VERTEX3D){
            ((Vertex3D*)itVertex->get())->keypoint.depthFeature.x() = estimate[0];
            ((Vertex3D*)itVertex->get())->keypoint.depthFeature.y() = estimate[1];
            ((Vertex3D*)itVertex->get())->keypoint.depthFeature.z() = estimate[2];
            /// set of features modified since last optimization
            std::pair<std::map<int,Vec3>::iterator,bool> ret;
            mtxOptFeatures.lock();
            ret =  optimizedFeatures.insert(std::pair<int,Vec3>(((Vertex3D*)itVertex->get())->vertexId, ((Vertex3D*)itVertex->get())->keypoint.depthFeature));
            if (ret.second==false) {
                ret.first->second = ((Vertex3D*)itVertex->get())->keypoint.depthFeature;
            }
            mtxOptFeatures.unlock();
          }
          if (itVertex->get()->type==Vertex::VERTEXSE2){
            ((VertexSE2*)itVertex->get())->pos.x() = estimate[0];
            ((VertexSE2*)itVertex->get())->pos.y() = estimate[1];
            ((VertexSE2*)itVertex->get())->theta = estimate[2];
          }
          else if (itVertex->get()->type==Vertex::VERTEXSE3){
                ((putslam::VertexSE3*)itVertex->get())->pose(0,3) = estimate[0];
                ((putslam::VertexSE3*)itVertex->get())->pose(1,3) = estimate[1];
                ((putslam::VertexSE3*)itVertex->get())->pose(2,3) = estimate[2];
                Quaternion quat(estimate[6],estimate[3],estimate[4],estimate[5]);
                //((putslam::VertexSE3*)itVertex->get())->pose.matrix().block<3,3>(0,0) = quat.matrix();/// set of features modified since last optimization
                for (int k=0;k<3;k++)
                    for (int l=0;l<3;l++)
                        ((putslam::VertexSE3*)itVertex->get())->pose(k,l) = quat.matrix()(k,l);/// set of features modified since last optimization
                std::pair<std::map<int,Mat34>::iterator,bool> ret;
                mtxOptPoses.lock();
                ret = optimizedPoses.insert(std::pair<int,Mat34>(((VertexSE3*)itVertex->get())->vertexId, ((VertexSE3*)itVertex->get())->pose));
                if (ret.second==false) {
                    ret.first->second = ((VertexSE3*)itVertex->get())->pose;
                }
                mtxOptPoses.unlock();
          }
      }
    }
    mtxGraph.unlock();
    updateGraph();
}

/// get all optimized features
void PoseGraphG2O::getOptimizedFeatures(std::vector<MapFeature>& features){
    features.clear();
    mtxOptFeatures.lock();
    for (std::map<int,Vec3>::iterator it = optimizedFeatures.begin(); it!=optimizedFeatures.end(); ++it){
        MapFeature feature;
        feature.id = it->first; feature.position = it->second;
        features.push_back(feature);
    }
    optimizedFeatures.clear();
    mtxOptFeatures.unlock();
}

/// get all optimized poses
void PoseGraphG2O::getOptimizedPoses(std::vector<VertexSE3>& poses){
    poses.clear();
    mtxOptPoses.lock();
    for (std::map<int,Mat34>::iterator it = optimizedPoses.begin(); it!=optimizedPoses.end(); ++it){
        VertexSE3 pose;
        pose.vertexId = it->first; pose.pose = it->second;
        poses.push_back(pose);
    }
    optimizedPoses.clear();
    mtxOptPoses.unlock();
}

/// returns measured positions and uncertainty of the feature in global coordinates
void PoseGraphG2O::getMeasurements(int featureId, std::vector<Edge3D>& features, Vec3& estimation){
    PoseGraph::VertexSet::iterator vertIt = findVertex(featureId);
    if (vertIt!=graph.vertices.end()){
        // find the estimated value
        estimation = ((Vertex3D*)vertIt->get())->keypoint.depthFeature;
        std::vector<unsigned int> closeSet = findIncominEdges(featureId);
        features.clear();
        // compute position and uncertainty of each 'guess'
        for (std::vector<unsigned int>::iterator it = closeSet.begin(); it!=closeSet.end();it++){
            PoseGraph::EdgeSet::iterator edgeIt = findEdge(*it);
            PoseGraph::VertexSet::iterator camIt = findVertex((unsigned int)edgeIt->get()->fromVertexId);
            if ((edgeIt!=graph.edges.end()) && (camIt!=graph.vertices.end())){
                Mat34 camPose = ((VertexSE3*)camIt->get())->pose;
                Mat34 featureInCam(Mat34::Identity());
                featureInCam(0,3) = ((Edge3D*)edgeIt->get())->trans.x();
                featureInCam(1,3) = ((Edge3D*)edgeIt->get())->trans.y();
                featureInCam(2,3) = ((Edge3D*)edgeIt->get())->trans.z();
                //compute feature position in global frame
                Mat34 featureGlobal = camPose * featureInCam;
                //compute information matrix in global frame J*unc*J'
                Mat33 info = ((Edge3D*)edgeIt->get())->info;
                info = camPose.rotation() * info * camPose.rotation().transpose();
                features.push_back(Edge3D(Vec3(featureGlobal(0,3), featureGlobal(1,3), featureGlobal(2,3)),info,((Edge3D*)edgeIt->get())->fromVertexId,((Edge3D*)edgeIt->get())->toVertexId));
            }
        }
    }
}

/// find all neighboring vertices for which distance is smaller than threshold (not checked)
bool PoseGraphG2O::findNearestNeighbors(int vertexId, int depth, std::vector<int>& neighborsIds){
    if (depth<1) return false;
    std::vector<unsigned int> incomingEdges = findIncominEdges(vertexId);
    std::vector<int> incomingVertices;
    for (std::vector<unsigned int>::iterator it = incomingEdges.begin(); it!=incomingEdges.end(); it++){
        incomingVertices.push_back((int)graph.edges[(*it)]->fromVertexId);
    }
    neighborsIds.insert(neighborsIds.end(),incomingVertices.begin(), incomingVertices.end());
    for (std::vector<int>::iterator it = incomingVertices.begin(); it!=incomingVertices.end(); it++){
        if (!findNearestNeighbors(*it,depth-1,neighborsIds))
            return false;
        else
            return true;
    }
    return true; //TODO: DB check that
}

/// marginalize measurements (pose-feature)
bool PoseGraphG2O::marginalize(const std::vector<int>& keyframes, const std::set<int>& features2remove){
    while(!updateGraph()){}
    //std::cout << "marginalize\n";
    for (const auto vertexId : features2remove){//remove features connected to frames only
        eraseVertex(vertexId);
        g2o::HyperGraph::Vertex* vert;
        for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
            if (it->second->id()==vertexId){
                vert = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
                if (!optimizer.removeVertex(vert,false)) {
                    std::cerr << __PRETTY_FUNCTION__ << ": Failure removing Vertex\n";
                }
                else{
                    //std::cout << "remove feature " << vertexId << "\n";
                    //std::cout << "remove success\n";
                }
                break;
            }
        }
    }
    auto keyframeIter = keyframes.begin();
    for (int poseId=keyframes[0]; poseId<keyframes.back();poseId++){
        if (*keyframeIter==poseId)
            keyframeIter++;
        else {
            g2o::HyperGraph::Vertex* vert;
            for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
                if (it->second->id()==poseId){
                    vert = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
                    if (!optimizer.removeVertex(vert,false)) { // remove vertex and all edges (efficient)
                        std::cerr << __PRETTY_FUNCTION__ << ": Failure removing Vertex\n";
                    }
                    else{
                        auto vertexPose = findVertex(poseId);
                        mtxGraph.lock();
                        //std::cout << "remove vertex pose id " << vertexPose->get()->vertexId << "\n";
                        graph.vertices.erase(vertexPose);    //remove our copy
                        mtxGraph.unlock();
                    }
                    break;
                }
            }
        }
    }
    //std::string result = "graphTmp" + std::to_string (keyframes.back()) + ".g2o";
    while(!updateGraph()){}
    //save2file(result);

    return true; //TODO: DB check that
}

/// erase edges related to the SE3 vertex
void PoseGraphG2O::eraseMeasurements(int poseId){
    mtxGraph.lock();
    for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end();){
        if (it->get()->fromVertexId == (unsigned int)poseId){
            it = graph.edges.erase(it);
        }
        else
            it++;
    }
    mtxGraph.unlock();
}

/// search for sub-graphs which aren't anchored and anchor them
void PoseGraphG2O::anchorVertices(void){
    std::vector<int> vertices;
    for (PoseGraph::VertexSet::iterator it = graph.vertices.begin(); it!=graph.vertices.end(); it++)
        vertices.push_back((int)(*it)->vertexId);
    typedef std::vector< std::vector<int> > GraphOfVertices;
    GraphOfVertices graphs;
    for (std::vector<int>::iterator it = vertices.begin(); it!=vertices.end(); it++)
        graphs.push_back(std::vector<int>(1,*it));
    for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end(); it++){
        GraphOfVertices::iterator itFrom; GraphOfVertices::iterator itTo;
        for (GraphOfVertices::iterator itGraph = graphs.begin(); itGraph!=graphs.end(); itGraph++){
            if (find (itGraph->begin(), itGraph->end(), (*it)->fromVertexId)!=itGraph->end())
                itFrom = itGraph;
            if (find (itGraph->begin(), itGraph->end(), (*it)->toVertexId)!=itGraph->end())
                itTo = itGraph;
        }
        if (itTo==itFrom)//do nothing (vertices are in the same graph)
            continue;
        else {//join subgraphs
            for (std::vector<int>::iterator itVert = (*itTo).begin(); itVert!=(*itTo).end();itVert++){ //copy all vertices from the second graph
                (*itFrom).push_back(*itVert);
            }
            graphs.erase(itTo);// erase the second graph;
        }
    }
    //print graphs
    /*for (GraphOfVertices::iterator itGraph = graphs.begin(); itGraph!=graphs.end(); itGraph++){
        std::cout << "graph: \n";
        for (std::vector<int>::iterator itVert = (*itGraph).begin(); itVert!=(*itGraph).end();itVert++){
            std::cout << *itVert << " ,";
        }
        std::cout << "\n";
    }*/
    for (GraphOfVertices::iterator itGraph = graphs.begin(); itGraph!=graphs.end(); itGraph++){
        g2o::OptimizableGraph::Vertex* v = optimizer.vertex((*itGraph->begin()));
        g2o::SparseOptimizer::VertexContainer::const_iterator it = optimizer.findActiveVertex(v);
        if (it!=optimizer.activeVertices().end()){
        	 (*it)->setFixed(true);
        }
    }
}

/// Find outlier using chi2
g2o::OptimizableGraph::EdgeContainer::iterator PoseGraphG2O::findOutlier(std::vector<unsigned int> edgeSet, g2o::OptimizableGraph::EdgeContainer& activeEdges){
    double maxChi2=-1e10;
    g2o::OptimizableGraph::EdgeContainer::iterator outlierIt;
    for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
        std::vector<unsigned int>::iterator itId = std::find(edgeSet.begin(), edgeSet.end(), (*it)->id());
        if ((itId!=edgeSet.end()) && ((*it)->chi2()>maxChi2)){
            maxChi2 = (*it)->chi2();
            outlierIt = it;
        }
    }
    return outlierIt;
}

/// Find outlier using chi2_i/median(chi2)
g2o::OptimizableGraph::EdgeContainer::iterator PoseGraphG2O::findOutlier(std::vector<unsigned int> edgeSet, g2o::OptimizableGraph::EdgeContainer& activeEdges, double threshold){
    std::vector<double> chi2values;
    double maxError = -1e10;
    g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = activeEdges.end();
    for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
        std::vector<unsigned int>::iterator itId = std::find(edgeSet.begin(), edgeSet.end(), (*it)->id());
        if (itId!=edgeSet.end()){
            chi2values.push_back((*it)->chi2());
        }
    }
    std::sort (chi2values.begin(), chi2values.end(), std::greater<double>());
    double median = chi2values[chi2values.size()/2];
    for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
        std::vector<unsigned int>::iterator itId = std::find(edgeSet.begin(), edgeSet.end(), (*it)->id());
        if ((itId!=edgeSet.end())){
            double error = (*it)->chi2()/median;
            if ((error>maxError)&&(error>threshold)){
                maxError = error;
                outlierIt = it;
            }
        }
    }
    return outlierIt;
}

/// checks if the edge is the single edge outgoing from the vertex fromVertex
bool PoseGraphG2O::isSingleOutgoingEdge(unsigned int edgeId){
    PoseGraph::EdgeSet::iterator edgeIt = findEdge(edgeId);
    unsigned int counter = 0;
    mtxGraph.lock();
    for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end();it++){
        if ((*edgeIt)->fromVertexId == (*it)->fromVertexId)
            counter++;
        if (counter>1) {
            mtxGraph.unlock();
            return false;
        }
    }
    mtxGraph.unlock();
    return true;
}

///return Transform between origin and vertex
Mat34 PoseGraphG2O::getTransform(int vertexId){
    g2o::OptimizableGraph::VertexContainer vertices = optimizer.activeVertices();
    Mat34 tmp;
    for (g2o::OptimizableGraph::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        if ((*it)->id()==vertexId){
            std::vector<double> res;
            (*it)->getEstimateData(res);
            std::cout << "g2o est: \n";
            tmp(0,3)=res[0]; tmp(1,3)=res[1]; tmp(2,3)=res[2];
            Eigen::Vector3d rot(res[3],res[4],res[5]);
            double w=rot.squaredNorm();
            if (w<1) {
              w=sqrt(1-w);
              Eigen::Quaterniond quat(w, res[3], res[4], res[5]);
              tmp(0,0)=quat.matrix()(0,0); tmp(0,1)=quat.matrix()(0,1); tmp(0,2)=quat.matrix()(0,2);
              tmp(1,0)=quat.matrix()(1,0); tmp(1,1)=quat.matrix()(1,1); tmp(1,2)=quat.matrix()(1,2);
              tmp(2,0)=quat.matrix()(2,0); tmp(2,1)=quat.matrix()(2,1); tmp(2,2)=quat.matrix()(2,2);
            } else {
              rot.normalize();
              Eigen::Quaterniond quat(0, rot(0), rot(1), rot(2));
              tmp(0,0)=quat.matrix()(0,0); tmp(0,1)=quat.matrix()(0,1); tmp(0,2)=quat.matrix()(0,2);
              tmp(1,0)=quat.matrix()(1,0); tmp(1,1)=quat.matrix()(1,1); tmp(1,2)=quat.matrix()(1,2);
              tmp(2,0)=quat.matrix()(2,0); tmp(2,1)=quat.matrix()(2,1); tmp(2,2)=quat.matrix()(2,2);
            }
            break;
        }
    }
    return tmp;
}

/// Get Hessian
void PoseGraphG2O::getHessian(Eigen::MatrixXd& hessian, const g2o::OptimizableGraph::VertexContainer& vertices){
    hessian.setZero();
    for (g2o::OptimizableGraph::VertexContainer::const_iterator it = vertices.begin(); it!=vertices.end(); it++){
        if ((*it)->colInHessian()>=0){
            for (int i=0;i<(*it)->dimension();i++){
                for (int j=0;j<(*it)->dimension();j++){
                    hessian(i+(*it)->colInHessian(),j+(*it)->colInHessian()) = (*it)->hessian(i,j);
                }
            }
        }
    }
}

///return Hessian
Mat66 PoseGraphG2O::getPoseIncrementCovariance(int vertexId){
    g2o::OptimizableGraph::VertexContainer vertices = optimizer.activeVertices();
    int dim=0;
    int vertDim=0;
    int colInHessian=-1;
    for (g2o::OptimizableGraph::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        dim+=(*it)->dimension();
        if ((*it)->id()==vertexId){
            colInHessian = (*it)->colInHessian();
            vertDim = (*it)->dimension();//just check if vertexId is pose vertex
        }
    }
    dim-=6; //first vertex is not optimized!
    Eigen::MatrixXd Hessian(dim,dim);
    getHessian(Hessian, vertices);

    if (HessianInv.rows()==0){
        HessianInv.resize(dim, dim);
        HessianInv=Hessian.inverse();
    }
    Mat66 tmp;
    if (colInHessian>=0&&vertDim==6)
        tmp = HessianInv.block<6, 6>(colInHessian, colInHessian);
    return tmp;
}

///return Hessian
Mat33 PoseGraphG2O::getFeatureIncrementCovariance(int vertexId){
    g2o::OptimizableGraph::VertexContainer vertices = optimizer.activeVertices();
    int dim=0;
    int vertDim=0;
    int colInHessian=-1;
    for (g2o::OptimizableGraph::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        dim+=(*it)->dimension();
        if ((*it)->id()==vertexId){
            colInHessian = (*it)->colInHessian();
            vertDim = (*it)->dimension();//just check if vertexId is pose vertex
        }
    }
    dim-=6; //first vertex is not optimized!
    Eigen::MatrixXd Hessian(dim,dim);
    getHessian(Hessian, vertices);

    if (HessianInv.rows()==0){
        HessianInv.resize(dim, dim);
        HessianInv=Hessian.inverse();
    }
    Mat33 tmp;
    if (colInHessian>=0&&vertDim==3)
        tmp = HessianInv.block<3, 3>(colInHessian, colInHessian);
    return tmp;
}

/**
 * Optimizes and removes weak edes (with error bigger than threshold)
 */
bool PoseGraphG2O::optimizeAndPrune(double threshold, unsigned int singleIteration, int verbose){
    (verbose == 0) ? optimizer.setVerbose(false) : optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    anchorVertices();
    optimizer.computeInitialGuess();
    bool opt = true;
    while (opt) {
        opt = false;
        optimize(singleIteration);
        optimizer.computeActiveErrors();
        g2o::OptimizableGraph::EdgeContainer activeEdges = optimizer.activeEdges();
        int iter = 0;
        for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
            //std::cout << "chi2: id: " << (*it)->id() << " chi2: " << (*it)->chi2() << std::endl;
            if ((*it)->chi2()>threshold){
                PoseGraph::EdgeSet::iterator edg = findEdge((*it)->id());
                std::vector<unsigned int> closeSet = findIncominEdges((unsigned int)edg->get()->toVertexId);
                if (closeSet.size()>0){
                    //g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = findOutlier(closeSet, activeEdges, threshold); //chi2/median(chi2)
                    g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = findOutlier(closeSet, activeEdges);//chi2 > threshold
                    //g2o::OptimizableGraph::EdgeContainer closeSet = findIncominEdges();
                    if ((outlierIt!=activeEdges.end())&&(!isSingleOutgoingEdge((*outlierIt)->id()))){
                        opt = true;
                        std::cout << "Remove edge: " << (*outlierIt)->id() << "\n";
                        if (!optimizer.removeEdge(*outlierIt))
                            std::cout << "g2o: Could not remove the edge.\n";
                        std::cout << "removed g2o\n";
                        if (removeEdge((*outlierIt)->id())==graph.edges.end())
                            std::cout << "putslam: Could not remove the edge.\n";
                        //it+=(*outlierIt)->id() - (*it)->id();
                        it = activeEdges.erase(outlierIt);
                        std::cout << "removed putslam\n";
                        if (it==activeEdges.end())
                            it = activeEdges.begin();
                    }
                }
            }
            iter++;
        }
    }
    std::cout << "end\n";
    return true;
    //g2o::OptimizableGraph::Edge e; e.id();
}

/// remove weak features (if measurements number is smaller than threshold)
void PoseGraphG2O::removeWeakFeatures(int threshold){
    g2o::SparseOptimizer::VertexContainer vertices =  optimizer.activeVertices();
    for (g2o::SparseOptimizer::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        //std::cout << (*it)->elementType() << "\n";
        PoseGraph::VertexSet::iterator vert = findVertex((*it)->id());//Todo waek solution
        if (vert->get()->type == Vertex::VERTEX3D) {
            std::vector<unsigned int> incomingEdges = findIncominEdges((*it)->id());
            if ((int)incomingEdges.size()<threshold){
                for (std::vector<unsigned int>::iterator iter = incomingEdges.begin(); iter!=incomingEdges.end(); iter++){
                    removeEdge(*iter);
                    removeEdgeG2O(*iter);
                }
                removeVertex((*it)->id());
                removeVertexG2O((*it)->id());
            }
        }
    }
}

/// set features to remove
void PoseGraphG2O::setFeatures2remove(const std::set<int>& _features2remove){
    for (const auto& featureId : _features2remove)
        features2remove.insert(featureId);
}

/// Prune 3D edges (measurements to features)
bool PoseGraphG2O::prune3Dedges(double threshold){
    optimizer.computeActiveErrors();
    g2o::OptimizableGraph::EdgeContainer activeEdges = optimizer.activeEdges();
    mtxGraph.lock();
    int removed=0;
    std::cout << "active edges: " << activeEdges.size() << "\n";
    for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
        //std::cout << "chi2: id: " << (*it)->id() << " chi2: " << (*it)->chi2() << std::endl;
        PoseGraph::EdgeSet::iterator edg = findEdge((*it)->id());
        PoseGraph::VertexSet::iterator vert = findVertex((unsigned int)edg->get()->toVertexId);
        if (vert->get()->type==Vertex::VERTEX3D){
            //std::cout << "id1: " << vert->get()->vertexId << "\n";
            std::vector<unsigned int> closeSet = findIncominEdges((unsigned int)edg->get()->toVertexId);
            if (closeSet.size()>0){
                g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = findOutlier(closeSet, activeEdges, threshold); //chi2/median(chi2)
                //std::cout << "\n\n\n\n\n\n\n\n\nremoved pruning\n" << (*it)->chi2() << "\n\n\n";
                if ((outlierIt!=activeEdges.end())&&(!isSingleOutgoingEdge((*outlierIt)->id()))){
                //if ((*it)->chi2()>threshold){
                    //if ((it!=activeEdges.end())&&(!isSingleOutgoingEdge((*it)->id()))){
                        //removeEdge((*it)->id());
                        //std::cout << "\n\n\n\n\n\n\n\n\nto big chi2\n";
                    if ((outlierIt!=activeEdges.end())&&(!isSingleOutgoingEdge((*outlierIt)->id()))){
                        removeEdgeG2O((*outlierIt)->id());
                        it = activeEdges.erase(outlierIt);
                        //std::cout << "\n\n\n\n\n\n\n\n\nremoved pruning\n";
                        if (it==activeEdges.end())
                            it = activeEdges.begin();
                        removed++;
                    }
                }
            }
        }
    }
    std::cout << "pruned: " << removed << " edges\n";
    mtxGraph.unlock();
    return true;
}

/**
 * Optimizes and removes weak edes (with error bigger than threshold)
 */
bool PoseGraphG2O::optimizeAndPrune2(double threshold, unsigned int singleIteration, int verbose){
    (verbose == 0) ? optimizer.setVerbose(false) : optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    anchorVertices();
    optimizer.computeInitialGuess();
    bool opt = true;
    while (opt) {
        opt = false;
        optimize(singleIteration);
        optimizer.computeActiveErrors();
        g2o::OptimizableGraph::EdgeContainer activeEdges = optimizer.activeEdges();
        int iter = 0;
        for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
            //std::cout << "chi2: id: " << (*it)->id() << " chi2: " << (*it)->chi2() << std::endl;
            //if ((*it)->chi2()>threshold){
                PoseGraph::EdgeSet::iterator edg = findEdge((*it)->id());
                std::vector<unsigned int> closeSet = findIncominEdges((unsigned int)edg->get()->toVertexId);
                if (closeSet.size()>0){
                    g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = findOutlier(closeSet, activeEdges, threshold); //chi2/median(chi2)
                    //g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = findOutlier(closeSet, activeEdges);//chi2 > threshold
                    //g2o::OptimizableGraph::EdgeContainer closeSet = findIncominEdges();
                    if ((outlierIt!=activeEdges.end())&&(!isSingleOutgoingEdge((*outlierIt)->id()))){
                        opt = true;
                        std::cout << "Remove edge: " << (*outlierIt)->id() << "\n";
                        if (!optimizer.removeEdge(*outlierIt))
                            std::cout << "g2o: Could not remove the edge.\n";
                        std::cout << "removed g2o\n";
                        if (removeEdge((*outlierIt)->id())==graph.edges.end())
                            std::cout << "putslam: Could not remove the edge.\n";
                        //it+=(*outlierIt)->id() - (*it)->id();
                        it = activeEdges.erase(outlierIt);
                        std::cout << "removed putslam\n";
                        if (it==activeEdges.end())
                            it = activeEdges.begin();
                    }
                }
            //}
            iter++;
        }
    }
    std::cout << "end\n";
    return true;
    //g2o::OptimizableGraph::Edge e; e.id();
}

/// Returns set of graph vertices
PoseGraph::VertexSet PoseGraphG2O::getVertices(void){
    PoseGraph::VertexSet verticesTmp;

    mtxGraph.lock();
    for (PoseGraph::VertexSet::iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
        if (it->get()->type==Vertex::VERTEX3D){
            verticesTmp.push_back(std::unique_ptr<Vertex>(new Vertex3D(*(Vertex3D*)it->get())));
        }
        if (it->get()->type==Vertex::VERTEXSE3){
            verticesTmp.push_back(std::unique_ptr<Vertex>(new putslam::VertexSE3(*(putslam::VertexSE3*)it->get())));
        }
        if (it->get()->type==Vertex::VERTEXSE2){
            verticesTmp.push_back(std::unique_ptr<Vertex>(new putslam::VertexSE2(*(putslam::VertexSE2*)it->get())));
        }
    }
    mtxGraph.unlock();
    return verticesTmp;
}

/// Get 3D vertex
Point3D PoseGraphG2O::getVertex(unsigned int id){
    Point3D point; point.x = -1; point.y = -1; point.z = -1;
    for (PoseGraph::VertexSet::iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
        if (it->get()->type==Vertex::VERTEX3D){
            if (it->get()->vertexId==id){
                Vec3 pos = ((Vertex3D*)((Vertex3D*)it->get()))->keypoint.depthFeature;
                point.x = pos.x();   point.y = pos.y();   point.z = pos.z();
                return point;
            }
        }
    }
    return point;
}

/// Returns set of graph edges
PoseGraph::EdgeSet PoseGraphG2O::getEdges(void){
    PoseGraph::EdgeSet edges;
    mtxGraph.lock();
    for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end();it++){
        if (it->get()->type==Edge::EDGE_3D){
            edges.push_back(std::unique_ptr<Edge>(new Edge3D(*(Edge3D*)it->get())));
        }
        if (it->get()->type==Edge::EDGE_SE3){
            edges.push_back(std::unique_ptr<Edge>(new EdgeSE3(*(EdgeSE3*)it->get())));
        }
        if (it->get()->type==Edge::EDGE_SE2){
            edges.push_back(std::unique_ptr<Edge>(new EdgeSE2(*(EdgeSE2*)it->get())));
        }
    }
    mtxGraph.unlock();
    return edges;
}

/// Fix all vertices of the current graph
void PoseGraphG2O::fixOptimizedVertices(void){
	optimizer.setFixed(newOptimizedVertices, true);
    newOptimizedVertices.clear();
}

/// Fix vertex
void PoseGraphG2O::fixVertex(int vertexId){
    g2o::SparseOptimizer::VertexContainer vertices =  optimizer.activeVertices();
    for (g2o::SparseOptimizer::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        if ((*it)->id()==vertexId)//except the first one
            (*it)->setFixed(true);
    }
}

/// unFix vertex
void PoseGraphG2O::unfixVertex(int vertexId){
    g2o::SparseOptimizer::VertexContainer vertices =  optimizer.activeVertices();
    for (g2o::SparseOptimizer::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        if ((*it)->id()==vertexId)//except the first one
            (*it)->setFixed(false);
    }
}

/// Release fixed vertices (except the firts one)
void PoseGraphG2O::releaseFixedVertices(void){
    g2o::SparseOptimizer::VertexContainer vertices =  optimizer.activeVertices();
    for (g2o::SparseOptimizer::VertexContainer::iterator it = vertices.begin(); it!=vertices.end(); it++){
        if ((*it)->id()!=0)//except the first one
        	(*it)->setFixed(false);
    }
}
