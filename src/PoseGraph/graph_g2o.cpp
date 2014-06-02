#include "../include/PoseGraph/graph_g2o.h"
#include <stdexcept>
#include <chrono>

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

putslam::Graph* putslam::createPoseGraphG2O(Mat34 cameraPose) {
    graph_g2o.reset(new PoseGraphG2O(cameraPose));
    return graph_g2o.get();
}

PoseGraphG2O::PoseGraphG2O(void) : Graph("Pose Graph g2o") {
    // create the linear solver
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();

    // create the block solver on top of the linear solver
    blockSolver = new g2o::BlockSolverX(linearSolver);

    // create the algorithm to carry out the optimization
    OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
    //optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    factory = g2o::Factory::instance();

    cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;  R  << 1,  0,  0,  0,  1,  0,  0,  0,  1;
    cameraPose = R; cameraPose.translation() = Vector3d(0.0, 0.0, 0.0);
    cameraOffset->setOffset(cameraPose);
    optimizer.addParameter(cameraOffset);
}

PoseGraphG2O::PoseGraphG2O(Mat34& cameraPose) : PoseGraphG2O() {
    Eigen::Isometry3d camPos;
    camPos = cameraPose.matrix();
    camPos.translation() = Vector3d (cameraPose(0,3), cameraPose(1,3), cameraPose(2,3));
    cameraOffset->setOffset(camPos);    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);
}

/// Destructor
PoseGraphG2O::~PoseGraphG2O(void){
}

const std::string& PoseGraphG2O::getName() const {
    return name;
}

/// Removes a vertex from the graph. Returns true on success
bool PoseGraphG2O::removeVertex(unsigned int id){
    mtxGraph.lock();
    PoseGraph::VertexSet::iterator v = findVertex(id);
    if (v != graph.vertices.end()) {
        graph.vertices.erase(v);
        mtxGraph.unlock();
        return true;
    }
    mtxGraph.unlock();
    return false;
}

/// Find vertex by id
PoseGraph::VertexSet::iterator PoseGraphG2O::findVertex(unsigned int id){
    mtxGraph.lock();
    for (PoseGraph::VertexSet::iterator it = graph.vertices.begin(); it!=graph.vertices.end(); it++){
        if (it->get()->vertexId == id){
            mtxGraph.unlock();
            return it;
        }
    }
    mtxGraph.unlock();
    return graph.vertices.end();
}

/// Find edge by id
PoseGraph::EdgeSet::iterator PoseGraphG2O::findEdge(unsigned int id){
    mtxGraph.lock();
    for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end(); it++){
        if (it->get()->id == id){
            mtxGraph.unlock();
            return it;
        }
    }
    mtxGraph.unlock();
    return graph.edges.end();
}

/// removes an edge from the graph. Returns true on success
bool PoseGraphG2O::removeEdge(unsigned int id){
    mtxGraph.lock();
    PoseGraph::EdgeSet::iterator e = findEdge(id);
    if (e != graph.edges.end()) {
        graph.edges.erase(e);
        mtxGraph.unlock();
        return true;
    }
    mtxGraph.unlock();
    return false;
}

/// clears the graph and empties all structures.
void PoseGraphG2O::clear(){
    mtxGraph.lock();
    optimizer.clear();
    graph.edges.clear();
    graph.vertices.clear();
    bufferGraph.vertices.clear();
    bufferGraph.edges.clear();
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
    g2o::OptimizableGraph::Vertex* vert = static_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
    g2o::HyperGraph::GraphElemBitset elemBitset;
    elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
    elemBitset.flip();

    g2o::HyperGraph::HyperGraphElement* element;
    if (type==Vertex::VERTEXSE3)
        element = factory->construct("VERTEX_SE3:QUAT", elemBitset);
    else if (type==Vertex::VERTEX3D)
        element = factory->construct("VERTEX_TRACKXYZ", elemBitset);
    else {
        std::cout << "error: unknown vertex type!\n";
        return false;
    }
    g2o::OptimizableGraph::Vertex* vse3 = static_cast<g2o::OptimizableGraph::Vertex*>(element);
    vse3->read(vertex);
    vse3->setId(id);
    if (graph.vertices.size()==1){
        vse3->setFixed(true);
    }
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
        currentLine << v.keypoint.depthFeature.x() << ' ' << v.keypoint.depthFeature.y() << ' ' << v.keypoint.depthFeature.z();
        addVertexG2O(v.vertexId, currentLine, Vertex::VERTEX3D);
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
    if (graph.vertices.size() > v.vertexId){//wrong id
        mtxGraph.unlock();
        return false;
    }
    else {//add vertex
        graph.vertices.push_back(std::unique_ptr<Vertex>(new putslam::VertexSE3(v)));//update putslam structure
        std::stringstream currentLine;
        currentLine << v.nodeSE3.pos.x() << ' ' << v.nodeSE3.pos.y() << ' ' << v.nodeSE3.pos.z() << ' ' << v.nodeSE3.rot.x() << ' ' << v.nodeSE3.rot.y() << ' ' << v.nodeSE3.rot.z() << ' ' << v.nodeSE3.rot.w();
        addVertexG2O(v.vertexId, currentLine, Vertex::VERTEXSE3);
        mtxGraph.unlock();
        return true;
    }
}

/// add edge to g2o interface
bool PoseGraphG2O::addEdgeG2O(uint_fast32_t id, uint_fast32_t fromId, uint_fast32_t toId, std::stringstream& edgeStream, Edge::Type type){
    g2o::HyperGraph::GraphElemBitset elemBitset;
    elemBitset[g2o::HyperGraph::HGET_PARAMETER] = 1;
    elemBitset.flip();
    HyperGraph::HyperGraphElement* element;
    if (type==Edge::EDGE_SE3)
        element = factory->construct("EDGE_SE3:QUAT", elemBitset);
    else if (type==Edge::EDGE_3D)
        element = factory->construct("EDGE_SE3_TRACKXYZ", elemBitset);
    else {
        std::cout << "error: unknown edge type!\n";
        return false;
    }
    g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(element);
    g2o::OptimizableGraph::Vertex* from = optimizer.vertex(fromId);
    g2o::OptimizableGraph::Vertex* to = optimizer.vertex(toId);
    edge->setVertex(0, from);
    edge->setVertex(1, to);
    edge->read(edgeStream);
    edge->setId(id);
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
    std::cout << "sdw1\n";
    updateGraph();//try to update graph
    return true;
}

/**
 * Adds an SE3 edge  to the graph. If the edge is already in the graph, it
 * does nothing and returns false. Otherwise it returns true.
 */
bool PoseGraphG2O::addEdge(EdgeSE3& e){
    mtxGraph.lock();
    if (findVertex(e.toVertexId)==graph.vertices.end()){// to vertex does not exist
        std::cout << "Warning: vertex does not exist. adding new vertex...\n";
        mtxGraph.unlock();
        Vec3 pos(0.0, 0.0, 0.0); Eigen::Quaternion<double> rot(1, 0, 0, 0);
        addVertexPose(putslam::VertexSE3(e.toVertexId, pos, rot));
        mtxGraph.lock();
    }
    if (findVertex(e.fromVertexId)==graph.vertices.end()){// to vertex does not exist
        std::cout << "Warning: vertex does not exist. adding new vertex...\n";
        mtxGraph.unlock();
        Vec3 pos(0.0, 0.0, 0.0); Eigen::Quaternion<double> rot(1, 0, 0, 0);
        addVertexPose(putslam::VertexSE3(e.fromVertexId, pos, rot));
        mtxGraph.lock();
    }
        std::cout << "add 3\n";
    e.id = graph.edges.size();
    graph.edges.push_back(std::unique_ptr<Edge>(new EdgeSE3(e)));
    std::stringstream currentLine;
    std::cout << "add 4\n";
    currentLine << e.trans.pos.x() << ' ' << e.trans.pos.y() << ' ' << e.trans.pos.z()
                << ' ' << e.trans.rot.x() << ' ' << e.trans.rot.y() << ' ' << e.trans.rot.z() << ' ' << e.trans.rot.w()
                << ' ' << e.info(0,0) << ' ' << e.info(0,1) << ' ' << e.info(0,2) << ' ' << e.info(0,3) << ' ' << e.info(0,4) << ' ' << e.info(0,5)
                << ' ' << e.info(1,1) << ' ' << e.info(1,2) << ' ' << e.info(1,3) << ' ' << e.info(1,4) << ' ' << e.info(1,5)
                << ' ' << e.info(2,2) << ' ' << e.info(2,3) << ' ' << e.info(2,4) << ' ' << e.info(2,5)
                << ' ' << e.info(3,3) << ' ' << e.info(3,4) << ' ' << e.info(3,5)
                << ' ' << e.info(4,4) << ' ' << e.info(4,5)
                << ' ' << e.info(5,5);
    std::cout << "add 5\n";
    addEdgeG2O(e.id, e.fromVertexId, e.toVertexId, currentLine, Edge::EDGE_SE3);
    std::cout << "add 6\n";
    mtxGraph.unlock();
    return true;
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
bool PoseGraphG2O::addEdge(Edge3D& e){
    mtxGraph.lock();
    if (findVertex(e.toVertexId)==graph.vertices.end()){// to vertex does not exist
        std::cout << "Warning: vertex does not exist. adding new vertex...\n";
        mtxGraph.unlock();
        Vec3 pos(0.0, 0.0, 0.0);
        addVertexFeature(putslam::Vertex3D(e.toVertexId, pos));
        mtxGraph.lock();
    }
    if (findVertex(e.fromVertexId)==graph.vertices.end()){// to vertex does not exist
        std::cout << "Warning: vertex does not exist. adding new vertex...\n";
        mtxGraph.unlock();
        Vec3 pos(0.0, 0.0, 0.0); Eigen::Quaternion<double> rot(1, 0, 0, 0);
        addVertexPose(putslam::VertexSE3(e.fromVertexId, pos, rot));
        mtxGraph.lock();
    }
    e.id = graph.edges.size();
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

/**
 * update graph: adds vertices and edges to the graph.
 * returns true, on success, or false on failure.
 */
bool PoseGraphG2O::updateGraph(void){
    if (mtxGraph.try_lock()){//try to lock graph
        std::cout << "update graph\n";
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
                if (!addVertex(*(Vertex3D*)it->get())){
                    mtxGraph.unlock();
                    return false;
                }
            }
            else if (it->get()->type==Vertex::VERTEXSE3){
                if (!addVertex(*(putslam::VertexSE3*)it->get())){
                    mtxGraph.unlock();
                    return false;
                }
            }
        }
        for (putslam::PoseGraph::EdgeSet::iterator it = tmpGraph.edges.begin(); it!=tmpGraph.edges.end();it++){
            if (it->get()->type==Edge::EDGE_3D){
                std::cout << "add Edge 3D\n";
                if (!addEdge(*(Edge3D*)it->get())){
                    mtxGraph.unlock();
                    return false;
                }
            }
            else if (it->get()->type==Edge::EDGE_SE3){
                std::cout << "add Edge SE3\n";
                if (!addEdge(*(EdgeSE3*)it->get())){
                    mtxGraph.unlock();
                    std::cout << "add Edge SE3 failed\n";
                    return false;
                }
            }
        }
        //merge buffer and pose graph
        mtxGraph.unlock();
    }
    std::cout << "update graph finished\n";
    return true;
}

/// Save graph to file
void PoseGraphG2O::save2file(const std::string filename) const{
    optimizer.save(filename.c_str());
}

/// Load Graph from g2o file
bool PoseGraphG2O::loadG2O(const std::string filename){
    graph.edges.clear();
    graph.vertices.clear();
    optimizer.clear();
    //optimizer.load(filename.c_str());
    ifstream file(filename);
    if (file.is_open()){ // open file
        string line;
        clear();
        while ( getline (file,line) ) { // load each line
            std::cout << line << "\n";
            std::istringstream is(line);
            std::string lineType;
            float_type pos[3], rot[4];
            is >> lineType;
            if (lineType == "PARAMS_SE3OFFSET"){
                cameraOffset = new g2o::ParameterSE3Offset;
                cameraOffset->setId(0);
                Eigen::Isometry3d cameraPose;
                is >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
                Eigen::Quaternion<double> qrot(rot[3], rot[0], rot[1], rot[2]);
                Eigen::Matrix3d R(qrot);
                cameraPose = R; cameraPose.translation() = Vector3d(pos[0], pos[1], pos[2]);
                cameraOffset->setOffset(cameraPose);
                optimizer.addParameter(cameraOffset);
            }
            else if (lineType == "VERTEX_SE3:QUAT"){
                unsigned int id;
                is >> id >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
                Vec3 position(pos[0], pos[1], pos[2]);  Eigen::Quaternion<double> qrot(rot[3], rot[0], rot[1], rot[2]);
                putslam::VertexSE3 vertex(id, position, qrot);
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
                std::cout << "trackxyz\n";
                if (!addVertexFeature(vertex))
                    std::cout << "error: vertex exists!\n";
            }
            else if (lineType == "EDGE_SE3:QUAT"){
                unsigned int to, from;
                float_type info[21];
                is >> to >> from >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> rot[3];
                for (int i=0;i<21;i++) is >> info[i];
                Vec3 position(pos[0], pos[1], pos[2]); Eigen::Quaternion<double> qrot(rot[3], rot[0], rot[1], rot[2]);
                RobotPose trans(position, qrot);
                Mat66 infoMat;
                std::cout << "sd\n";
                infoMat(0,0) = info[0]; infoMat(0,1) = info[1]; infoMat(0,2) = info[2]; infoMat(0,3) = info[3]; infoMat(0,4) = info[4]; infoMat(0,5) = info[5];
                infoMat(1,0) = info[1]; infoMat(1,1) = info[6]; infoMat(1,2) = info[7]; infoMat(1,3) = info[8]; infoMat(1,4) = info[9]; infoMat(1,5) = info[10];
                infoMat(2,0) = info[2]; infoMat(2,1) = info[7]; infoMat(2,2) = info[11]; infoMat(2,3) = info[12]; infoMat(2,4) = info[13]; infoMat(2,5) = info[14];
                infoMat(3,0) = info[3]; infoMat(3,1) = info[8]; infoMat(3,2) = info[12]; infoMat(3,3) = info[15]; infoMat(3,4) = info[16]; infoMat(3,5) = info[17];
                infoMat(4,0) = info[4]; infoMat(4,1) = info[9]; infoMat(4,2) = info[13]; infoMat(4,3) = info[16]; infoMat(4,4) = info[18]; infoMat(4,5) = info[19];
                infoMat(5,0) = info[5]; infoMat(5,1) = info[10]; infoMat(5,2) = info[14]; infoMat(5,3) = info[17]; infoMat(5,4) = info[19]; infoMat(5,5) = info[20];
                std::cout << "sd1\n";
                EdgeSE3 edge(trans,infoMat,to,from);
                std::cout << "sd2\n";
                if (!addEdgeSE3(edge))
                    std::cout << "error: vertex doesn't exist!\n";
                std::cout << "sd3\n";
            }
            else if (lineType == "EDGE_SE3_TRACKXYZ"){
                unsigned int to, from, sensorFrame;
                float_type info[6];
                is >> to >> from >> sensorFrame >> pos[0] >> pos[1] >> pos[2];
                for (int i=0;i<6;i++) is >> info[i];
                Vec3 position(pos[0], pos[1], pos[2]);
                Mat33 infoMat;
                infoMat(0,0) = info[0]; infoMat(0,1) = info[1]; infoMat(0,2) = info[2];
                infoMat(1,0) = info[1]; infoMat(1,1) = info[3]; infoMat(1,2) = info[4];
                infoMat(2,0) = info[2]; infoMat(2,1) = info[4]; infoMat(2,2) = info[5];
                Edge3D edge(position, infoMat, to, from);
                if (!addEdge3D(edge))
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

/// Export camera path to file (RGB-D SLAM format)
void PoseGraphG2O::export2RGBDSLAM(const std::string filename) const{
    ofstream file(filename);
    for (putslam::PoseGraph::VertexSet::const_iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
        if (it->get()->type==Vertex::VERTEXSE3){
            file << std::setprecision (numeric_limits<double>::digits10 + 1) << ((putslam::VertexSE3*)it->get())->timestamp << " " << std::setprecision (8) << ((putslam::VertexSE3*)it->get())->nodeSE3.pos.x() << " " << ((putslam::VertexSE3*)it->get())->nodeSE3.pos.y() << " " << ((putslam::VertexSE3*)it->get())->nodeSE3.pos.z() << " " << ((putslam::VertexSE3*)it->get())->nodeSE3.rot.x() << " " << ((putslam::VertexSE3*)it->get())->nodeSE3.rot.y() << " " << ((putslam::VertexSE3*)it->get())->nodeSE3.rot.z() << " " << ((putslam::VertexSE3*)it->get())->nodeSE3.rot.w() << std::endl;
        }
    }
    file.close();
}

/// Import camera path from file (RGB-D SLAM format)
bool PoseGraphG2O::importRGBDSLAM(const std::string filename){
    ifstream file(filename);
    if (file.is_open()){ // open file
        string line;
        clear();
        uint_fast32_t vertexId = 0;
        putslam::VertexSE3 vertexPrev;
        while ( getline (file,line) ) { // load each line
            std::istringstream is(line);
            float_type timestamp = 0;
            float_type pos[3], rot[4];
            is >> timestamp >> pos[0] >> pos[1] >> pos[2] >> rot[1] >> rot[2] >> rot[3] >> rot[0];
            Vec3 pos1(pos[0], pos[1], pos[2]);  putslam::Quaternion rot1(rot[0], rot[1], rot[2], rot[3]);
            putslam::VertexSE3 vertex(0, pos1, rot1);
            vertex.timestamp = timestamp; vertex.vertexId = vertexId;
            if (addVertexPose(vertex)) //add vertex
                vertexId++;
            else
                return false;
            if (vertexId>1){ // add edge
                putslam::Mat34 v3 = (vertex.nodeSE3.pos * vertex.nodeSE3.rot).inverse() * vertexPrev.nodeSE3.pos * vertexPrev.nodeSE3.rot;
                putslam::Quaternion q(v3.rotation()); putslam::Vec3 p(v3.translation());
                RobotPose trans(p,q); Mat66 infoMat; infoMat.setIdentity();
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
void PoseGraphG2O::optimize(uint_fast32_t maxIterations) {
    std::cout << "start local graph optimization (t = 0s)\n";
    auto start = std::chrono::system_clock::now();
    optimizer.initializeOptimization();
    optimizer.computeInitialGuess();
    optimizer.optimize(maxIterations);

    updateEstimate();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
    std::cout << "finish local graph optimization (t = " << elapsed.count() << "ms)\n";
}

/// copy g2o optimization result to to putslam graph
void PoseGraphG2O::updateEstimate(void){
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
    mtxGraph.unlock();
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
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.pos.x() = estimate[0];
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.pos.y() = estimate[1];
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.pos.z() = estimate[2];
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.x() = estimate[3];
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.y() = estimate[4];
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.z() = estimate[5];
          ((putslam::VertexSE3*)graph.vertices[iter].get())->nodeSE3.rot.w() = estimate[6];
      }
      iter++;
    }
    updateGraph();
    mtxGraph.unlock();
}

/// Find all edges which points to the vertex 'toVertexId'
std::vector<unsigned int> PoseGraphG2O::findIncominEdges(unsigned int toVertexId){
    std::vector<unsigned int> edgeIds;
    for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end(); it++){
        if (it->get()->toVertexId == toVertexId)
            edgeIds.push_back(it->get()->id);
    }
    return edgeIds;
}

/// Find outlier using chi2
g2o::OptimizableGraph::EdgeContainer::iterator PoseGraphG2O::findOutlier(std::vector<unsigned int> edgeSet, g2o::OptimizableGraph::EdgeContainer& activeEdges){
    float_type maxChi2=-1e10;
    g2o::OptimizableGraph::EdgeContainer::iterator outlierIt;
    for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
        std::vector<unsigned int>::iterator itId = std::find(edgeSet.begin(), edgeSet.end(), (*it)->id());
        if ((itId!=edgeSet.end()) && ((*it)->chi2()>maxChi2)){
            maxChi2 = (*it)->chi2();
            outlierIt = it;
        }
    }
    std::cout << "maxCi2 " << maxChi2 << "\n";
    return outlierIt;
}

/**
 * Optimizes and removes weak edes (with error bigger than threshold)
 */
bool PoseGraphG2O::optimizeAndPrune(float_type threshold, unsigned int singleIteration){
    optimizer.initializeOptimization();
    optimizer.computeInitialGuess();
    bool opt = true;
    while (opt) {
        opt = false;
        optimize(singleIteration);
        optimizer.computeActiveErrors();
        g2o::OptimizableGraph::EdgeContainer activeEdges = optimizer.activeEdges();
        int iter = 0;
        for (g2o::OptimizableGraph::EdgeContainer::iterator it = activeEdges.begin(); it!=activeEdges.end(); it++){
            std::cout << "sss1\n";
            std::cout << "chi2: id: " << (*it)->id() << " chi2: " << (*it)->chi2() << std::endl;
            if ((*it)->chi2()>threshold){
                PoseGraph::EdgeSet::iterator edg = findEdge((*it)->id());
                std::cout << "to vertex: " << edg->get()->toVertexId << "\n";
                std::vector<unsigned int> closeSet = findIncominEdges(edg->get()->toVertexId);
                if (closeSet.size()>0){
                    g2o::OptimizableGraph::EdgeContainer::iterator outlierIt = findOutlier(closeSet, activeEdges);
                    //g2o::OptimizableGraph::EdgeContainer closeSet = findIncominEdges();
                    opt = true;
                    if (*it!=*outlierIt)
                        std::cout << "FD\n";
                    std::cout << "Original edge: " << (*it)->id() << "\n";
                    std::cout << "Remove edge: " << (*outlierIt)->id() << "\n";
                    if (!optimizer.removeEdge(*outlierIt))
                        std::cout << "g2o: Could not remove the edge.\n";
                    std::cout << "removed g2o\n";
                    if (!removeEdge((*outlierIt)->id()))
                        std::cout << "putslam: Could not remove the edge.\n";
                    it+=(*outlierIt)->id() - (*it)->id();
                    std::cout << "removed putslam\n";
                }
            }
            iter++;
        }
    }
    std::cout << "end\n";
    return true;
    //g2o::OptimizableGraph::Edge e; e.id();
}
