/** @file graph_g20.h
 *
 * implementation - g2o graph optimization
 * \author Dominik Belter
 */

#ifndef GRAPH_G2O_H_INCLUDED
#define GRAPH_G2O_H_INCLUDED

#include "graph.h"
#include "Defs/g2o.h"
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>

namespace putslam {
    /// create a single graph (with g2o optimization)
    Graph* createPoseGraphG2O(void);
    /// create a single graph (with g2o optimization) and define camera pose
    Graph* createPoseGraphG2O(Mat34 cameraPose);
};

using namespace putslam;

/// Pose Graph g2o implementation
class PoseGraphG2O : public Graph {
    public:
        /// Pointer
        typedef std::unique_ptr<PoseGraphG2O> Ptr;

        /// Construction
        PoseGraphG2O(void);

        /// Overloaded constructor
        PoseGraphG2O(Mat34& cameraPose);

        /// Destructor
        ~PoseGraphG2O(void);

        /// Name of the graph
        const std::string& getName() const;

        /// clears the graph and empties all structures.
        void clear();

        /**
         * adds a vertex to the graph - feature
         * returns true, on success, or false on failure.
         */
        bool addVertexFeature(const Vertex3D& v);

        /**
         * adds a vertex to the graph - pose
         * returns true, on success, or false on failure.
         */
        bool addVertexPose(const VertexSE3& v);

        /**
         * adds a vertex to the graph - x,y,theta
         * returns true, on success, or false on failure.
         */
        bool addVertexSE2(const VertexSE2& v);

        /**
         * update a vertex of the graph - robot pose
         * returns true, on success, or false on failure.
         */
        bool updateVertex(const putslam::VertexSE3& v);

        /**
         * Adds an SE3 edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdgeSE3(const EdgeSE3& e);

        /**
         * Adds an 3D edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge3D(const Edge3D& e);

        /**
         * Adds an 3D reprojection edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge3DReproj(const Edge3DReproj& e);

        /**
         * Adds an SE2 edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdgeSE2(const EdgeSE2& e);

        /// Save graph to file
        void save2file(const std::string filename) const;

        /// Load graph from file
        void load(const std::string filename);

        /// Export camera path to file (RGB-D SLAM format)
        void export2RGBDSLAM(const std::string filename) const;

        /// Import camera path from file (RGB-D SLAM format)
        bool importRGBDSLAM(const std::string filename);

        /// Optimize graph
        bool optimize(int_fast32_t maxIterations, int verbose = 0, double minimalChi2Ratio = 0.99);

        /// Removes weak edes (with error bigger than threshold
        bool optimizeAndPrune(double threshold, unsigned int singleIteration, int verbose = 0);

        /// Removes weak edes (with error bigger than threshold chi2/median(chi2)
        bool optimizeAndPrune2(double threshold, unsigned int singleIteration, int verbose = 0);

        /// Load Graph from g2o file
        bool loadG2O(const std::string filename);

        /// Returns set of graph vertices
        PoseGraph::VertexSet getVertices(void);

        /// Get 3D vertex
        Point3D getVertex(unsigned int id);

        /// Returns set of graph edges
        PoseGraph::EdgeSet getEdges(void);

        /// Return trajectory (set of SE3 poses)
        std::vector<Mat34> getTrajectory(void) const;

        ///return covariance increment
        Mat66 getPoseIncrementCovariance(int vertexId);

        ///return covariance increment
        Mat33 getFeatureIncrementCovariance(int vertexId);

        ///return Transform between origin and vertex
        Mat34 getTransform(int vertexId);

        /// Fix all optimized vertices of the current graph
        void fixOptimizedVertices(void);

        /// Release fixed vertices (except the firts one)
        void releaseFixedVertices(void);

        /// get all optimized features
        void getOptimizedFeatures(std::vector<MapFeature>& features);

        /// get all optimized poses
        void getOptimizedPoses(std::vector<VertexSE3>& poses);

        /// set Robust Kernel
        void setRobustKernel(std::string name, double delta);

        /// disable Robust Kernel
        void disableRobustKernel(void);

        /// remove weak features (if measurements number is smaller than threshold)
        void removeWeakFeatures(int threshold);

        /// Prune 3D edges (measurements to features)
        bool prune3Dedges(double threshold);

        /// returns measured positions and uncertainty of the feature in global coordinates
        void getMeasurements(int featureId, std::vector<Edge3D>& features, Vec3& estimation);

        /// find all neighboring vertices for which distance is smaller than threshold (not checked)
        bool findNearestNeighbors(int vertexId, int depth, std::vector<int>& neighborsIds);

        /// marginalize measurements (pose-feature)
        bool marginalize(const std::vector<int>& keyframes, const std::set<int>& features2remove);

        /// Fix vertex
        void fixVertex(int vertexId);

        /// unFix vertex
        void unfixVertex(int vertexId);

        /// set features to remove
        void setFeatures2remove(const std::set<int>& _features2remove);

        /// check trajectory
        void checkTrajectory(const std::vector<Mat34>& odoMeasurements);

        /// erase edges related to the SE3 vertex
        void eraseMeasurements(int poseId);

    private:
        /// Pose graph
        PoseGraph bufferGraph;
        /// g2o linear solver
        g2o::BlockSolverX::LinearSolverType * linearSolver;
        /// the linear solver
        g2o::BlockSolverX* blockSolver;
        /// the algorithm to carry out the optimization
        g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm;
        /// the optimizer to load the data and carry out the optimization
        g2o::SparseOptimizer optimizer;
        /// g2o factory
        g2o::Factory* factory;
        /// mutex for critical section - buffer graph
        std::recursive_mutex mtxBuffGraph;
        /// camera offset
        g2o:: ParameterSE3Offset* cameraOffset;
        /// set of new vertices
        g2o::HyperGraph::VertexSet newVertices;
        /// set of new vertices (after optimization they can be fixed)
        g2o::HyperGraph::VertexSet newOptimizedVertices;
        /// set of features modified since last optimization
        std::map<int, Vec3> optimizedFeatures;
        /// mutex for optimized features
        std::mutex mtxOptFeatures;
        /// set of camera poses modified since last optimization
        std::map<int, Mat34> optimizedPoses;
        /// mutex for optimized poses
        std::mutex mtxOptPoses;
        /// current inverse of hessian
        Eigen::MatrixXd HessianInv;
        /// features to remove
        std::set<int> features2remove;


        /// Removes a vertex from the graph. Returns true on success
        PoseGraph::VertexSet::iterator removeVertex(unsigned int id);

        /// removes an edge from the graph. Returns true on success
        PoseGraph::EdgeSet::iterator removeEdge(unsigned int id);

        /**
         * update graph: adds vertices and edges to the graph.
         * returns true, on success, or false on failure.
         */
        bool updateGraph(void);

        /// add vertex to g2o interface
        bool addVertexG2O(uint_fast32_t id, std::stringstream& vertex, Vertex::Type type);

        /// add edge to g2o interface
        bool addEdgeG2O(uint_fast32_t id, uint_fast32_t fromId, uint_fast32_t toId, std::stringstream& edgeStream, Edge::Type type);

        /**
         * adds a vertex to the graph - feature
         * returns true, on success, or false on failure.
         */
        bool addVertex(const Vertex3D& v);

        /**
         * adds a vertex to the graph - pose
         * returns true, on success, or false on failure.
         */
        bool addVertex(const VertexSE3& v);

        /**
         * adds a vertex to the graph - x,y,theta
         * returns true, on success, or false on failure.
         */
        bool addVertex(const VertexSE2& v);

        /**
         * Adds an SE3 edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(EdgeSE3& e);

        /**
         * Adds an 3D edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(Edge3D& e);


        /**
         * Adds an 3D reprojection edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(Edge3DReproj& e);

        /**
         * Adds an SE2 edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(EdgeSE2& e);

        /// @returns the map <i>id -> vertex</i> where the vertices are stored
        const PoseGraph::VertexSet& vertices() const;

        /// @returns the set of edges of the hyper graph
        const PoseGraph::EdgeSet& edges() const;

        /// Find outlier using chi2
        g2o::OptimizableGraph::EdgeContainer::iterator findOutlier(std::vector<unsigned int> edgeSet, g2o::OptimizableGraph::EdgeContainer& activeEdges);

        /// Find outlier using chi2_i/median(chi2)
        g2o::OptimizableGraph::EdgeContainer::iterator findOutlier(std::vector<unsigned int> edgeSet, g2o::OptimizableGraph::EdgeContainer& activeEdges, double threshold);

        /// copy g2o optimization result to to putslam graph
        void updateEstimate(void);

        /// search for sub-graphs which aren't anchored and anchor them
        void anchorVertices(void);

        /// checks if the edge is the single edge outgoing from the vertex fromVertex
        bool isSingleOutgoingEdge(unsigned int edgeId);

        /// removes an edge from the g2o graph. Returns true on success
        bool removeEdgeG2O(unsigned int id);

        /// removes vertex from the g2o graph. Returns true on success
        bool removeVertexG2O(unsigned int id);

        /// Get Hessian
        void getHessian(Eigen::MatrixXd& hessian, const g2o::OptimizableGraph::VertexContainer& vertices);
};

#endif // GRAPH_G2O_H_INCLUDED
