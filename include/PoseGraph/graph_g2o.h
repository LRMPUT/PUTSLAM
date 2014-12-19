/** @file graph_g20.h
 *
 * implementation - g2o graph optimization
 *
 */

#ifndef GRAPH_G2O_H_INCLUDED
#define GRAPH_G2O_H_INCLUDED

#include "graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"

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
        bool optimize(uint_fast32_t maxIterations);

        /// Removes weak edes (with error bigger than threshold
        bool optimizeAndPrune(float_type threshold, unsigned int singleIteration);

        /// Removes weak edes (with error bigger than threshold chi2/median(chi2)
        bool optimizeAndPrune2(float_type threshold, unsigned int singleIteration);

        /// Load Graph from g2o file
        bool loadG2O(const std::string filename);

        /// Returns set of graph vertices
        PoseGraph::VertexSet getVertices(void);

        /// Returns set of graph edges
        PoseGraph::EdgeSet getEdges(void);

        /// Return trajectory (set of SE3 poses)
        std::vector<Mat34> getTrajectory(void) const;

        ///return Hessian
        Eigen::MatrixXd getHessian(int vertexId);

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

        /// Removes a vertex from the graph. Returns true on success
        bool removeVertex(unsigned int id);

        /// removes an edge from the graph. Returns true on success
        bool removeEdge(unsigned int id);

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
        g2o::OptimizableGraph::EdgeContainer::iterator findOutlier(std::vector<unsigned int> edgeSet, g2o::OptimizableGraph::EdgeContainer& activeEdges, float_type threshold);

        /// copy g2o optimization result to to putslam graph
        void updateEstimate(void);

        /// search for sub-graphs which aren't anchored and anchor them
        void anchorVertices(void);

        /// checks if the edge is the single edge outgoing from the vertex fromVertex
        bool isSingleOutgoingEdge(unsigned int edgeId);

};

#endif // GRAPH_G2O_H_INCLUDED
