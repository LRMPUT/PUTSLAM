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
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>

namespace putslam {
    /// create a single graph (with g2o optimization)
    Graph* createPoseGraphG2O(void);
};

using namespace putslam;

/// Pose Graph g2o implementation
class PoseGraphG2O : public Graph {
    public:
        /// Pointer
        typedef std::unique_ptr<PoseGraphG2O> Ptr;

        /// Construction
        PoseGraphG2O(void);

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
         * Adds an SE3 edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdgeSE3(const EdgeSE3& e);

        /**
         * Adds an 3D edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge3D(const Edge3D& e);

        /// Save graph to file
        void save2file(const std::string filename) const;

        /// Export camera path to file (RGB-D SLAM format)
        void export2RGBDSLAM(const std::string filename) const;

        /// Import camera path from file (RGB-D SLAM format)
        bool importRGBDSLAM(const std::string filename);

        /// Optimize graph
        void optimize(uint_fast32_t maxIterations);

	private:
        /// Pose graph
		PoseGraph graph;
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
        /// mutex for critical section - graph
        std::recursive_mutex mtxGraph;
        /// mutex for critical section - buffer graph
        std::recursive_mutex mtxBuffGraph;

        /// Removes a vertex from the graph. Returns true on success
        bool removeVertex(Vertex* v);

        /// removes an edge from the graph. Returns true on success
        bool removeEdge(Edge* e);

        /**
         * update graph: adds vertices and edges to the graph.
         * returns true, on success, or false on failure.
         */
        bool updateGraph(void);

        /// add vertex to g2o interface
        bool addVertexG2O(uint_fast32_t id, std::stringstream& vertex);

        /// add edge to g2o interface
        bool addEdgeG2O(uint_fast32_t fromId, uint_fast32_t toId, std::stringstream& edgeStream);

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
         * Adds an SE3 edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(const EdgeSE3& e);

        /**
         * Adds an 3D edge to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(const Edge3D& e);

        /// @returns the map <i>id -> vertex</i> where the vertices are stored
        const PoseGraph::VertexSet& vertices() const;

        /// @returns the set of edges of the hyper graph
        const PoseGraph::EdgeSet& edges() const;
};

#endif // GRAPH_G2O_H_INCLUDED
