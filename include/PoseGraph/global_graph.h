/** @file global_graph.h
 *
 * implementation - global graph optimization
 *
 */

#ifndef GLOBAL_GRAPH_H_INCLUDED
#define GLOBAL_GRAPH_H_INCLUDED

#include "graph.h"
#include <iostream>
#include <memory>

namespace putslam {
    /// create a single global graph (with loop closure detection)
    Graph* createGlobalGraph(void);
};

using namespace putslam;

/// Global Graph implementation
class GlobalGraph : public Graph {
    public:
        /// Pointer
        typedef std::unique_ptr<GlobalGraph> Ptr;

        /// Construction
        GlobalGraph(void);

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
         * adds a vertex to the graph pose
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

        /// Removes weak edes (with error bigger than threshold
        bool optimizeAndPrune(float_type threshold, unsigned int singleIteration);

	private:
		PoseGraph graph;	

        /**
         * update graph: adds vertices and edges to the graph.
         * returns true, on success, or false on failure.
         */
        bool updateGraph(const VertexSE3& v);

        /// Removes a vertex from the graph. Returns true on success
        bool removeVertex(Vertex* v);

        /// removes an edge from the graph. Returns true on success
        bool removeEdge(Edge* e);

        /// @returns the map <i>id -> vertex</i> where the vertices are stored
        const PoseGraph::VertexSet& vertices() const;

        /// @returns the set of edges of the hyper graph
        const PoseGraph::EdgeSet& edges() const;
};

#endif // GLOBAL_GRAPH_H_INCLUDED
