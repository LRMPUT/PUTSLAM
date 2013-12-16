/** @file graph_g20.h
 *
 * implementation - g2o graph optimization
 *
 */

#ifndef GRAPH_G2O_H_INCLUDED
#define GRAPH_G2O_H_INCLUDED

#include "graph.h"
#include <iostream>
#include <memory>

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

        /// Name of the graph
        const std::string& getName() const;

        /// Removes a vertex from the graph. Returns true on success
        bool removeVertex(Vertex* v);

        /// removes an edge from the graph. Returns true on success
        bool removeEdge(Edge* e);

        /// clears the graph and empties all structures.
        void clear();

        /// @returns the map <i>id -> vertex</i> where the vertices are stored
        const PoseGraph::VertexSet& vertices() const;

        /// @returns the set of edges of the hyper graph
        const PoseGraph::EdgeSet& edges() const;

        /**
         * adds a vertex to the graph.
         * returns true, on success, or false on failure.
         */
        bool addVertex(Vertex& v);

        /**
         * Adds an edge  to the graph. If the edge is already in the graph, it
         * does nothing and returns false. Otherwise it returns true.
         */
        bool addEdge(Edge& e);

        /**
         * update graph: adds a vertexes and edges to the graph.
         * returns true, on success, or false on failure.
         */
        virtual bool updateGraph(const Vertex7D& v);

        /// Optimize graph
        virtual void optimize(void);

	private:
		PoseGraph graph;
};

#endif // GRAPH_G2O_H_INCLUDED
