/** @file graph.h
 *
 * Pose Graph interface
 *
 */

#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Graph interface
    class Graph {
        public:

            /// overloaded constructor
            Graph (const std::string _name) : name(_name) {}

            /// Name of the graph
            virtual const std::string& getName() const = 0;

            /// Removes a vertex from the graph. Returns true on success
            virtual bool removeVertex(Vertex* v) = 0;

            /// removes an edge from the graph. Returns true on success
            virtual bool removeEdge(Edge* e) = 0;

            /// clears the graph and empties all structures.
            virtual void clear() = 0;

            /// @returns the map <i>id -> vertex</i> where the vertices are stored
            virtual const PoseGraph::VertexSet& vertices() const = 0;

            //! @returns the set of edges of the hyper graph
            virtual const PoseGraph::EdgeSet& edges() const = 0;

            /**
             * adds a vertex to the graph.
             * returns true, on success, or false on failure.
             */
            virtual bool addVertex(Vertex* v) = 0;

            /**
             * Adds an edge  to the graph. If the edge is already in the graph, it
             * does nothing and returns false. Otherwise it returns true.
             */
            virtual bool addEdge(Edge* e) = 0;

            /// Virtual descrutor
            virtual ~Graph() {}

        protected:
            /// Graph
            PoseGraph graph;
            /// Graph name
            const std::string name;
    };
};

#endif // _GRAPH_H_
