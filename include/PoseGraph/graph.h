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

            /// clears the graph and empties all structures.
            virtual void clear() = 0;

            /**
             * adds a vertex to the graph - feature
             * returns true, on success, or false on failure.
             */
            virtual bool addVertexFeature(const Vertex3D& v) = 0;

            /**
             * adds a vertex to the graph - feature
             * returns true, on success, or false on failure.
             */
            virtual bool addVertexPose(const VertexSE3& v) = 0;

            /**
             * Adds an SE3 edge to the graph. If the edge is already in the graph, it
             * does nothing and returns false. Otherwise it returns true.
             */
            virtual bool addEdgeSE3(const EdgeSE3& e) = 0;

            /**
             * Adds an 3D edge to the graph. If the edge is already in the graph, it
             * does nothing and returns false. Otherwise it returns true.
             */
            virtual bool addEdge3D(const Edge3D& e) = 0;

            /// Optimize graph
            virtual void optimize(uint_fast32_t maxIterations) = 0;

            /// Save graph to file
            virtual void save2file(std::string filename) const = 0;

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
