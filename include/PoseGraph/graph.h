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
#include <thread>
#include <mutex>

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
             * adds a vertex to the graph - pose
             * returns true, on success, or false on failure.
             */
            virtual bool addVertexPose(const VertexSE3& v) = 0;

            /**
             * adds a vertex to the graph - x,y,theta
             * returns true, on success, or false on failure.
             */
            virtual bool addVertexSE2(const VertexSE2& v) = 0;

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

            /**
             * Adds an SE2 edge to the graph. If the edge is already in the graph, it
             * does nothing and returns false. Otherwise it returns true.
             */
            virtual bool addEdgeSE2(const EdgeSE2& e) = 0;

            /// Optimize graph
            virtual bool optimize(uint_fast32_t maxIterations) = 0;

            /// Save graph to file
            virtual void save2file(const std::string filename) const = 0;

            /// Load graph from file
            virtual void load(const std::string filename) = 0;

            /// Export camera path to file (RGB-D SLAM format)
            virtual void export2RGBDSLAM(const std::string filename) const = 0;

            /// Import camera path from file (RGB-D SLAM format)
            virtual bool importRGBDSLAM(const std::string filename) = 0;

            /// Removes weak edes (with error bigger than threshold
            virtual bool optimizeAndPrune(float_type threshold, unsigned int singleIteration) = 0;

            /// Removes weak edes (with error bigger than threshold
            virtual bool optimizeAndPrune2(float_type threshold, unsigned int singleIteration) = 0;

            /// Return trajectory (set of SE3 poses)
            virtual std::vector<Mat34> getTrajectory(void) const = 0;

            /// Export graph to m-file
            void plot2file(const std::string filename, const std::string pointPropertySE3 = "'o'", const std::string pointProperty3D = "'.'", const std::string linePropertySE3 = "'g','LineWidth',1", const std::string lineProperty3D = "'g','LineWidth',1", const std::string prunedEdgesPropertySE3 = "'--r','LineWidth',2", const std::string prunedEdgesProperty3D = "'--r','LineWidth',1"){
                std::ofstream file(filename);
                file << "close all; clear all;\n";
                file << "hold on;\n";
                for (putslam::PoseGraph::VertexSet::const_iterator it = graph.vertices.begin(); it!=graph.vertices.end();it++){
                    if (it->get()->type==Vertex::VERTEXSE3)
                        file << "plot3(" << std::setprecision (5) << ((putslam::VertexSE3*)it->get())->nodeSE3.pos.x() << ", " << ((putslam::VertexSE3*)it->get())->nodeSE3.pos.y() << ", " << ((putslam::VertexSE3*)it->get())->nodeSE3.pos.z() << ", " << pointPropertySE3 << ");\n";
                    else if (it->get()->type==Vertex::VERTEX3D)
                        file << "plot3(" << std::setprecision (5) << ((putslam::Vertex3D*)it->get())->keypoint.depthFeature.x() << ", " << ((putslam::Vertex3D*)it->get())->keypoint.depthFeature.y() << ", " << ((putslam::Vertex3D*)it->get())->keypoint.depthFeature.z() << ", " << pointProperty3D << ");\n";
                    else if (it->get()->type==Vertex::VERTEXSE2)
                        file << "plot(" << std::setprecision (5) << ((putslam::VertexSE2*)it->get())->pos.x() << ", " << ((putslam::VertexSE2*)it->get())->pos.y() << ", " << pointProperty3D << ");\n";
                }
                for (putslam::PoseGraph::EdgeSet::const_iterator it = graph.edges.begin(); it!=graph.edges.end();it++){
                    putslam::PoseGraph::VertexSet::const_iterator toIt = findVertex(it->get()->toVertexId);
                    putslam::PoseGraph::VertexSet::const_iterator fromIt = findVertex(it->get()->fromVertexId);
                    if (it->get()->type==Edge::EDGE_SE3)
                        file << "plot3([" << std::setprecision (5) << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.x() << ", " << ((putslam::VertexSE3*)toIt->get())->nodeSE3.pos.x() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.y() << ", " << ((putslam::VertexSE3*)toIt->get())->nodeSE3.pos.y() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.z() << ", " << ((putslam::VertexSE3*)toIt->get())->nodeSE3.pos.z() << "], "<< linePropertySE3 << ");\n";
                    else if (it->get()->type==Edge::EDGE_3D)
                        file << "plot3([" << std::setprecision (5) << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.x() << ", " << ((putslam::Vertex3D*)toIt->get())->keypoint.depthFeature.x() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.y() << ", " << ((putslam::Vertex3D*)toIt->get())->keypoint.depthFeature.y() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.z() << ", " << ((putslam::Vertex3D*)toIt->get())->keypoint.depthFeature.z() << "], "<< lineProperty3D << ");\n";
                    else if (it->get()->type==Edge::EDGE_SE2)
                        file << "plot([" << std::setprecision (5) << ((putslam::VertexSE2*)fromIt->get())->pos.x() << ", " << ((putslam::VertexSE2*)toIt->get())->pos.x() << "], [" << ((putslam::VertexSE2*)fromIt->get())->pos.y() << ", " << ((putslam::VertexSE2*)toIt->get())->pos.y() << "], "<< lineProperty3D << ");\n";
                }
                for (putslam::PoseGraph::EdgeSet::const_iterator it = graph.prunedEdges.begin(); it!=graph.prunedEdges.end();it++){
                    putslam::PoseGraph::VertexSet::const_iterator toIt = findVertex(it->get()->toVertexId);
                    putslam::PoseGraph::VertexSet::const_iterator fromIt = findVertex(it->get()->fromVertexId);
                    if (it->get()->type==Edge::EDGE_SE3)
                        file << "plot3([" << std::setprecision (5) << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.x() << ", " << ((putslam::VertexSE3*)toIt->get())->nodeSE3.pos.x() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.y() << ", " << ((putslam::VertexSE3*)toIt->get())->nodeSE3.pos.y() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.z() << ", " << ((putslam::VertexSE3*)toIt->get())->nodeSE3.pos.z() << "], "<< prunedEdgesPropertySE3 << ");\n";
                    else if (it->get()->type==Edge::EDGE_3D)
                        file << "plot3([" << std::setprecision (5) << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.x() << ", " << ((putslam::Vertex3D*)toIt->get())->keypoint.depthFeature.x() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.y() << ", " << ((putslam::Vertex3D*)toIt->get())->keypoint.depthFeature.y() << "], [" << ((putslam::VertexSE3*)fromIt->get())->nodeSE3.pos.z() << ", " << ((putslam::Vertex3D*)toIt->get())->keypoint.depthFeature.z() << "], "<< prunedEdgesProperty3D << ");\n";
                }
                file << "xlabel('x [m]');\n ylabel('y [m]');\n zlabel('z [m]');\n";
                file.close();
            }

            /// Returns set of graph edges
            virtual PoseGraph::EdgeSet getEdges(void) = 0;

            /// Returns set of graph vertices
            virtual PoseGraph::VertexSet getVertices(void) = 0;

            /// Virtual descrutor
            virtual ~Graph() {}

        protected:
            /// Graph
            PoseGraph graph;
            /// Graph name
            const std::string name;
            /// mutex for critical section - graph
            std::recursive_mutex mtxGraph;

            /// Find vertex by id
            PoseGraph::VertexSet::iterator findVertex(unsigned int id){
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
            PoseGraph::EdgeSet::iterator findEdge(unsigned int id){
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

            /// Find all edges which points to the vertex 'toVertexId'
            std::vector<unsigned int> findIncominEdges(unsigned int toVertexId){
                std::vector<unsigned int> edgeIds;
                for (PoseGraph::EdgeSet::iterator it = graph.edges.begin(); it!=graph.edges.end(); it++){
                    if (it->get()->toVertexId == toVertexId)
                        edgeIds.push_back(it->get()->id);
                }
                return edgeIds;
            }
    };
};

#endif // _GRAPH_H_
