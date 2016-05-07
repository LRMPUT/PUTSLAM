/** @file weightedGraph.h
 *
 * Weighted Graph
<<<<<<< HEAD
<<<<<<< HEAD
 *
=======
 * \author Dominik Belter
>>>>>>> 3d7b5dc4acb7309a57e560af665d2ccc2724bfc0
=======
 * \author Dominik Belter
>>>>>>> 3d7b5dc4acb7309a57e560af665d2ccc2724bfc0
 */

#ifndef _WEIGHTEDGRAPH_H_
#define _WEIGHTEDGRAPH_H_

#include "../Defs/putslam_defs.h"
#include <unordered_map>
#include <string>
#include <vector>
#include <list>

namespace putslam {

    class WeightedVertex;
    class WeightedEdge;

    /// WeightedGraph class
    class WeightedGraph {
    public:
        typedef std::list<std::shared_ptr<WeightedEdge>> EdgeSet;
        typedef std::list<std::shared_ptr<WeightedVertex>> VertexSet;
        typedef std::unordered_map<int, std::shared_ptr<WeightedVertex>> VertexIDMap;
        //typedef std::pair<std::shared_ptr<WeightedVertex>,std::shared_ptr<WeightedVertex>> VerticesPair;
        /// add vertex
        bool addVertex(int vertexId);

        /// add edge
        bool addEdge(const WeightedEdge& e);

        /// find neighbouring vertices
        void findNeighbouringNodes(int id, double covisibilityThr, std::set<int>& verticesIds);

        /// add edge to vertex
        bool addEdgeToVertex(int vertexId, const std::shared_ptr<WeightedEdge>& e);

    protected:
        /// Vertices
        VertexIDMap vertices;

        /// Edges
        EdgeSet edges;
    };

    class WeightedEdge {
    public:
        inline WeightedEdge() : id (idCounter++){}
        inline WeightedEdge(double _weight, std::pair<int,int> _vertIds) :
            id(idCounter++), weight(_weight), vertIds(_vertIds){}
        virtual ~WeightedEdge(){}
        /// returns the id
        int getId() const {return id;}
        /// returns vertices
        inline std::pair<int,int> getVertices(void) const {return vertIds;}
        /// returns weight
        inline double getWeight(void) const {return weight;}
    protected :
        /// id
        int id;
        /// cost
        double weight;
        /// vertices
        std::pair<int,int> vertIds;
    private:
        static int idCounter;
    };

    class WeightedVertex {
    public:
        inline WeightedVertex() {}
        inline WeightedVertex(int vertexId) : id(vertexId){}
        virtual ~WeightedVertex(){}
        /// returns the id
        int getId() const {return id;}
        /// add edge
        inline void addEdge(const std::shared_ptr<WeightedEdge>& e) {edges.insert(edges.end(),e);}
        /// get edges
        inline void getEdges(WeightedGraph::EdgeSet& weightEdges) {weightEdges=edges;}
        /// get neighbours
        void getNeighbours(std::vector<std::pair<int,double>>& neighbours);
    protected :
        /// id
        int id;
        /// edges
        WeightedGraph::EdgeSet edges;
    };
}

#endif // _WEIGHTEDGRAPH_H_
