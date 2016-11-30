/** @file weightedGraph.cpp
 *
 * Weighted Graph
 * \author Dominik Belter
 */

#include "PoseGraph/weightedGraph.h"
#include <iostream>

using namespace putslam;
int WeightedEdge::idCounter=0;

/// add vertex
bool WeightedGraph::addVertex(int vertexId){
    auto got = vertices.find(vertexId);
    if ( got != vertices.end() ){
        return false;
    }
    else {
        vertices.insert(std::make_pair(vertexId,std::shared_ptr<WeightedVertex>(new WeightedVertex(vertexId))));
        return true;
    }
}

/// add edge to vertex
bool WeightedGraph::addEdgeToVertex(int vertexId, const std::shared_ptr<WeightedEdge>& e){
    auto got = vertices.find(vertexId);
    if ( got == vertices.end() ){
        return false;
    }
    else {
        got->second->addEdge(e);
        return true;
    }
}

/// add vertex
bool WeightedGraph::addEdge(const WeightedEdge& e){
    edges.insert(edges.end(),std::shared_ptr<WeightedEdge>(new WeightedEdge(e)));
    std::pair<int,int> verts = e.getVertices();

    //std::cout << "vertices " << verts.first << ", " << verts.second << "\n";
    if (!addEdgeToVertex(verts.first,edges.back())) return false;
    if (!addEdgeToVertex(verts.second,edges.back())) return false;
    //std::cout << "covisibility graph size: " << vertices.size() << "\n";
    //std::cout << "edges: " << edges.size() << " \n";
    return true; // DB: Check that!
}

/// find neighbouring vertices
void WeightedGraph::findNeighbouringNodes(int id, double covisibilityThr, std::set<int>& verticesIds){
    WeightedVertex* vert = vertices.at(id).get();
    WeightedGraph::EdgeSet edges;
    vert->getEdges(edges);
    verticesIds.clear();
    std::vector<std::pair<int,double>> neighbours;
    vert->getNeighbours(neighbours);
    for (auto neighbour : neighbours){
        if (neighbour.second>covisibilityThr)
            verticesIds.insert(neighbour.first);
    }
}

/// get neighbours
void WeightedVertex::getNeighbours(std::vector<std::pair<int,double>>& neighbours){
    for (auto edge : edges){
        int vertId;
        std::pair<int,int> vertices = edge->getVertices();
        if (vertices.first!=id)
            vertId = vertices.first;
        else if (vertices.second!=id)
            vertId = vertices.second;
        else
            throw std::runtime_error(std::string("Something is wrong with edges in weighted graph\n"));
        neighbours.push_back(std::make_pair(vertId,edge->getWeight()));
    }
}

