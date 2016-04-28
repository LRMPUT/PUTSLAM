#include "../include/PoseGraph/weightedGraph.h"
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
    std::cout << "vertices " << verts.first << ", " << verts.second << "\n";
    if (!addEdgeToVertex(verts.first,edges.back())) return false;
    if (!addEdgeToVertex(verts.second,edges.back())) return false;
    std::cout << "covisibility graph size: " << vertices.size() << "\n";
    std::cout << "edges: " << edges.size() << " \n";
}

