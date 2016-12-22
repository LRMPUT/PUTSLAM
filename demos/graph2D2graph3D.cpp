#include <iostream>
#include <thread>
#include "Defs/putslam_defs.h"
#include "PoseGraph/graph_g2o.h"
#include "Utilities/CLParser.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <cmath>

using namespace std;

int main(int argc, char * argv[])
{
    try {
        using namespace putslam;

        CLParser cmd_line(argc,argv,true);
        if (cmd_line.get_arg("-h").size())
            std::cout << "To run type: ./graph2D2graph3D -i input2Dgraph.g2o -o output3Dgraph.g2o\n";

        // load graph
        /*tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/putslamconfigGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
        KinectGrabber::UncertaintyModel sensorModel(configFile);*/

        Mat34 pose; pose.setIdentity();
        Graph* graph = createPoseGraphG2O(pose);
        if (cmd_line.get_arg("-i").length()!=0){
            std::cout << cmd_line.get_arg("-i") << "\n";
            graph->load(cmd_line.get_arg("-i"));
        }
        else {
            std::cout << "No input file specified (-i fileneme)\n";
            return 0;
        }

        //do conversion
        PoseGraph::VertexSet vertices = graph->getVertices();
        PoseGraph::EdgeSet edges = graph->getEdges();
        //std::cout << "//do conversion1\n";
        //std::cout << vertices.size() << "\n";
        graph->clear();
        int vertexNo = 0;
        for (PoseGraph::VertexSet::iterator it = vertices.begin();it!=vertices.end();it++){
            if (it->get()->type==Vertex::VERTEXSE2){
                Vec3 pos(((VertexSE2*)it->get())->pos(0), ((VertexSE2*)it->get())->pos(1), 0.0);
                Eigen::Matrix3d pose; pose.setIdentity();
                pose(0,0) = cos(((VertexSE2*)it->get())->theta);
                pose(0,1) = -sin(((VertexSE2*)it->get())->theta);
                pose(1,0) = sin(((VertexSE2*)it->get())->theta);
                pose(1,1) = cos(((VertexSE2*)it->get())->theta);
                Quaternion rot(pose);
                VertexSE3 vertex(vertexNo, Mat34(pos * rot));
                if (!graph->addVertexPose(vertex))
                    std::cout << "error: vertex exists!\n";
                vertexNo++;
            }
        }

        for (PoseGraph::EdgeSet::iterator it = edges.begin();it!=edges.end();it++){
            if (it->get()->type==Edge::EDGE_SE2){
                Vec3 pos(((EdgeSE2*)it->get())->trans(0), ((EdgeSE2*)it->get())->trans(1), 0.0);
                Eigen::Matrix3d pose; pose.setIdentity();
                pose(0,0) = cos(((VertexSE2*)it->get())->theta);
                pose(0,1) = -sin(((VertexSE2*)it->get())->theta);
                pose(1,0) = sin(((VertexSE2*)it->get())->theta);
                pose(1,1) = cos(((VertexSE2*)it->get())->theta);
                Quaternion quat(pose);
                Mat34 trans(pos* quat);
                Mat66 infoMat; infoMat.setIdentity();
                infoMat(0,0) = ((EdgeSE2*)it->get())->info(0,0);
                infoMat(0,1) = ((EdgeSE2*)it->get())->info(0,1);
                infoMat(1,0) = ((EdgeSE2*)it->get())->info(1,0);
                infoMat(1,1) = ((EdgeSE2*)it->get())->info(1,1);
                //!do something with orientation

                EdgeSE3 edge(trans,infoMat,((EdgeSE2*)it->get())->fromVertexId,((EdgeSE2*)it->get())->toVertexId);
                if (!graph->addEdgeSE3(edge))
                    std::cout << "error: edge doesn't exist!\n";
            }
        }

        if (cmd_line.get_arg("-o").length()!=0){
            graph->save2file(cmd_line.get_arg("-o"));
        }
        else {
            std::cout << "No output file specified (-o fileneme)\n";
            return 0;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
