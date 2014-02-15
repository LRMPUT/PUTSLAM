#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "Core/Math/CMat44.h"
#include "Core/Tools/XMLParserCV.h"
#include <cmath>

using namespace std;



int main()
{
    try {
        using namespace putslam;

        Parser* XMLparser = createXMLParserCV("configGlobal.xml");

        Graph * graph = createPoseGraphG2O();
        cout << "Current graph: " << graph->getName() << std::endl;

        //add vertices - robot poses
        Vec3 pos1(1.0, -1.0, 0.0);  Quaternion rot1(1, 0, 0, 0);
        VertexSE3 vertex1(0, pos1, rot1);
        if (!graph->addVertexPose(vertex1))
            std::cout << "error: vertex exists!\n";
        Vec3 pos2(1.0, 1.0, 0.0);  Quaternion rot2(1, 0, 0, 0);
        VertexSE3 vertex2(1, pos2, rot2);
        if (!graph->addVertexPose(vertex2))
            std::cout << "error: vertex exists!\n";
        Vec3 pos3(-1.0, 1.2, 0.0);  Quaternion rot3(1, 0, 0, 0);
        VertexSE3 vertex3(2, pos3, rot3);
        if (!graph->addVertexPose(vertex3))
            std::cout << "error: vertex exists!\n";
        Vec3 pos4(-1.0, -1.0, 0.0);  Quaternion rot4(1, 0, 0, 0);
        VertexSE3 vertex4(3, pos4, rot4);
        if (!graph->addVertexPose(vertex4))
            std::cout << "error: vertex exists!\n";

        //add edges of the graph - eg. odometry measurements
        Vec3 pos5(0.0, 1.0, 0.0); Quaternion rot5(1, 0, 0, 0);
        RobotPose trans1(pos5, rot5);
        Mat66 infoMat5; infoMat5.setIdentity();
        EdgeSE3 edge1(trans1,infoMat5,0,1);
        if (!graph->addEdgeSE3(edge1))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos6(-1.0, 0.0, 0.0); Quaternion rot6(1, 0, 0, 0);
        RobotPose trans2(pos6, rot6);
        Mat66 infoMat6; infoMat6.setIdentity();
        EdgeSE3 edge2(trans2,infoMat6,1,2);
        if (!graph->addEdgeSE3(edge2))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos7(0.0, -1.0, 0.0); Quaternion rot7(1, 0, 0, 0);
        RobotPose trans3(pos7, rot7);
        Mat66 infoMat7; infoMat7.setIdentity();
        EdgeSE3 edge3(trans3,infoMat7,2,3);
        if (!graph->addEdgeSE3(edge3))
            std::cout << "error: vertex doesn't exist!\n";

        //add 3D feature
        Vec3 pos8(1.05, 0.0, 0.0);
        Vertex3D vertex5(4, pos8);
        if (!graph->addVertexFeature(vertex5))
            std::cout << "error: vertex exists!\n";
        //add edges of the graph -- measurements
        Vec3 pos9(0.0, 1.0, 0.0);
        Mat33 infoMat9; infoMat9.setIdentity();
        Edge3D edge4(pos9,infoMat9, 0,4);
        if (!graph->addEdge3D(edge4))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos10(0.0, -1.0, 0.0);
        Mat33 infoMat10; infoMat10.setIdentity();
        Edge3D edge5(pos10,infoMat10, 1,4);
        if (!graph->addEdge3D(edge5))
            std::cout << "error: vertex doesn't exist!\n";

        // save current graph to file
        // to view run ./g2o_viewer initGraph.g2o
        graph->save2file("initGraph.g2o");

        //optimize
        graph->optimize(10);

        // save optimal graph to file
        // to view run ./g2o_viewer optimalGraph.g2o
        graph->save2file("optimalGraph.g2o");

    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
