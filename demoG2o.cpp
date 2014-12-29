#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/depthSensorModel.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>
#include <atomic>
#include "include/Grabber/kinectGrabber.h"

using namespace std;

Graph * graph;

auto startT = std::chrono::high_resolution_clock::now();

// optimization thread
void optimize(){
    //optimize
    graph->optimize(70);
}

//simulates tracking module
void tracker()
{
    using namespace std::chrono;
    //add vertex or edge

    //add 3D feature
    for (int i=0;i<5;i++){
        Vec3 pos11(0.05, 1.0+i*0.05, 0.0);
        Vertex3D vertex6(5+i, pos11);
        vertex6.timestamp = duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexFeature(vertex6))
            std::cout << "error: vertex exists!\n";
    }

    //add edges of the graph -- measurements
    Vec3 pos12(-1.05, 0.22, 0.33);
    Mat33 infoMat12; infoMat12.setIdentity();
    Edge3D edge6(pos12,infoMat12, 1,5);
    if (!graph->addEdge3D(edge6))
        std::cout << "error: vertex doesn't exist!\n";

    for (int i=0;i<5;i++){
        Vec3 pos13(1.05+i*0.05, 0.0, 0.0);
        Mat33 infoMat13; infoMat13.setIdentity();
        Edge3D edge7(pos13,infoMat13, 2,5);
        if (!graph->addEdge3D(edge7))
            std::cout << "error: vertex doesn't exist!\n";
    }

}

int main()
{
    try {
        using namespace putslam;
        using namespace std::chrono;

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
        DepthSensorModel sensorModel(configFile);
        graph = createPoseGraphG2O(sensorModel.config.pose);
        cout << "Current graph: " << graph->getName() << std::endl;

        //add vertices - robot poses
        Vec3 pos1(0.0, 0.0, 0.0);  Quaternion rot1(1, 0, 0, 0);
        VertexSE3 vertex1(0, pos1, rot1);
        vertex1.timestamp = duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex1))
            std::cout << "error: vertex exists!\n";
        Vec3 pos2(0.0, 2.0, 0.0);  Quaternion rot2(1, 0, 0, 0);
        VertexSE3 vertex2(1, pos2, rot2);
        vertex2.timestamp = duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex2))
            std::cout << "error: vertex exists!\n";
        Vec3 pos3(-2.0, 2.0, 0.0);  Quaternion rot3(1, 0, 0, 0);
        VertexSE3 vertex3(2, pos3, rot3);
        vertex3.timestamp = duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex3))
            std::cout << "error: vertex exists!\n";
        Vec3 pos4(-2.0, 0.0, 0.0);  Quaternion rot4(1, 0, 0, 0);
        VertexSE3 vertex4(3, pos4, rot4);
        vertex4.timestamp = duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex4))
            std::cout << "error: vertex exists!\n";

        //add edges of the graph - eg. odometry measurements
        Vec3 pos5(0.01, 1.06, 0.0); Quaternion rot5(0.999902, -0.0107791, 0.00867285, -0.00190021);
        RobotPose trans1(pos5, rot5);
        Mat66 infoMat5; infoMat5.setIdentity();
        EdgeSE3 edge1(trans1,infoMat5,0,1);
        if (!graph->addEdgeSE3(edge1))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos6(-1.15, -0.09, 0.0); Quaternion rot6(0.999989, 0.00272799, -0.000777724, 0.00363979);
        RobotPose trans2(pos6, rot6);
        Mat66 infoMat6; infoMat6.setIdentity();
        EdgeSE3 edge2(trans2,infoMat6,1,2);
        if (!graph->addEdgeSE3(edge2))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos7(0.12, -1.02, 0.0); Quaternion rot7(0.99976, -0.00251893, -0.015912, 0.0148755);
        RobotPose trans3(pos7, rot7);
        Mat66 infoMat7; infoMat7.setIdentity();
        EdgeSE3 edge3(trans3,infoMat7,2,3);
        if (!graph->addEdgeSE3(edge3))
            std::cout << "error: vertex doesn't exist!\n";

        //add 3D feature
        Vec3 pos8(0.05, 1.0, 0.0);
        Vertex3D vertex5(4, pos8);
        vertex5.timestamp = duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexFeature(vertex5))
            std::cout << "error: vertex exists!\n";
        //add edges of the graph -- measurements
        Mat33 infoMat9;        Eigen::Vector3d point;    Mat33 covMat9;
        sensorModel.getPoint(320, 375, 5.1, point);
        Vec3 pos9(point(0), point(1), point(2));
        sensorModel.computeCov(320, 375, 5.1, covMat9);
        std::cout <<  infoMat9.inverse() << std::endl;
        infoMat9 = covMat9.inverse();
        Edge3D edge4(pos9,infoMat9, 0,4);
        if (!graph->addEdge3D(edge4))
            std::cout << "error: vertex doesn't exist!\n";

        Mat33 infoMat10; infoMat10.setIdentity();     Mat33 covMat10;
        sensorModel.getPoint(320, 448, 3.16, point);
        Vec3 pos10(point(0), point(1), point(2));
        sensorModel.computeCov(320, 448, 3.16, covMat10);
        infoMat10 = covMat10.inverse();
        Edge3D edge5(pos10,infoMat10, 1,4);
        if (!graph->addEdge3D(edge5))
            std::cout << "error: vertex doesn't exist!\n";

        // save current graph to file
        // to view run ./g2o_viewer initGraph.g2o
        graph->save2file("initGraph.g2o");

        //optimize
        std::thread tOpt(optimize);

        //start tracking thread
        std::thread tTracker(tracker);

        tOpt.join();
        tTracker.join();

        // graph pruning
        //graph->optimizeAndPrune(0.5, 10);

        ///checking export/import methods

        // save optimal graph to file
        // to view run ./g2o_viewer optimalGraph.g2o
        graph->save2file("optimalGraph.g2o");

        // export to RGB-D SLAM format
        graph->export2RGBDSLAM("output_trajectory.graph");

        // import from RGB-D SLAM
        graph->importRGBDSLAM("output_trajectory.graph");

        // export to RGB-D SLAM format
        graph->export2RGBDSLAM("output_trajectory1.graph");

        // save graph to file
        graph->save2file("optimalGraph1.g2o");

        //clear the graph
        graph->clear();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
