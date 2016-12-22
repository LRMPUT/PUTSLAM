#include <iostream>
#include <thread>
#include "Defs/putslam_defs.h"
#include "Grabber/depthSensorModel.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <cmath>
#include <atomic>
#include "Grabber/kinectGrabber.h"

using namespace std;

Graph * graph;

auto startT = std::chrono::high_resolution_clock::now();

// optimization thread
void optimize(){
    //optimize
    graph->optimize(70,1);
    //((PoseGraphG2O *)graph)->fixOptimizedVertices();
}

//simulates tracking module
void tracker()
{
    using namespace std::chrono;
    //add vertex or edge

    //add 3D feature
    for (int i=0;i<5000;i++){
        Vertex3D vertex6(5+i, Vec3(0.05, 1.0+i*0.05, 0.0));
        vertex6.timestamp = (double) duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexFeature(vertex6))
            std::cout << "error: vertex exists!\n";
    }

    //add edges of the graph -- measurements
    Edge3D edge6(Vec3(-1.05, 0.22, 0.33), Mat33::Identity(), 1,5);
    if (!graph->addEdge3D(edge6))
        std::cout << "error: vertex doesn't exist!\n";

    for (int i=0;i<5000;i++){
        Edge3D edge7(Vec3(1.05+i*0.05, 0.0, 0.0), Mat33::Identity(), 2,5);
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
        config.LoadFile("../../resources/putslamconfigGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
        DepthSensorModel sensorModel(configFile);
        graph = createPoseGraphG2O(sensorModel.config.pose);
        cout << "Current graph: " << graph->getName() << std::endl;

        //add vertices - robot poses
        VertexSE3 vertex1(0, Mat34::Identity());
        vertex1.timestamp = (double) duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex1))
            std::cout << "error: vertex exists!\n";

        VertexSE3 vertex2(1, Mat34(Vec3(0.0, 2.0, 0.0)*Quaternion(1, 0, 0, 0)));
        vertex2.timestamp = (double) duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex2))
            std::cout << "error: vertex exists!\n";

        VertexSE3 vertex3(2, Mat34(Vec3(-2.0, 2.0, 0.0) * Quaternion(1, 0, 0, 0)));
        vertex3.timestamp = (double) duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex3))
            std::cout << "error: vertex exists!\n";

        VertexSE3 vertex4(3, Mat34(Vec3(-2.0, 0.0, 0.0) * Quaternion(1, 0, 0, 0)));
        vertex4.timestamp = (double) duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexPose(vertex4))
            std::cout << "error: vertex exists!\n";

        //add edges of the graph - eg. odometry measurements
        EdgeSE3 edge1(Mat34(Vec3(0.01, 1.06, 0.0) * Quaternion(0.999902, -0.0107791, 0.00867285, -0.00190021)), Mat66::Identity(),0,1);
        if (!graph->addEdgeSE3(edge1))
            std::cout << "error: vertex doesn't exist!\n";

        EdgeSE3 edge2(Mat34(Vec3(-1.15, -0.09, 0.0) * Quaternion(0.999989, 0.00272799, -0.000777724, 0.00363979)), Mat66::Identity(),1,2);
        if (!graph->addEdgeSE3(edge2))
            std::cout << "error: vertex doesn't exist!\n";

        EdgeSE3 edge3(Mat34 (Vec3(0.12, -1.02, 0.0) * Quaternion (0.99976, -0.00251893, -0.015912, 0.0148755)), Mat66::Identity(),2,3);
        if (!graph->addEdgeSE3(edge3))
            std::cout << "error: vertex doesn't exist!\n";

        //add 3D feature
        Vertex3D vertex5(4, Vec3 (0.05, 1.0, 0.0));
        vertex5.timestamp = (double) duration_cast<std::chrono::microseconds> (high_resolution_clock::now() - startT).count();
        if (!graph->addVertexFeature(vertex5))
            std::cout << "error: vertex exists!\n";
        //add edges of the graph -- measurements
        Mat33 infoMat9;        Eigen::Vector3d point;    Mat33 covMat9;
        sensorModel.getPoint(320, 375, 5.1, point);//get Euclidean coordinates

        sensorModel.computeCov(320, 375, 5.1, covMat9);//compute covariance of measurement
        std::cout <<  infoMat9.inverse() << std::endl;
        Edge3D edge4(Vec3(point(0), point(1), point(2)), covMat9.inverse(), 0,4);
        if (!graph->addEdge3D(edge4))
            std::cout << "error: vertex doesn't exist!\n";

        Mat33 infoMat10; infoMat10.setIdentity();     Mat33 covMat10;
        sensorModel.getPoint(320, 448, 3.16, point);
        sensorModel.computeCov(320, 448, 3.16, covMat10);
        Edge3D edge5(Vec3 (point(0), point(1), point(2)), covMat10.inverse(), 1,4);
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


        //optimize
        std::thread tOpt2(optimize);
        tOpt2.join();

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
