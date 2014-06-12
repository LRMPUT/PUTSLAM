#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>
#include <atomic>

using namespace std;

Graph * graph;

auto startT = std::chrono::high_resolution_clock::now();

// optimization thread
void optimize(){
    // graph pruning and optimization
    //graph->optimize(70);
    graph->optimizeAndPrune(10.0, 70);
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
        KinectGrabber::UncertaintyModel sensorModel(configFile);
        graph = createPoseGraphG2O(sensorModel.config.pose);

        //load graph from file
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/TrackingOnFeatures/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/Matching/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/TrackingWithBundleAdjustment/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/MatchingwithBundleAdjustment/graphFile.g2o");
        //((PoseGraphG2O *)graph)->loadG2O("graphMN/room/TrackingWithBundleAdjustment/graphFile.g2o");
        ((PoseGraphG2O *)graph)->loadG2O("graphMN/room/MatchingWithBundleAdjustment/graphFile.g2o");

        //save init graph
        //graph->save2file("graphMN/TrackingOnFeatures/graphFile_test.g2o");
        //graph->save2file("graphMN/Matching/graphFile_test.g2o");
        //graph->save2file("graphMN/TrackingWithBundleAdjustment/graphFile_test.g2o");
        //graph->save2file("graphMN/MatchingwithBundleAdjustment/graphFile_test.g2o");
        //graph->save2file("graphMN/room/TrackingWithBundleAdjustment/graphFile_test.g2o");
        graph->save2file("graphMN/room/MatchingWithBundleAdjustment/graphFile_test.g2o");

        //optimize
        std::thread tOpt(optimize);

        tOpt.join();

        ///checking export/import methods

        // save optimal graph to file
        // to view run ./g2o_viewer optimalGraph.g2o
        //graph->save2file("graphMN/TrackingOnFeatures/graphFilePruned.g2o");
        //graph->save2file("graphMN/Matching/graphFilePruned.g2o");
        //graph->save2file("graphMN/TrackingWithBundleAdjustment/graphFilePruned.g2o");
        //graph->save2file("graphMN/MatchingwithBundleAdjustment/graphFilePruned.g2o");
        //graph->save2file("graphMN/room/TrackingWithBundleAdjustment/graphFilePruned.g2o");
        graph->save2file("graphMN/room/MatchingWithBundleAdjustment/graphFilePruned.g2o");

        //clear the graph
        graph->clear();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
