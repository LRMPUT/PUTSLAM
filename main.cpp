#include <iostream>
#include <thread>
#include "include/putslam/Defs/putslam_defs.h"
#include "Grabber/depthSensorModel.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>
#include <ctime>
#include <ratio>
#include <chrono>
#include "include/putslam/Grabber/kinectGrabber.h"
#include "include/putslam/Grabber/xtionGrabber.h"

using namespace std;

std::unique_ptr<std::thread> thread_poseGraph;
std::unique_ptr<std::thread> thread_globalGraph;

void globalGraphUpdate(Graph* global_graph, const VertexSE3& transform){
    global_graph->addVertexPose(transform); //update graph
    global_graph->optimize(10); // loop closure detection

}

void poseGraphUpdate(Graph* graph, Graph* global_graph, const VertexSE3& transform){
    if(graph->addVertexPose(transform)){ //detect previously visited places and update graph (add vertex or node to previously visited vertex)
        if (thread_globalGraph) {
            thread_globalGraph->join(); //wait until global graph thread is comleted (it should be considered as an error)
            thread_globalGraph.release(); //release object (is it possible to start thread without 'new'?)
        }
        thread_globalGraph = std::unique_ptr<std::thread> (new std::thread(&globalGraphUpdate, global_graph, transform)); // throw thread
        graph->optimize(10);
    }
}

unsigned const max_tracking_duration = 6;//seconds

int main()
{
    try {
        using namespace putslam;

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string grabberType(config.FirstChildElement( "Grabber" )->FirstChildElement( "name" )->GetText());

        Grabber* grabber;
        if (grabberType == "Kinect") {
            std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
            grabber = createGrabberKinect(configFile, Grabber::MODE_BUFFER);
        }
        else if (grabberType == "Xtion") {
            std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
            grabber = createGrabberXtion(configFile, Grabber::MODE_BUFFER);
        }
        else if (grabberType == "MesaImaging")
            grabber = createGrabberKinect();
        else // Default
            grabber = createGrabberKinect();

        Mat33 cov;
        DepthSensorModel KinectModel("../../resources/KinectModel.xml");
        KinectModel.computeCov(80, 360, 0.5837, cov);
        Eigen::Vector3d vec;
        KinectModel.getPoint((long int)377.177, (long int)112.906, 6.468, vec);

        // create objects and print configuration
        cout << "Current grabber: " << grabber->getName() << std::endl;
        Tracker * tracker = createTrackerKLT();
        cout << "Current tracker: " << tracker->getName() << std::endl;
        Graph * graph = createPoseGraphG2O();
        cout << "Current graph: " << graph->getName() << std::endl;
        Graph * global_graph = createGlobalGraph();
        cout << "Current global graph: " << global_graph->getName() << std::endl;

        auto start = chrono::system_clock::now();
        while (1){ //tracking
            grabber->grab(); // grab frame
            if (!tracker->track(grabber->getSensorFrame())) { //check if tracker should start new tracking
                if (thread_poseGraph) {
                    thread_poseGraph->join(); //wait until pose graph thread is comleted (it should be considered as an error)
                    thread_poseGraph.release(); //release object (is it possible to start thread without 'new'?)
                }
                thread_poseGraph = unique_ptr<thread>(new thread(&poseGraphUpdate, graph, global_graph, tracker->getVertex())); // throw thread
                tracker->reset();
            }
            if (chrono::duration_cast<chrono::duration<unsigned> >(chrono::system_clock::now() - start).count()>max_tracking_duration){
                thread_poseGraph->join();
                thread_globalGraph->join();
                break;
            }
        }

    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}

	return 0;
}
