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

std::unique_ptr<std::thread> thread_poseGraph;
std::unique_ptr<std::thread> thread_globalGraph;

void globalGraphUpdate(Graph* global_graph, const Vertex7D& transform){
    global_graph->updateGraph(transform); //update graph
    global_graph->optimize(); // loop closure detection

}

void poseGraphUpdate(Graph* graph, Graph* global_graph, const Vertex7D& transform){
    if(graph->updateGraph(transform)){ //detect previously visited places and update graph (add vertex or node to previously visited vertex)
        if (thread_globalGraph) {
            thread_globalGraph->join(); //wait until global graph thread is comleted (it should be considered as an error)
            thread_globalGraph.release(); //release object (is it possible to start thread without 'new'?)
        }
        thread_globalGraph = std::unique_ptr<std::thread> (new std::thread(&globalGraphUpdate, global_graph, transform)); // throw thread
        graph->optimize();
    }
}

unsigned const max_tracking_duration = 6;

int main()
{
    try {
        using namespace putslam;

        Parser* XMLparser = createXMLParserCV("configGlobal.xml");

        std::string grabber_type = XMLparser->getAttribute("Grabber", "name");
        Grabber* grabber;
        if (grabber_type == "Kinect")
            grabber = createGrabberKinect();
        else if (grabber_type == "MesaImaging")
            grabber = createGrabberKinect();
        else // Default
            Grabber* grabber = createGrabberKinect();

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
            if (!tracker->track(grabber->getImage())) { //check if tracker should start new tracking
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
