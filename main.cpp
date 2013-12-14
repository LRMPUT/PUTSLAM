#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "Tracker/trackerKLT.h"
#include "Core/Math/CMat44.h"
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

void tracking(Grabber* grabber, Tracker* tracker, Graph* graph, Graph* global_graph){

    while (1){
        grabber->grab(); // grab frame
        if (!tracker->track(grabber->getImage())) { //check if tracker should start new tracking
            if (thread_poseGraph) {
                thread_poseGraph->join(); //wait until pose graph thread is comleted (it should be considered as an error)
                thread_poseGraph.release(); //release object (is it possible to start thread without 'new'?)
            }
            thread_poseGraph = std::unique_ptr<std::thread>(new std::thread(&poseGraphUpdate, graph, global_graph, tracker->getVertex())); // throw thread
            tracker->reset();
        }
    }
}

int main()
{
    try {
        using namespace putslam;

        Grabber* grabber = createGrabberKinect();
        cout << "Current grabber: " << grabber->getName() << std::endl;
        Tracker * tracker = createTrackerKLT();
        Graph * graph = createPoseGraphG2O();
        Graph * global_graph = createGlobalGraph();

        std::thread thread_tracking(&tracking, grabber, tracker, graph, global_graph); // throw thread

        thread_tracking.join();
    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}

	return 0;
}
