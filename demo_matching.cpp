#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Grabber/xtion_grabber.h"
#include "Grabber/file_grabber.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <opencv/highgui.h>
#include <cmath>
#include <ctime>
#include <ratio>
#include <chrono>

#include "include/Grabber/file_grabber.h"
#include "include/Matcher/matcherOpenCV.h"

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
        /// Still do not take into account the config file
        else if (grabberType == "File") {
			std::string configFile(
					config.FirstChildElement("Grabber")->FirstChildElement(
							"calibrationFile")->GetText());
			grabber = createGrabberFile(configFile);
		}
        else if (grabberType == "MesaImaging")
            grabber = createGrabberKinect();
        else // Default
            grabber = createGrabberKinect();

        // create objects and print configuration
        cout << "Current grabber: " << grabber->getName() << std::endl;
        string matcherParameters = config.FirstChildElement("Matcher")->FirstChildElement("parametersFile")->GetText();

        Matcher * matcher = createMatcherOpenCV(matcherParameters);
        cout << "Current matcher: " << matcher->getName() << std::endl;
        Graph * graph = createPoseGraphG2O();
        cout << "Current graph: " << graph->getName() << std::endl;
        Graph * global_graph = createGlobalGraph();
		cout << "Current global graph: " << global_graph->getName()
				<< std::endl;

		auto start = chrono::system_clock::now();
		bool ifStart = true;
		// Main loop
		while (true) {
			grabber->grab(); // grab frame
			SensorFrame currentSensorFrame = grabber->getSensorFrame();

			if ( ifStart )
			{
				matcher->Matcher::loadInitFeatures(currentSensorFrame);
				ifStart = false;
				//break;
			}
			else
			{
				Eigen::Matrix4f transformation;
				matcher->Matcher::match(currentSensorFrame, transformation);

				std::cout<<std::endl<<transformation<<std::endl;
				break;
			}


			//imshow("1",a.image);
			//imshow("2",a.depth);
            cvWaitKey(500);

//			if (!tracker->track(grabber->getSensorFrame())) { //check if tracker should start new tracking
//				if (thread_poseGraph) {
//					thread_poseGraph->join(); //wait until pose graph thread is comleted (it should be considered as an error)
//					thread_poseGraph.release(); //release object (is it possible to start thread without 'new'?)
//				}
//
//				// MATCHING ADD RESULT
//
//
//				thread_poseGraph = unique_ptr < thread
//						> (new thread(&poseGraphUpdate, graph, global_graph,
//								tracker->getVertex())); // throw thread
//				tracker->reset();
//			}
			/*if (chrono::duration_cast < chrono::duration<unsigned>
					> (chrono::system_clock::now() - start).count()
					> max_tracking_duration) {
				thread_poseGraph->join();
				thread_globalGraph->join();
				break;
			}*/
		}

    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}

	return 0;
}
