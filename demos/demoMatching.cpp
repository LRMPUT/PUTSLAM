#include <iostream>
#include <thread>
#include "../include/Defs/putslam_defs.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <opencv/highgui.h>
#include <cmath>
#include <ctime>
#include <ratio>
#include <chrono>
#include <fstream>

#include "../include/Grabber/fileGrabber.h"
#include "../include/Grabber/fileGrabber.h"
#include "../include/Grabber/kinectGrabber.h"
#include "../include/Grabber/xtionGrabber.h"
#include "../include/Matcher/matcherOpenCV.h"
#include "../include/Map/featuresMap.h"

using namespace std;

std::unique_ptr<std::thread> thread_poseGraph;
std::unique_ptr<std::thread> thread_globalGraph;

void globalGraphUpdate(Graph* global_graph, const VertexSE3& transform) {
	global_graph->addVertexPose(transform); //update graph
	global_graph->optimize(10); // loop closure detection

}

void poseGraphUpdate(Graph* graph, Graph* global_graph,
		const VertexSE3& transform) {
	if (graph->addVertexPose(transform)) { //detect previously visited places and update graph (add vertex or node to previously visited vertex)
		if (thread_globalGraph) {
			thread_globalGraph->join(); //wait until global graph thread is comleted (it should be considered as an error)
			thread_globalGraph.release(); //release object (is it possible to start thread without 'new'?)
		}
		thread_globalGraph =
				std::unique_ptr < std::thread
						> (new std::thread(&globalGraphUpdate, global_graph,
								transform)); // throw thread
		graph->optimize(10);
	}
}

unsigned const max_tracking_duration = 6; //seconds

void saveTrajectoryFreiburgFormat(Eigen::Matrix4f transformation,
		std::ofstream & estTrajectory, double timestamp) {
	std::ostringstream ossTimestamp;
	ossTimestamp << std::setfill('0') << std::setprecision(17) << timestamp;
	// Saving estimate in Freiburg format
	Eigen::Quaternion<float> Q(transformation.block<3, 3>(0, 0));
	estTrajectory << ossTimestamp.str() << " " << transformation(0, 3) << " "
			<< transformation(1, 3) << " " << transformation(2, 3) << " "
			<< Q.coeffs().x() << " " << Q.coeffs().y() << " " << Q.coeffs().z()
			<< " " << Q.coeffs().w() << endl;
}

int main() {

	using namespace putslam;

	tinyxml2::XMLDocument config;
	config.LoadFile("../../resources/configGlobal.xml");
	if (config.ErrorID())
		std::cout << "unable to load config file.\n";

	// Create map
	std::string configFileGrabber(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
	std::string configFileMap(config.FirstChildElement( "Map" )->FirstChildElement( "parametersFile" )->GetText());
	Map* map = createFeaturesMap(configFileMap, configFileGrabber);

	std::string grabberType(
			config.FirstChildElement("Grabber")->FirstChildElement("name")->GetText());

	Grabber* grabber;
	if (grabberType == "Kinect") {
		std::string configFile(
				config.FirstChildElement("Grabber")->FirstChildElement(
						"calibrationFile")->GetText());
		grabber = createGrabberKinect(configFile, Grabber::MODE_BUFFER);
	} else if (grabberType == "Xtion") {
		std::string configFile(
				config.FirstChildElement("Grabber")->FirstChildElement(
						"calibrationFile")->GetText());
		grabber = createGrabberXtion(configFile, Grabber::MODE_BUFFER);
	}
	/// Still do not take into account the config file
	else if (grabberType == "File") {
		std::string configFile(
				config.FirstChildElement("Grabber")->FirstChildElement(
						"calibrationFile")->GetText());
		grabber = createGrabberFile(configFile);
	} else if (grabberType == "MesaImaging")
		grabber = createGrabberKinect();
	else
		// Default
		grabber = createGrabberKinect();

	// create objects and print configuration
	cout << "Current grabber: " << grabber->getName() << std::endl;
	string matcherParameters =
			config.FirstChildElement("Matcher")->FirstChildElement(
					"parametersFile")->GetText();

	Matcher * matcher = createMatcherOpenCV(matcherParameters);
	cout << "Current matcher: " << matcher->getName() << std::endl;

	// Reading robot starting pose
	Eigen::Matrix4f robotPose = grabber->getStartingSensorPose();

	// File to save trajectory
	ofstream trajectoryFreiburgStream("result/estimatedTrajectory");

	auto start = chrono::system_clock::now();
	bool ifStart = true;
	// Main loop
	while (true) {

		bool middleOfSequence = grabber->grab(); // grab frame
		if (!middleOfSequence)
			break;

		SensorFrame currentSensorFrame = grabber->getSensorFrame();

		if (ifStart) {
			matcher->Matcher::loadInitFeatures(currentSensorFrame);

			///
			/// Add the found feature to the map
			///

			// Getting observed features
			Matcher::featureSet features = matcher->getFeatures();

			// cameraPose as Eigen::Transform
			Mat34 cameraPose = Mat34(robotPose.cast<double>());
			Quaternion cameraOrient = Quaternion(cameraPose.rotation());

			// Convert to mapFeatures format
			std::vector<RGBDFeature> mapFeatures;
			for (int j = 0; j < features.feature3D.size(); j++) {
				// Create an extended descriptor
				ExtendedDescriptor desc(cameraOrient, features.descriptors.row(j));

				// In further processing we expect more descriptors
				std::vector<ExtendedDescriptor> extDescriptors{desc};

				// Convert translation
				Eigen::Translation<double, 3> featurePosition(features.feature3D[j].cast<double>());

				// Add to set addded later to map
				RGBDFeature f(featurePosition, extDescriptors);
				mapFeatures.push_back(f);

			}

			// Finally, adding to map
			// TODO: Uncomment when error is corrected
			// map->addFeatures(mapFeatures, cameraPose);
			ifStart = false;
		} else {
			Eigen::Matrix4f transformation;
			matcher->Matcher::match(currentSensorFrame, transformation);

			robotPose = robotPose * transformation;

			// Get the visible features
			//std::vector<MapFeature> mapFeatures = map->getAllFeatures();

			// Perform RANSAC matching and return measurements for found inliers in map compatible format
			// Remember! The match returns the list of inlier features from current pose!
			// TODO: Choosing descriptor based on orientation
			//std::vector<MapFeature> measurementList;
			// TODO: Uncomment when error is corrected
			//matcher->Matcher::match(mapFeatures, measurementList);

			// Add the measurements of inliers
			// TODO: Uncomment when error is corrected
			//map->addMeasurements(measurementList);

			// Add new features

		}



		// Save trajectory
		saveTrajectoryFreiburgFormat(robotPose, trajectoryFreiburgStream,
				currentSensorFrame.timestamp);

	}

	// Optimize after trajectory
	// TODO: Uncomment when error is corrected
	//map->startOptimizationThread(1);

	// Wait for optimization finish
	// TODO: Uncomment when error is corrected
	//map->finishOptimization();

	// Close trajectory stream
	trajectoryFreiburgStream.close();

	return 0;
}
