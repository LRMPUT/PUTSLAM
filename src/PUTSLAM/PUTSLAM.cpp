#include "../include/PUTSLAM/PUTSLAM.h"

void PUTSLAM::moveMapFeaturesToLocalCordinateSystem(const Mat34& cameraPose,
		std::vector<MapFeature>& mapFeatures) {
	// Move mapFeatures to local coordinate system
	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it) {
		Mat34 featurePos((*it).position);
		featurePos = (cameraPose.inverse()).matrix() * featurePos.matrix();
		it->position = Vec3(featurePos(0, 3), featurePos(1, 3),
				featurePos(2, 3));
		//				 Compute image position
		Eigen::Vector3d projectedMapPoint =
				map->getDepthSensorModel().inverseModel(featurePos(0, 3),
						featurePos(1, 3), featurePos(2, 3));
		it->u = projectedMapPoint[0];
		it->v = projectedMapPoint[1];
	}
}

int PUTSLAM::chooseFeaturesToAddToMap(const Matcher::featureSet& features,
		int addedCounter, int maxOnceFeatureAdd,
		const std::vector<MapFeature>& mapFeatures,
		float minEuclideanDistanceOfFeatures, float minImageDistanceOfFeatures,
		int cameraPoseId, std::vector<RGBDFeature>& mapFeaturesToAdd) {
	// Lets process possible features to add
	for (int j = 0;
			j < features.feature3D.size() && addedCounter < maxOnceFeatureAdd;
			j++) {

		// We only add features of proper depth
		if (features.feature3D[j][2] > 0.8 && features.feature3D[j][2] < 6.0) {

			bool featureOk = true;

			// Lets remove features to close to existing features
			for (int i = 0; i < mapFeatures.size(); i++) {

				// Euclidean norm
				Eigen::Vector3f tmp(mapFeatures[i].position.x(),
						mapFeatures[i].position.y(),
						mapFeatures[i].position.z());
				float norm = (tmp - features.feature3D[j]).norm();

				if (norm < minEuclideanDistanceOfFeatures) {
					featureOk = false;
					break;
				}

				// Image error
				Eigen::Vector3d projectedMapPoint =
						map->getDepthSensorModel().inverseModel(
								mapFeatures[i].position.x(),
								mapFeatures[i].position.y(),
								mapFeatures[i].position.z());

				cv::Point2f point(projectedMapPoint[0], projectedMapPoint[1]);
				float imageNorm = cv::norm(
						point - features.undistortedFeature2D[j]);

				// 5 pixels is a minimum distance
				if (imageNorm < minImageDistanceOfFeatures) {
					featureOk = false;
					break;
				}

			}

			// Lets remove features to close to features to add :)
			if (featureOk) {
				for (int i = 0; i < mapFeaturesToAdd.size(); i++) {

					Eigen::Vector3f tmp(mapFeaturesToAdd[i].position.x(),
							mapFeaturesToAdd[i].position.y(),
							mapFeaturesToAdd[i].position.z());
					float norm = (tmp - features.feature3D[j]).norm();

					if (norm < minEuclideanDistanceOfFeatures) {
						featureOk = false;
						break;
					}

					cv::Point2f point(mapFeaturesToAdd[i].u,
							mapFeaturesToAdd[i].v);
					float imageNorm = cv::norm(
							point - features.undistortedFeature2D[j]);

					// 5 pixels is a minimum distance
					if (imageNorm < minImageDistanceOfFeatures) {
						featureOk = false;
						break;
					}
				}
			}

			if (featureOk) {
				// Create an extended descriptor
				ExtendedDescriptor desc(cameraPoseId,
						features.descriptors.row(j));

				// In further processing we expect more descriptors
				std::vector<ExtendedDescriptor> extDescriptors { desc };

				// Convert translation
				Eigen::Translation<double, 3> featurePosition(
						features.feature3D[j].cast<double>());

				// Add to set added later to map
				RGBDFeature f(featurePosition,
						features.undistortedFeature2D[j].x,
						features.undistortedFeature2D[j].y, extDescriptors);
				mapFeaturesToAdd.push_back(f);

				addedCounter++;
			}
		}
	}
	return addedCounter;
}

/// PUBLIC

void PUTSLAM::startProcessing() {

	bool ifStart = true;

	// Optimize during trajectory acquisition
	//map->startOptimizationThread(1,1, "Cauchy",1); with robust kernel
	map->startOptimizationThread(1, 1);

	/// TODO: MAKE IT NICER
	int addFeaturesWhenMapSizeLessThan =
			((FeaturesMap*) map)->getAddFeaturesWhenMapSizeLessThan();
	int addFeaturesWhenMeasurementSizeLessThan =
			((FeaturesMap*) map)->getAddFeaturesWhenMeasurementSizeLessThan();
	int maxOnceFeatureAdd = ((FeaturesMap*) map)->getMaxOnceFeatureAdd();
	float minEuclideanDistanceOfFeatures =
			((FeaturesMap*) map)->getMinEuclideanDistanceOfFeatures();
	float minImageDistanceOfFeatures =
			((FeaturesMap*) map)->getMinImageDistanceOfFeatures();
	int addNoFeaturesWhenMapSizeGreaterThan =
			((FeaturesMap*) map)->getAddNoFeaturesWhenMapSizeGreaterThan();

	// Main loop
	while (true) {

		bool middleOfSequence = grabber->grab(); // grab frame
		if (!middleOfSequence)
			break;

		SensorFrame currentSensorFrame = grabber->getSensorFrame();

		int cameraPoseId = 0;
		bool addFeatureToMap = false;

		// Variable to store map features
		std::vector<MapFeature> mapFeatures;

		// The beginning of the sequence
		if (ifStart) {
			matcher->Matcher::loadInitFeatures(currentSensorFrame);

			// cameraPose as Eigen::Transform
			Mat34 cameraPose = Mat34(robotPose.cast<double>());

			// Add new position to the map
			cameraPoseId = map->addNewPose(cameraPose,
					currentSensorFrame.timestamp);

			ifStart = false;
			addFeatureToMap = true;
		}
		// The next pose in the sequence
		else {
			Eigen::Matrix4f transformation;
			std::vector<cv::DMatch> inlierMatches;

			matcher->Matcher::match(currentSensorFrame, transformation,
					inlierMatches);

			// Saving inliers for Dominic
			//			Matcher::featureSet features = matcher->getFeatures();
			//			saveFeaturesToFile(features, inlierMatches, currentSensorFrame.timestamp);

			robotPose = robotPose * transformation;

			// cameraPose as Eigen::Transform
			Mat34 cameraPoseIncrement = Mat34(transformation.cast<double>());

			// Add new position to the map
			cameraPoseId = map->addNewPose(cameraPoseIncrement,
					currentSensorFrame.timestamp);

			// Get the visible features
			Mat34 cameraPose = map->getSensorPose();
			mapFeatures = map->getVisibleFeatures(cameraPose);

			// Move mapFeatures to local coordinate system
			moveMapFeaturesToLocalCordinateSystem(cameraPose, mapFeatures);

			std::cout
					<< "Returned visible map feature size before if not cover test: "
					<< mapFeatures.size() << std::endl;

			// Now lets check if those features are not behind sth
			RGBD::removeMapFeaturesWithoutDepth(mapFeatures,
					currentSensorFrame.depthImage, 0.1f);

			std::cout << "Returned visible map feature size: "
					<< mapFeatures.size() << std::endl;

			// Perform RANSAC matching and return measurements for found inliers in map compatible format
			// Remember! The match returns the list of inlier features from current pose!
			std::vector<MapFeature> measurementList;
			Eigen::Matrix4f mapEstimatedTransformation;
			matcher->Matcher::matchXYZ(mapFeatures, cameraPoseId,
					measurementList, mapEstimatedTransformation);
			//			matcher->Matcher::match(mapFeatures, cameraPoseId, measurementList, mapEstimatedTransformation);

			// TESTING VO with map corrections
			VoMapPose = VoMapPose * transformation * mapEstimatedTransformation;

			std::cout << "Measurement list size : " << measurementList.size()
					<< std::endl;

			// Compare VO and VOMap estimate -> decide whether to add measurements to map
			float distanceDiff =
					mapEstimatedTransformation.block<3, 1>(0, 3).norm();
			std::cout << "Difference between VO i Map : " << distanceDiff
					<< " meters" << std::endl;

			if (distanceDiff < 0.05) {
				// Add the measurements of inliers
				map->addMeasurements(measurementList);
			} else {
				//				addFeatureToMap = true;
				map->addMeasurement(cameraPoseId - 1, cameraPoseId,
						cameraPoseIncrement);
			}

			// Insufficient number of features -> time to add some features
			if (mapFeatures.size() < addFeaturesWhenMapSizeLessThan
					|| (measurementList.size()
							< addFeaturesWhenMeasurementSizeLessThan
							&& mapFeatures.size()
									< addNoFeaturesWhenMapSizeGreaterThan)) {
				addFeatureToMap = true;
			}
		}

		// Should we add some features to the map?
		if (addFeatureToMap) {
			std::cout << "Adding features to map " << std::endl;

			// Getting observed features
			Matcher::featureSet features = matcher->getFeatures();

			// Convert to mapFeatures format
			std::vector<RGBDFeature> mapFeaturesToAdd;
			int addedCounter = 0;

			// Lets process possible features to add
			addedCounter = chooseFeaturesToAddToMap(features, addedCounter,
					maxOnceFeatureAdd, mapFeatures,
					minEuclideanDistanceOfFeatures, minImageDistanceOfFeatures,
					cameraPoseId, mapFeaturesToAdd);

			std::cout << "map->addFeatures -> adding " << addedCounter
					<< " features" << std::endl;

			// Finally, adding to map
			map->addFeatures(mapFeaturesToAdd, cameraPoseId);

			addFeatureToMap = false;
		}

		// Saving features for Dominik
		//		Matcher::featureSet features = matcher->getFeatures();
		//		saveFeaturesToFile(features, currentSensorFrame.timestamp);

		// Save trajectory
		saveTrajectoryFreiburgFormat(robotPose, trajectoryFreiburgStream,
				currentSensorFrame.timestamp);

		saveTrajectoryFreiburgFormat(VoMapPose, trajectoryVOMapStream,
				currentSensorFrame.timestamp);
	}

	map->save2file("createdMapFile.map", "preOptimizedGraphFile.g2o");

	// Wait for optimization finish
	map->finishOptimization("graph_trajectory.res", "optimizedGraphFile.g2o");

	// Close trajectory stream
	trajectoryFreiburgStream.close();
	trajectoryVOMapStream.close();
}

/// PRIVATE

void PUTSLAM::loadConfigs() {
	tinyxml2::XMLDocument config;
	config.LoadFile("../../resources/configGlobal.xml");
	if (config.ErrorID())
		std::cout << "unable to load config file.\n";

	// Create map
	std::string configFileGrabber(
			config.FirstChildElement("Grabber")->FirstChildElement(
					"calibrationFile")->GetText());
	std::string configFileMap(
			config.FirstChildElement("Map")->FirstChildElement("parametersFile")->GetText());
	map = createFeaturesMap(configFileMap, configFileGrabber);

	std::string grabberType(
			config.FirstChildElement("Grabber")->FirstChildElement("name")->GetText());

	std::string grabberConfigFile(
			config.FirstChildElement("Grabber")->FirstChildElement(
					"calibrationFile")->GetText());
	if (grabberType == "Kinect") {
		grabber = createGrabberKinect(grabberConfigFile, Grabber::MODE_BUFFER);
	} else if (grabberType == "Xtion") {
		grabber = createGrabberXtion(grabberConfigFile, Grabber::MODE_BUFFER);
	}
	/// Still do not take into account the config file
	else if (grabberType == "File") {
		grabber = createGrabberFile(grabberConfigFile);
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

	matcher = createMatcherOpenCV(matcherParameters, grabberConfigFile);
	cout << "Current matcher: " << matcher->getName() << std::endl;
}

void PUTSLAM::saveTrajectoryFreiburgFormat(Eigen::Matrix4f transformation,
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

void PUTSLAM::saveFeaturesToFile(Matcher::featureSet features,
		double timestamp) {

	int whatever = system("mkdir featuresDir");

	std::ostringstream fileName;
	fileName << std::setfill('0') << std::setprecision(17) << timestamp;
	std::ofstream file("featuresDir/" + fileName.str() + ".features");

	for (int i = 0; i < features.feature3D.size(); i++) {
		file << features.undistortedFeature2D[i].x << " "
				<< features.undistortedFeature2D[i].y << " "
				<< features.feature3D[i](0) << " " << features.feature3D[i](1)
				<< " " << features.feature3D[i](2) << std::endl;
	}
	file.close();
}

void PUTSLAM::saveFeaturesToFile(Matcher::featureSet features,
		std::vector<cv::DMatch> inlierMatches, double timestamp) {

	int whatever = system("mkdir featuresDir");

	std::ostringstream fileName;
	fileName << std::setfill('0') << std::setprecision(17) << timestamp;
	std::ofstream file("featuresDir/" + fileName.str() + ".features",
			std::ofstream::out | std::ofstream::app);

	for (int i = 0; i < inlierMatches.size(); i++) {
		int id = inlierMatches[i].trainIdx;
		file << features.undistortedFeature2D[id].x << " "
				<< features.undistortedFeature2D[id].y << " "
				<< features.feature3D[id](0) << " " << features.feature3D[id](1)
				<< " " << features.feature3D[id](2) << std::endl;
	}
	file.close();
}
