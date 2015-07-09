#include "../include/PUTSLAM/PUTSLAM.h"
#include "../include/Utilities/simulator.h"
#include "../include/MotionModel/decayingVelocityModel.h"

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

			//std::cout << "Correct depth of feature" << std::endl;

			bool featureOk = true;

			// Lets remove features too close to existing features
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

			// Lets remove features too close to features to add :)
			// TODO: code is repeated -> extract method
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
//				std::cout << "Adding feature of (u,v) = ("
//						<< features.undistortedFeature2D[j].x << ", "
//						<< features.undistortedFeature2D[j].y << ")"
//						<< std::endl;
//
//				std::cout << "VALUE: " << features.descriptors.empty() << " j="
//						<< j << " " <<features.descriptors.rows << " " << features.descriptors.cols << 	std::endl;

				// Create an extended descriptor
				cv::Mat descMat;
				if (!features.descriptors.empty()) {
					descMat = features.descriptors.row(j).clone();
				}

				ExtendedDescriptor desc(cameraPoseId,
						features.undistortedFeature2D[j].x,
						features.undistortedFeature2D[j].y, descMat); // TODO: change between descriptor based and descriptor free versions -

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

///Attahc visualizer
void PUTSLAM::attachVisualizer(QGLVisualizer* visualizer) {
	((FeaturesMap*) map)->attach(visualizer);
}

void PUTSLAM::startProcessing() {

	bool ifStart = true;
	bool onlyVO = false;

	// Optimize during trajectory acquisition
	if (optimizationThreadVersion == OPTTHREAD_ON)
		map->startOptimizationThread(1, 0);
	else if (optimizationThreadVersion == OPTTHREAD_ON_ROBUSTKERNEL)
		map->startOptimizationThread(1, 0, "Cauchy", 1);

	// Thread looking for too close features
	if (mapManagmentThreadVersion == MAPTHREAD_ON)
		map->startMapManagerThread(1);

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
	int minMeasurementsToAddPoseToFeatureEdge =
			((FeaturesMap*) map)->getMinMeasurementsToAddPoseToFeatureEdge();
	bool addPoseToPoseEdges = ((FeaturesMap*) map)->getAddPoseToPoseEdges();

	double depthImageScale =
			((FileGrabber*) grabber)->parameters.depthImageScale;

	///for inverse SLAM problem
	//Simulator simulator;
	//simulator.loadTrajectory("../../resources/traj_living_room_kt2.txt");
	//std::vector<Mat34> traj = simulator.getTrajectory();
	int frameCounter=0;

	std::unique_ptr<MotionModel> motionModel;
	motionModel.reset(new DecayingVelocityModel(1000, 1, 0.9));
	double motionModelLastTime = -1;

	auto startMainLoop = std::chrono::system_clock::now();
	// Main loop
	while (true) {

		bool middleOfSequence = grabber->grab(); // grab frame
		if (!middleOfSequence)
			break;
		///for inverse SLAM problem
		// if (trajIt>traj.size()-1)
		//   break;

		SensorFrame currentSensorFrame = grabber->getSensorFrame();

		// TODO DOMINIK - I have about 40 different config files for tests and now they are invalid as you introduced addiitional parameter.
		// Let me know about some changes like that or at least send me new config file ...

//        if (matcher->matcherParameters.showRGBframe)
//            cv::imshow( "PUTSLAM RGB frame", currentSensorFrame.rgbImage );
//        if (matcher->matcherParameters.showDepthFrame)
//            cv::imshow( "PUTSLAM Depth frame", currentSensorFrame.depthImage );

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
			if (!onlyVO) {
				cameraPoseId = map->addNewPose(cameraPose,
						currentSensorFrame.timestamp,
						currentSensorFrame.rgbImage,
						currentSensorFrame.depthImage);
			}

			// Correct motionModel
			motionModelLastTime = currentSensorFrame.timestamp;
			Mat34 x = motionModel.get()->correct(cameraPose);
			motionModelPose = Eigen::Matrix4f(x.matrix().cast<float>());

			ifStart = false;
			addFeatureToMap = true;
		}
		// The next pose in the sequence
		else {
			Eigen::Matrix4f transformation;
			std::vector<cv::DMatch> inlierMatches;

			// Running VO - matching or tracking depending on parameters
			double inlierRatio = matcher->Matcher::runVO(currentSensorFrame,
					transformation, inlierMatches);
			VORansacInlierRatioLog.push_back(inlierRatio);

			//for inverse slam problem
			// Mat34 transReal = traj[trajIt-1].inverse()*traj[trajIt];
			//             transformation = transReal.cast<float>().matrix();
			//            std::cout << "iteration: " << trajIt << "\n";

			// Saving inliers for Dominic
			//			Matcher::featureSet features = matcher->getFeatures();
			//			saveFeaturesToFile(features, inlierMatches, currentSensorFrame.timestamp);

			if (!onlyVO) {
				robotPose = robotPose * transformation;

				// cameraPose as Eigen::Transform
				Mat34 cameraPoseIncrement = Mat34(
						transformation.cast<double>());

				// Motion model
				if (int(currentSensorFrame.timestamp) % 4 != 0) {
					Mat34 x = motionModel.get()->predict(
							motionModelLastTime - currentSensorFrame.timestamp);
					motionModelPose = Eigen::Matrix4f(x.matrix().cast<float>());
					motionModelLastTime = currentSensorFrame.timestamp;
				}

				// Add new position to the map
				cameraPoseId = map->addNewPose(cameraPoseIncrement,
						currentSensorFrame.timestamp,
						currentSensorFrame.rgbImage,
						currentSensorFrame.depthImage);

				// Get the visible features
				Mat34 cameraPose = map->getSensorPose();
				mapFeatures = map->getVisibleFeatures(cameraPose);
				//mapFeatures = map->getVisibleFeatures(cameraPose, 500, 10.05);

				//map->removeDistantFeatures(mapFeatures, 500, 10.01);

				// Find the ids of frames for which feature observations have the most similar angle
				std::vector<int> frameIds;
				std::vector<float_type> angles;
				map->findNearestFrame(mapFeatures, frameIds, angles,
						matcher->matcherParameters.maxAngleBetweenFrames);

				//Remove features that we do not have a good observation angle
				std::vector<MapFeature>::iterator mapFeaturesIter =
						mapFeatures.begin();
				std::vector<int>::iterator frameIdsIter = frameIds.begin();
				std::vector<float_type>::iterator anglesIter = angles.begin();

				for (; mapFeaturesIter != mapFeatures.end();) {
					if (*frameIdsIter == -1) {
						mapFeaturesIter = mapFeatures.erase(mapFeaturesIter);
						frameIdsIter = frameIds.erase(frameIdsIter);
						anglesIter = angles.erase(anglesIter);
					} else {
						++mapFeaturesIter;
						++frameIdsIter;
						++anglesIter;
					}
				}

				// Move mapFeatures to local coordinate system
				moveMapFeaturesToLocalCordinateSystem(cameraPose, mapFeatures);

				std::cout
						<< "Returned visible map feature size before if not cover test: "
						<< mapFeatures.size() << std::endl;

				// Now lets check if those features are not behind sth
				RGBD::removeMapFeaturesWithoutDepth(mapFeatures,
						currentSensorFrame.depthImage, 0.15f, frameIds, angles,
						depthImageScale);

				std::cout << "Returned visible map feature size: "
						<< mapFeatures.size() << std::endl;

				// Show map features
				if (matcher->matcherParameters.verbose > 0)
					showMapFeatures(currentSensorFrame.rgbImage, mapFeatures);

				// Perform RANSAC matching and return measurements for found inliers in map compatible format
				// Remember! The match returns the list of inlier features from current pose!
				std::vector<MapFeature> measurementList;
				Eigen::Matrix4f mapEstimatedTransformation;

				double mapMatchingInlierRatio = 0.0f;
				if (matcher->matcherParameters.MapMatchingVersion
						== Matcher::MatcherParameters::MAPMATCH_DESCRIPTORS) {

					mapMatchingInlierRatio = matcher->Matcher::match(
							mapFeatures, cameraPoseId, measurementList,
							mapEstimatedTransformation);

				} else if (matcher->matcherParameters.MapMatchingVersion
						== Matcher::MatcherParameters::MAPMATCH_XYZ_DESCRIPTORS) {
					mapMatchingInlierRatio = matcher->Matcher::matchXYZ(
							mapFeatures, cameraPoseId, measurementList,
							mapEstimatedTransformation, frameIds);
				} else if (matcher->matcherParameters.MapMatchingVersion
						== Matcher::MatcherParameters::MAPMATCH_PATCHES
						|| matcher->matcherParameters.MapMatchingVersion
								== Matcher::MatcherParameters::MAPMATCH_XYZ_DESCRIPTORS_PATCHES) {

					// Prepare set of images
					std::vector<cv::Mat> mapRgbImages(frameIds.size()),
							mapDepthImages(frameIds.size());
					std::vector<putslam::Mat34> cameraPoses(frameIds.size());
					for (int i = 0; i < frameIds.size(); i++) {
						map->getImages(frameIds[i], mapRgbImages[i],
								mapDepthImages[i]);
						cameraPoses[i] = map->getSensorPose(frameIds[i]);
					}

					if (matcher->matcherParameters.MapMatchingVersion
							== Matcher::MatcherParameters::MAPMATCH_PATCHES) {
						std::vector<std::pair<double, double>> tmp;
						mapMatchingInlierRatio =
								matcher->matchToMapUsingPatches(mapFeatures,
										cameraPoseId, cameraPose, frameIds,
										cameraPoses, mapRgbImages,
										mapDepthImages, measurementList,
										mapEstimatedTransformation, depthImageScale, tmp);

					} else if (matcher->matcherParameters.MapMatchingVersion
							== Matcher::MatcherParameters::MAPMATCH_XYZ_DESCRIPTORS_PATCHES) {
						// XYZ+desc
						std::vector<MapFeature> probablyInliers;
						mapMatchingInlierRatio = matcher->Matcher::matchXYZ(
								mapFeatures, cameraPoseId, probablyInliers,
								mapEstimatedTransformation);

						// PATCHES
						std::vector<std::pair<double, double>> errorLog;
						mapMatchingInlierRatio =
								matcher->matchToMapUsingPatches(probablyInliers,
										cameraPoseId, cameraPose, frameIds,
										cameraPoses, mapRgbImages,
										mapDepthImages, measurementList,
										mapEstimatedTransformation,
										currentSensorFrame.depthImageScale,
										errorLog, false);

						std::cout << "Measurement list size : "
								<< measurementList.size() << std::endl;

						// Add error to log
						patchesErrorLog.insert(patchesErrorLog.end(),
								errorLog.begin(), errorLog.end());

					}
				} else {
					std::cout
							<< "Unrecognized map matching version -- double check matcherOpenCVParameters.xml"
							<< std::endl;
				}
				MapMatchingRansacInlierRatioLog.push_back(
						mapMatchingInlierRatio);

				/// for inverse slam problem (ver. A)
				//mapEstimatedTransformation.setIdentity();
				// TESTING VO with map corrections
				VoMapPose = VoMapPose * transformation
						* mapEstimatedTransformation;

				std::cout << "Measurement list size : "
						<< measurementList.size() << std::endl;

				// Compare VO and VOMap estimate -> decide whether to add measurements to map
				float distanceDiff = mapEstimatedTransformation.block<3, 1>(0,
						3).norm();
				std::cout << "Difference between VO and Map : " << distanceDiff
						<< " meters" << std::endl;

				// Add pose-pose constrain - depends on config file
				if (addPoseToPoseEdges) {
					map->addMeasurement(cameraPoseId - 1, cameraPoseId,
							cameraPoseIncrement);
				}

				// Add pose-feature constrain
				measurementToMapSizeLog.push_back(measurementList.size());
				if (measurementList.size()
						> minMeasurementsToAddPoseToFeatureEdge) {
//					matcher->computeNormals(currentSensorFrame.depthImage, measurementList);
//					matcher->computeRGBGradients(currentSensorFrame.rgbImage, currentSensorFrame.depthImage, measurementList);
					map->addMeasurements(measurementList);
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
		}

		// Should we add some features to the map?
		if (addFeatureToMap && !onlyVO) {
			std::cout << "Adding features to map " << std::endl;

			// Getting observed features
			Matcher::featureSet features = matcher->getFeatures();

			std::cout << "Feature size: "
					<< features.undistortedFeature2D.size() << std::endl;

			// Convert to mapFeatures format
			std::vector<RGBDFeature> mapFeaturesToAdd;
			int addedCounter = 0;

			// Lets process possible features to add
			// TODO: change between descriptor based and descriptor free versions
			addedCounter = chooseFeaturesToAddToMap(features, addedCounter,
					maxOnceFeatureAdd, mapFeatures,
					minEuclideanDistanceOfFeatures, minImageDistanceOfFeatures,
					cameraPoseId, mapFeaturesToAdd);

			std::cout << "map->addFeatures -> adding " << addedCounter
					<< " features" << std::endl;

//            matcher->computeNormals(currentSensorFrame.depthImage, mapFeaturesToAdd);
//            matcher->computeRGBGradients(currentSensorFrame.rgbImage, currentSensorFrame.depthImage, mapFeaturesToAdd);

			// Finally, adding to map
			map->addFeatures(mapFeaturesToAdd, cameraPoseId);

			std::cout << "map->addFeatures -> added " << addedCounter
					<< " features" << std::endl;

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

		saveTrajectoryFreiburgFormat(motionModelPose,
				trajectoryMotionModelStream, currentSensorFrame.timestamp);

        frameCounter++;
	}
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startMainLoop);
    saveFPS(double(frameCounter)/(elapsed.count()/1000.0));

	// Save statistics
	std::cout << "Saving logs to file" << std::endl;
	saveLogs();

	map->save2file("createdMapFile.map", "preOptimizedGraphFile.g2o");

	// We optimize only at the end if that version is chosen
	if (optimizationThreadVersion == OPTTHREAD_ATEND)
		map->startOptimizationThread(1, 1);

	// Wait for optimization thread to finish
	if (optimizationThreadVersion != OPTTHREAD_OFF)
		map->finishOptimization("graph_trajectory.res",
				"optimizedGraphFile.g2o");

	// Wait for management thread to finish
	if (mapManagmentThreadVersion == MAPTHREAD_ON)
		map->finishManagementThr();  // Wait for optimization thread to finish

	///for inverse SLAM problem
//    for (int i=0; i<traj.size();i++){
//        VertexSE3 vert(i, traj[i], i);
//        ((FeaturesMap*) map)->updatePose(vert, true);
//    }
	map->exportOutput("graph_trajectory.res", "optimizedGraphFile.g2o");

	// Close trajectory stream
	trajectoryFreiburgStream.close();
	trajectoryVOMapStream.close();
	trajectoryMotionModelStream.close();

	// Run statistics
	std::cout << "Evaluating trajectory" << std::endl;
	evaluateResults(((FileGrabber*) grabber)->parameters.basePath,
			((FileGrabber*) grabber)->parameters.datasetName);

	// Save map
//	std::cout << "Saving to octomap" << std::endl;
//	int size = map->getPoseCounter();
//	for (int i = 0; i < size; i = i + size / 15) {
//
//		std::cout << "Octomap uses point clouds with id = " << i << std::endl;
//
//		cv::Mat rgbImage, depthImage;
//		map->getImages(i, rgbImage, depthImage);
//		Mat34 pose = map->getSensorPose(i);
//		Eigen::Matrix4f tmpPose = Eigen::Matrix4f(pose.matrix().cast<float>());
//
//		// Save for octomap
//		std::vector<Eigen::Vector3f> pointCloud = RGBD::imageToPointCloud(
//				rgbImage, depthImage,
//				matcher->matcherParameters.cameraMatrixMat, tmpPose, depthImageScale);
//		RGBD::saveToFile(pointCloud, "octomap.log", i == 0);
//
//	}

	std::cout << "Job finished! Good bye :)" << std::endl;
}

/// PRIVATE

void PUTSLAM::loadConfigs() {
	tinyxml2::XMLDocument config;
	config.LoadFile("../../resources/configGlobal.xml");
	if (config.ErrorID())
		std::cout << "unable to load config file.\n";

	// Thread settings
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute("verbose",
			&verbose);
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute(
			"optimizationThreadVersion", &optimizationThreadVersion);
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute(
			"mapManagmentThreadVersion", &mapManagmentThreadVersion);

	if (verbose > 0) {
		std::cout << "PUTSLAM: optimizationThreadVersion = "
				<< optimizationThreadVersion << std::endl;
		std::cout << "PUTSLAM: mapManagmentThreadVersion = "
				<< mapManagmentThreadVersion << std::endl;
	}

	// Create map
	if (verbose > 0) {
		std::cout<<"Getting grabber config" << std::endl;
	}
	std::string configFileGrabber(
			config.FirstChildElement("Grabber")->FirstChildElement(
					"calibrationFile")->GetText());

	if (verbose > 0) {
		std::cout<<"Getting map config" << std::endl;
	}
	std::string configFileMap(
			config.FirstChildElement("Map")->FirstChildElement("parametersFile")->GetText());

	if (verbose > 0) {
		std::cout << "Creating features map" << std::endl;
	}
	map = createFeaturesMap(configFileMap, configFileGrabber);

	if (verbose > 0) {
		std::cout << "Features map is initialized" << std::endl;
	}

	std::string grabberType(
			config.FirstChildElement("Grabber")->FirstChildElement("name")->GetText());

	std::string grabberConfigFile(
			config.FirstChildElement("Grabber")->FirstChildElement(
					"calibrationFile")->GetText());

	if (verbose > 0) {
		std::cout << "Creating grabber with type = " << grabberType << std::endl;
	}

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
	if (verbose > 0) {
		cout << "Current grabber: " << grabber->getName() << std::endl;
	}
	string matcherParameters =
			config.FirstChildElement("Matcher")->FirstChildElement(
					"parametersFile")->GetText();

	if (verbose > 0) {
		std::cout<<"Creating matcher" << std::endl;
	}
	matcher = createMatcherOpenCV(matcherParameters, grabberConfigFile);
	if (verbose > 0) {
		cout << "Current matcher: " << matcher->getName() << std::endl;
	}

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

void PUTSLAM::saveFPS(float_type fps) {
	ofstream fileFPS;
	fileFPS.open("fps.res");
	fileFPS << fps;
	fileFPS.close();
}

void PUTSLAM::saveLogs() {
	ofstream statisticsLogStream("statistics.py");

	statisticsLogStream << "import matplotlib.pyplot as plt" << endl;
	statisticsLogStream << "import numpy as np" << endl;

	statisticsLogStream << "plt.ioff()" << std::endl;

	// VORansacInlierRatioLog
	statisticsLogStream << "VORansacInlierRatioLog = np.array([";
	for (int a = 0; a < VORansacInlierRatioLog.size(); a++) {
		statisticsLogStream << VORansacInlierRatioLog[a] << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream
			<< "plt.plot(VORansacInlierRatioLog, label='-1 means no matches before RANSAC')"
			<< endl;
	statisticsLogStream
			<< "fig.suptitle('RANSAC inlier ratio in VO', fontsize=20)" << endl;
	statisticsLogStream << "plt.xlabel('Frame counter', fontsize=18)" << endl;
	statisticsLogStream << "plt.ylabel('Inlier ratio', fontsize=16)" << endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('VORansacInlierRatio.png')" << endl;

	// MapMatchingRansacInlierRatioLog
	statisticsLogStream << "MapMatchingRansacInlierRatioLog = np.array([";
	for (int a = 0; a < MapMatchingRansacInlierRatioLog.size(); a++) {
		statisticsLogStream << MapMatchingRansacInlierRatioLog[a] << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream
			<< "plt.plot(MapMatchingRansacInlierRatioLog, label='-1 means no matches before RANSAC')"
			<< endl;
	statisticsLogStream
			<< "fig.suptitle('RANSAC inlier ratio in Map Matching', fontsize=20)"
			<< endl;
	statisticsLogStream << "plt.xlabel('Frame counter', fontsize=18)" << endl;
	statisticsLogStream << "plt.ylabel('Inlier ratio', fontsize=16)" << endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('MapMatchingRansacInlierRatioLog.png')"
			<< endl;

	// Measurement to map size
	statisticsLogStream << "mapMeasurementSize = np.array([";
	for (int a = 0; a < measurementToMapSizeLog.size(); a++) {
		statisticsLogStream << measurementToMapSizeLog[a] << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream
			<< "plt.plot(mapMeasurementSize, label='-1 means no matches before RANSAC')"
			<< endl;
	statisticsLogStream
			<< "fig.suptitle('Measurement number to features in map', fontsize=20)"
			<< endl;
	statisticsLogStream << "plt.xlabel('Frame counter', fontsize=18)" << endl;
	statisticsLogStream << "plt.ylabel('Measurement number', fontsize=16)"
			<< endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('mapMatchinggSize.png')" << endl;

	// diff 2D in patches
	statisticsLogStream << "error2DPatchesSize = np.array([";
	for (int a = 0; a < patchesErrorLog.size(); a++) {
		statisticsLogStream << patchesErrorLog[a].first << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream << "plt.plot(error2DPatchesSize)" << endl;
	statisticsLogStream << "fig.suptitle('2D patches diff', fontsize=20)"
			<< endl;
	statisticsLogStream << "plt.xlabel('Patches used counter', fontsize=18)"
			<< endl;
	statisticsLogStream << "plt.ylabel('diff [px]', fontsize=16)" << endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('diff2DPatchesSize.png')" << endl;

	// diff 3D in patches
	statisticsLogStream << "error3DPatchesSize = np.array([";
	for (int a = 0; a < patchesErrorLog.size(); a++) {
		statisticsLogStream << patchesErrorLog[a].second << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream << "plt.plot(error3DPatchesSize)" << endl;
	statisticsLogStream << "fig.suptitle('3D patches diff', fontsize=20)"
			<< endl;
	statisticsLogStream << "plt.xlabel('Patches used counter', fontsize=18)"
			<< endl;
	statisticsLogStream << "plt.ylabel('diff [m]', fontsize=16)" << endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('diff3DPatchesSize.png')" << endl;

	statisticsLogStream.close();
}

void PUTSLAM::evaluateResults(std::string basePath, std::string datasetName) {

	std::string fullPath = basePath + "/" + datasetName + "/";

	std::string evalATEVO =
			"python2 ../../scripts/evaluate_ate.py " + fullPath
					+ "groundtruth.txt VO_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot VOAte.png > VOAte.res";
	std::string evalATEMap =
			"python2 ../../scripts/evaluate_ate.py " + fullPath
					+ "groundtruth.txt graph_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot g2oAte.png > g2oAte.res";
	std::string evalRPEVO =
			"python2 ../../scripts/evaluate_rpe.py " + fullPath
					+ "groundtruth.txt VO_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot VORpe.png > VORpe.res";
	std::string evalRPEMap =
			"python2 ../../scripts/evaluate_rpe.py " + fullPath
					+ "groundtruth.txt graph_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot g2oRpe.png > g2oRpe.res";
	try {
		int tmp = std::system(evalATEVO.c_str());
		tmp = std::system(evalATEMap.c_str());
		tmp = std::system(evalRPEVO.c_str());
		tmp = std::system(evalRPEMap.c_str());
	} catch (std::system_error& error) {
		std::cout << "Error: " << error.code() << " - " << error.what() << '\n';
	}

	ofstream datasetNameStream("DatasetName");
	datasetNameStream << fullPath;
	datasetNameStream.close();

}

void PUTSLAM::showMapFeatures(cv::Mat rgbImage,
		std::vector<MapFeature> mapFeatures) {
	std::vector<cv::KeyPoint> mapFeatures2D(mapFeatures.size());
	std::transform(mapFeatures.begin(), mapFeatures.end(),
			mapFeatures2D.begin(),
			[](const MapFeature& m) {return cv::KeyPoint(m.u, m.v, 3);});

	cv::Mat img2draw;
	cv::drawKeypoints(rgbImage, mapFeatures2D, img2draw, cv::Scalar(0, 0, 255));

	cv::imshow("Map features", img2draw);
	cv::waitKey(10000);
}
