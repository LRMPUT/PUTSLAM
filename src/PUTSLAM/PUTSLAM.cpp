#include "../include/PUTSLAM/PUTSLAM.h"
#include "../include/Utilities/simulator.h"
#include "../include/MotionModel/decayingVelocityModel.h"

#include <assert.h>

PUTSLAM::PUTSLAM() {

	loadConfigs();

	// Reading robot starting pose
	VOPoseEstimate = grabber->getStartingSensorPose();
	VoMapPose = VOPoseEstimate;

	// File to save trajectory
	trajectoryFreiburgStream.open("VO_trajectory.res");
	trajectoryVOMapStream.open("VOMap_trajectory.res");
	trajectoryMotionModelStream.open("MotionModel_trajectory.res");
	drawImages = false;
	visualize = false;
	map->setDrawOptions(false);

}

void PUTSLAM::moveMapFeaturesToLocalCordinateSystem(const Mat34& cameraPose,
		std::vector<MapFeature>& mapFeatures) {

	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it) {

		// Feature position in GCS
		Mat34 featurePos((*it).position);

		// Moving feature from GCS to LCS
		featurePos = (cameraPose.inverse()).matrix() * featurePos.matrix();
		it->position = Vec3(featurePos(0, 3), featurePos(1, 3),
				featurePos(2, 3));

		// Feature projection onto image
		Eigen::Vector3d projectedMapPoint =
				map->getDepthSensorModel().inverseModel(featurePos(0, 3),
						featurePos(1, 3), featurePos(2, 3));

		// Saving image position
		it->u = projectedMapPoint[0];
		it->v = projectedMapPoint[1];
	}
}

bool PUTSLAM::removeCloseFeatures(std::vector<RGBDFeature> &existingFeatures,
		Eigen::Vector3f feature3D, cv::Point2f feature2D, double minEuclideanDistanceOfFeatures, double minImageDistanceOfFeatures) {

	for (auto &existingFeature : existingFeatures) {

		Eigen::Vector3f tmp(existingFeature.position.x(),
				existingFeature.position.y(),
				existingFeature.position.z());
		float norm = (tmp - feature3D).norm();
		if (norm < minEuclideanDistanceOfFeatures) {
			return false;
		}

		cv::Point2f point(existingFeature.u, existingFeature.v);
		float imageNorm = cv::norm(point - feature2D);
		if (imageNorm < minImageDistanceOfFeatures) {
			return false;
		}
	}
	return true;
}


bool PUTSLAM::removeCloseFeatures(const std::vector<MapFeature> &existingFeatures,
		Eigen::Vector3f feature3D, cv::Point2f feature2D, double minEuclideanDistanceOfFeatures, double minImageDistanceOfFeatures) {

	for (auto &existingFeature : existingFeatures) {

		Eigen::Vector3f tmp(existingFeature.position.x(),
				existingFeature.position.y(),
				existingFeature.position.z());
		float norm = (tmp - feature3D).norm();
		if (norm < minEuclideanDistanceOfFeatures) {
			return false;
		}

		cv::Point2f point(existingFeature.u, existingFeature.v);
		float imageNorm = cv::norm(point - feature2D);
		if (imageNorm < minImageDistanceOfFeatures) {
			return false;
		}
	}
	return true;
}

int PUTSLAM::chooseFeaturesToAddToMap(const Matcher::featureSet& features,
		int addedCounter, int maxOnceFeatureAdd,
		const std::vector<MapFeature>& mapFeatures,
		float minEuclideanDistanceOfFeatures, float minImageDistanceOfFeatures,
		int cameraPoseId, std::vector<RGBDFeature>& mapFeaturesToAdd) {

	assert(
			("chooseFeaturesToAddToMap", features.feature3D.size()
					== features.undistortedFeature2D.size()));
	assert(
			("chooseFeaturesToAddToMap 2", features.feature3D.size()
					== features.descriptors.rows));

	// Lets process possible features to add
	for (int j = 0;
			j < features.feature3D.size() && addedCounter < maxOnceFeatureAdd;
			j++) {

		// We only add features of proper depth
		if (features.feature3D[j][2] > 0.8 && features.feature3D[j][2] < 6.0) {

			// Assume that the feature is ok
			bool featureOk = true;

			// Lets remove features too close to existing features
			featureOk = removeCloseFeatures(mapFeatures,
									features.feature3D[j], features.undistortedFeature2D[j],
									minEuclideanDistanceOfFeatures,
									minImageDistanceOfFeatures);

			// Lets remove features too close to features to add :)
			if (featureOk) {

				featureOk = removeCloseFeatures(mapFeaturesToAdd,
						features.feature3D[j], features.undistortedFeature2D[j],
						minEuclideanDistanceOfFeatures,
						minImageDistanceOfFeatures);
			}

			if (featureOk) {
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

///Attach visualizer
void PUTSLAM::attachVisualizer(QGLVisualizer* visualizer) {
	((FeaturesMap*) map)->attach(visualizer);
}

void PUTSLAM::createAndSaveOctomap(double depthImageScale) {
	int size = map->getPoseCounter();
	// We process every octomapCloudStepSize cloud
	for (int i = 0; i < size; i = i + octomapCloudStepSize) {

		std::cout << "Octomap uses point cloud with id = " << i << std::endl;

		// Getting pose and images
		cv::Mat rgbImage, depthImage;
		map->getImages(i, rgbImage, depthImage);
		Mat34 pose = map->getSensorPose(i);
		Eigen::Matrix4f tmpPose = Eigen::Matrix4f(pose.matrix().cast<float>());

		// Creating color cloud
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3i>> colorPointCloud =
				RGBD::imageToColorPointCloud(rgbImage, depthImage,
						matcher->matcherParameters.cameraMatrixMat, tmpPose,
						depthImageScale);

		// We add every point
		for (int k = 0; k < colorPointCloud.size(); k++) {
			octomap::point3d endpoint((float) colorPointCloud[k].first.x(),
					(float) colorPointCloud[k].first.y(),
					(float) colorPointCloud[k].first.z());
			octomap::ColorOcTreeNode* n = octomapTree.get()->updateNode(
					endpoint,
					true);

			// Adding also color
			octomapTree.get()->integrateNodeColor(
					(float) colorPointCloud[k].first.x(),
					(float) colorPointCloud[k].first.y(),
					(float) colorPointCloud[k].first.z(),
					colorPointCloud[k].second.x(),
					colorPointCloud[k].second.y(),
					colorPointCloud[k].second.z());
		}

	}
	// We update the colors
	std::cout << "Updating tree color" << std::endl;
	octomapTree.get()->updateInnerOccupancy();
	// Writing octomap to file
	std::string filename(octomapFileToSave);
	std::cout << "Writing color tree to " << filename << std::endl;
	// write color tree
	octomapTree.get()->write(filename);
}

void PUTSLAM::createAndSaveOctomapOffline(double depthImageScale) {
	std::ifstream reconstructStr("reconstruction.res");

	int i = 0;
	while (1) {
		bool middleOfSequence = grabber->grab(); // grab frame
		if (!middleOfSequence)
			break;

		SensorFrame currentSensorFrame = grabber->getSensorFrame();
		double timeS, tx, ty, tz, qw, qx, qy, qz;
		reconstructStr >> timeS >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

		if (i % octomapCloudStepSize == 0) {
			std::cout << "Octomap uses point clouds with id = " << i
					<< std::endl;
			Eigen::Quaternion<float> Q(qw, qx, qy, qz);
			Eigen::Matrix4f tmpPose;
			tmpPose.setIdentity();
			tmpPose.block<3, 3>(0, 0) = Q.toRotationMatrix();
			tmpPose(0, 3) = tx;
			tmpPose(1, 3) = ty;
			tmpPose(2, 3) = tz;

			// Save for octomap
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3i>> colorPointCloud =
					RGBD::imageToColorPointCloud(currentSensorFrame.rgbImage,
							currentSensorFrame.depthImage,
							matcher->matcherParameters.cameraMatrixMat, tmpPose,
							depthImageScale);

			for (int k = 0; k < colorPointCloud.size(); k++) {
				octomap::point3d endpoint((float) colorPointCloud[k].first.x(),
						(float) colorPointCloud[k].first.y(),
						(float) colorPointCloud[k].first.z());
				octomap::ColorOcTreeNode* n = octomapTree.get()->updateNode(
						endpoint, true);

				octomapTree.get()->integrateNodeColor(
						(float) colorPointCloud[k].first.x(),
						(float) colorPointCloud[k].first.y(),
						(float) colorPointCloud[k].first.z(),
						colorPointCloud[k].second.x(),
						colorPointCloud[k].second.y(),
						colorPointCloud[k].second.z());
			}

		}
		i++;
	}

	// set inner node colors
	std::cout << "Updating tree color" << std::endl;
	octomapTree.get()->updateInnerOccupancy();

	std::string filename(octomapFileToSave);
	std::cout << "Writing color tree to " << filename << std::endl;

	// write color tree
	octomapTree.get()->write(filename);
}

void PUTSLAM::processFirstFrame(SensorFrame &currentSensorFrame,
		int &cameraPoseId) {
	matcher->Matcher::loadInitFeatures(currentSensorFrame);
	VOFeaturesSizeLog.push_back(matcher->getNumberOfFeatures());

	// cameraPose as Eigen::Transform
	Mat34 cameraPose = Mat34(VOPoseEstimate.cast<double>());

	// Add new position to the map
	if (!onlyVO) {
		cameraPoseId = map->addNewPose(cameraPose, currentSensorFrame.timestamp,
				currentSensorFrame.rgbImage, currentSensorFrame.depthImage);
	}

	// Correct motionModel
//	motionModelLastTime = currentSensorFrame.timestamp;
//	Mat34 x = motionModel.get()->correct(cameraPose);
//	motionModelPose = Eigen::Matrix4f(x.matrix().cast<float>());
}

//play trajectory
void PUTSLAM::startPlaying(std::string trajectoryFilename, int delayPlay) {
	///for inverse SLAM problem
	Simulator simulator;
	simulator.loadTrajectory(trajectoryFilename);
	std::vector<Mat34> traj = simulator.getTrajectory();
	int trajIt = 0;

	// Main loop
	while (true) {
		bool middleOfSequence = grabber->grab(); // grab frame
		if (!middleOfSequence)
			break;
		///for inverse SLAM problem
		if (trajIt > traj.size() - 1)
			break;
		SensorFrame currentSensorFrame = grabber->getSensorFrame();

		if (drawImages) {
			cv::imshow("PUTSLAM RGB frame", currentSensorFrame.rgbImage);
			cv::imshow("PUTSLAM Depth frame", currentSensorFrame.depthImage);
		}
		if (trajIt == 0) {
			// cameraPose as Eigen::Transform
			Mat34 cameraPose = Mat34(VOPoseEstimate.cast<double>());

			// Add new position to the map
			map->addNewPose(cameraPose, currentSensorFrame.timestamp,
					currentSensorFrame.rgbImage, currentSensorFrame.depthImage);
		} else {
			Eigen::Matrix4f transformation;

			//for inverse slam problem
			Mat34 transReal = traj[trajIt - 1].inverse() * traj[trajIt];
			transformation = transReal.cast<float>().matrix();
			std::cout << "iteration: " << trajIt << "\n";

			VOPoseEstimate = VOPoseEstimate * transformation;

			// cameraPose as Eigen::Transform
			Mat34 cameraPoseIncrement = Mat34(transformation.cast<double>());

			// Add new position to the map
			map->addNewPose(cameraPoseIncrement, currentSensorFrame.timestamp,
					currentSensorFrame.rgbImage, currentSensorFrame.depthImage);
		}
		usleep(delayPlay);
		trajIt++;
	}

	std::cout << "Job finished! Good bye :)" << std::endl;
}

/// set drawing options
void PUTSLAM::setDrawOptions(bool _draw, bool _drawImages) {
	visualize = _draw;
	drawImages = _drawImages;
	map->setDrawOptions(visualize);
}

// At beggining
void PUTSLAM::readingSomeParameters() {

	addFeaturesWhenMapSizeLessThan =
			((FeaturesMap*) map)->getAddFeaturesWhenMapSizeLessThan();
	addFeaturesWhenMeasurementSizeLessThan =
			((FeaturesMap*) map)->getAddFeaturesWhenMeasurementSizeLessThan();
	maxOnceFeatureAdd = ((FeaturesMap*) map)->getMaxOnceFeatureAdd();
	minEuclideanDistanceOfFeatures =
			((FeaturesMap*) map)->getMinEuclideanDistanceOfFeatures();
	minImageDistanceOfFeatures =
			((FeaturesMap*) map)->getMinImageDistanceOfFeatures();
	addNoFeaturesWhenMapSizeGreaterThan =
			((FeaturesMap*) map)->getAddNoFeaturesWhenMapSizeGreaterThan();
	minMeasurementsToAddPoseToFeatureEdge =
			((FeaturesMap*) map)->getMinMeasurementsToAddPoseToFeatureEdge();
	addPoseToPoseEdges = ((FeaturesMap*) map)->getAddPoseToPoseEdges();

	depthImageScale = ((FileGrabber*) grabber)->parameters.depthImageScale;

	getVisibleFeaturesGraphMaxDepth = 5000;
	getVisibleFeatureDistanceThreshold = 15.0;
}

void PUTSLAM::initialization() {
	// Optimize during trajectory acquisition
	if (optimizationThreadVersion == OPTTHREAD_ON)
        map->startOptimizationThread(1, 0);
	else if (optimizationThreadVersion == OPTTHREAD_ON_ROBUSTKERNEL)
		map->startOptimizationThread(1, 0, "Cauchy", 1);

	// Thread looking for too close features
	if (mapManagmentThreadVersion == MAPTHREAD_ON)
		map->startMapManagerThread(1);

	// thread for geometric loop closure
	if (loopClosureThreadVersion == LCTHREAD_ON)
		map->startLoopClosureThread(0, loopClosureMatcher);

	// Creating octomap
	if (octomap > 0)
		octomapTree.reset(new octomap::ColorOcTree(octomapResolution));

	if (octomapOffline > 0) {
		createAndSaveOctomapOffline(depthImageScale);
		exit(0);
	}
}

void PUTSLAM::loadConfigs() {
	tinyxml2::XMLDocument config;
	config.LoadFile("../../resources/configGlobal.xml");
	if (config.ErrorID())
		std::cout << "unable to load config file.\n";

	config.FirstChildElement("PUTSLAM")->QueryIntAttribute("verbose", &verbose);
	config.FirstChildElement("PUTSLAM")->QueryIntAttribute("onlyVO", &onlyVO);
	config.FirstChildElement("PUTSLAM")->QueryBoolAttribute("keepCameraFrames",
			&keepCameraFrames);
	config.FirstChildElement("PUTSLAM")->QueryIntAttribute("octomap", &octomap);
	config.FirstChildElement("PUTSLAM")->QueryDoubleAttribute(
			"octomapResolution", &octomapResolution);
	config.FirstChildElement("PUTSLAM")->QueryIntAttribute(
			"octomapCloudStepSize", &octomapCloudStepSize);
	octomapFileToSave = config.FirstChildElement("PUTSLAM")->Attribute(
			"octomapFileToSave");
	config.FirstChildElement("PUTSLAM")->QueryIntAttribute("octomapOffline",
			&octomapOffline);
	if (!keepCameraFrames && octomap)
		throw std::runtime_error(
				std::string(
						"Camera frames are not used (keepCameraFrames==false). Octomap is not available.\nModify config files.\n"));

	// Thread settings
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute("verbose",
			&verbose);
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute(
			"optimizationThreadVersion", &optimizationThreadVersion);
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute(
			"mapManagmentThreadVersion", &mapManagmentThreadVersion);
	config.FirstChildElement("ThreadSettings")->QueryIntAttribute(
			"loopClosureThreadVersion", &loopClosureThreadVersion);

	if (verbose > 0) {
		std::cout << "PUTSLAM: optimizationThreadVersion = "
				<< optimizationThreadVersion << std::endl;
		std::cout << "PUTSLAM: mapManagmentThreadVersion = "
				<< mapManagmentThreadVersion << std::endl;
	}

	// Create map
	if (verbose > 0) {
		std::cout << "Getting grabber config" << std::endl;
	}
	std::string configFileGrabber(
			config.FirstChildElement("Grabber")->FirstChildElement(
					"calibrationFile")->GetText());

	if (verbose > 0) {
		std::cout << "Getting map config" << std::endl;
	}
	std::string configFileMap(
			config.FirstChildElement("Map")->FirstChildElement("parametersFile")->GetText());

	if (verbose > 0) {
		std::cout << "Creating features map" << std::endl;
	}
	map = createFeaturesMap(configFileMap, configFileGrabber);
	map->setStoreImages(keepCameraFrames);

	if (verbose > 0) {
		std::cout << "Features map is initialized" << std::endl;
	}

	std::string grabberType(
			config.FirstChildElement("Grabber")->FirstChildElement("name")->GetText());

	std::string grabberConfigFile(
			config.FirstChildElement("Grabber")->FirstChildElement(
					"calibrationFile")->GetText());

	if (verbose > 0) {
		std::cout << "Creating grabber with type = " << grabberType
				<< std::endl;
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
	string matcherParametersLC =
			config.FirstChildElement("Matcher")->FirstChildElement(
					"parametersFileLC")->GetText();

	if (verbose > 0) {
		std::cout << "Creating matcher" << std::endl;
	}
	matcher = createMatcherOpenCV(matcherParameters, grabberConfigFile);
	if (verbose > 0) {
		cout << "Current matcher: " << matcher->getName() << std::endl;
	}
	loopClosureMatcher = createloopClosingMatcherOpenCV(matcherParametersLC,
			grabberConfigFile);
	if (verbose > 0) {
		cout << "Loop closure current matcher: " << matcher->getName()
				<< std::endl;
	}

	if (matcher->matcherParameters.MapMatchingVersion
			== Matcher::MatcherParameters::MAPMATCH_PATCHES
			|| matcher->matcherParameters.MapMatchingVersion
					== Matcher::MatcherParameters::MAPMATCH_XYZ_DESCRIPTORS_PATCHES) {
		if (!keepCameraFrames)
			throw std::runtime_error(
					std::string(
							"Camera frames are not used (keepCameraFrames==false). Matching with patches is not available.\nModify config files.\n"));
	}
}

Eigen::Matrix4f PUTSLAM::runVO(SensorFrame &currentSensorFrame, std::vector<cv::DMatch> &inlierMatches) {
	Stopwatch<> voTime;
	Eigen::Matrix4f transformation;
	double inlierRatio = matcher->Matcher::runVO(currentSensorFrame,
			transformation, inlierMatches);
	voTime.stop();
	timeMeasurement.voTimes.push_back(voTime.elapsed());

	VORansacInlierRatioLog.push_back(inlierRatio);
	VOFeaturesSizeLog.push_back(matcher->Matcher::getNumberOfFeatures());
	return transformation;
}

void PUTSLAM::addPoseToMap(SensorFrame &currentSensorFrame, Eigen::Matrix4f &poseIncrement, int &cameraPoseId ) {
	// cameraPose as Eigen::Transform
	Mat34 cameraPoseIncrement = Mat34(poseIncrement.cast<double>());

	// Add new position to the map
	Stopwatch<> tmp;
	tmp.start();
	cameraPoseId = map->addNewPose(cameraPoseIncrement,
			currentSensorFrame.timestamp, currentSensorFrame.rgbImage,
			currentSensorFrame.depthImage);
	tmp.stop();
	timeMeasurement.mapAddNewPoseTimes.push_back(tmp.elapsed());
}

Mat34 PUTSLAM::getMapPoseEstimate() {
	Stopwatch<> tmp;
	tmp.start();
	Mat34 cameraPose = map->getSensorPose();
	tmp.stop();
	timeMeasurement.mapGetSensorPoseTimes.push_back(tmp.elapsed());
	return cameraPose;
}

Eigen::Matrix4f PUTSLAM::getPoseIncrementFromMap(int frameCounter) {

	if ( frameCounter > 1 ) {
		Eigen::Matrix4f first( map->getSensorPose(frameCounter-2).matrix().cast<float>() );
		Eigen::Matrix4f second( map->getSensorPose(frameCounter-1).matrix().cast<float>() );
		return first.inverse() * second;
	}
	return Eigen::Matrix4f::Identity();
}

std::vector<MapFeature> PUTSLAM::getAndFilterFeaturesFromMap(SensorFrame &currentSensorFrame, Mat34 cameraPose,std::vector<int> &frameIds,
std::vector<float_type> &angles ) {

	Stopwatch<> tmp;
	tmp.start();
	std::vector<MapFeature> mapFeatures = map->getVisibleFeatures(cameraPose);
	tmp.stop();
	timeMeasurement.mapGetVisibleFeaturesTimes.push_back(tmp.elapsed());

	//mapFeatures = map->getVisibleFeatures(cameraPose, getVisibleFeaturesGraphMaxDepth, getVisibleFeatureDistanceThreshold);



	tmp.start();
	map->findNearestFrame(mapFeatures, frameIds, angles,
			matcher->matcherParameters.maxAngleBetweenFrames);
	tmp.stop();
	timeMeasurement.mapFindNearestFrameTimes.push_back(tmp.elapsed());

	//Remove features that we do not have a good observation angle
	tmp.start();
	removeMapFeaturesWithoutGoodObservationAngle(mapFeatures, frameIds, angles);
	tmp.stop();
	timeMeasurement.mapRemoveMapFeaturesTimes.push_back(tmp.elapsed());

	// Move mapFeatures to local coordinate system
	tmp.start();
	moveMapFeaturesToLocalCordinateSystem(cameraPose, mapFeatures);
	tmp.stop();
	timeMeasurement.mapMoveMapFeaturesToLCSTimes.push_back(
			tmp.elapsed());



	// Now lets check if those features are not behind sth
//	const double additionalDistance = 0.65f;
//	RGBD::removeMapFeaturesWithoutDepth(mapFeatures,
//			currentSensorFrame.depthImage, additionalDistance,
//			frameIds, angles, depthImageScale);

	showMapFeatures(currentSensorFrame.rgbImage, mapFeatures, 0, "Map Features after");

	// Check some asserts
//				assert(("PUTSLAM: mapFeatures, frameIdsand angles", mapFeatures.size()
//								== frameIds.size()));
//				assert(("PUTSLAM: mapFeatures, frameIdsand angles", mapFeatures.size()
//								== angles.size()));

	return mapFeatures;
}

// Processing
void PUTSLAM::startProcessing() {
	readingSomeParameters();
	initialization();

	int frameCounter = 0;
	auto startMainLoop = std::chrono::system_clock::now();
	SensorFrame lastSensorFrame;

	// Main loop
	while (true) {

		//getchar();

		// if loop was closed -> wait 10 seconds
		if (map->getAndResetLoopClosureSuccesful())
			usleep(10000000);


		// Get the frame to processing
		bool middleOfSequence = grabber->grab();
		if (!middleOfSequence)
			break;

        SensorFrame currentSensorFrame = grabber->getSensorFrame();

		if (drawImages) {
			cv::imshow("PUTSLAM RGB frame", currentSensorFrame.rgbImage);
			cv::imshow("PUTSLAM Depth frame", currentSensorFrame.depthImage);
		}

		int cameraPoseId = 0;
		bool addFeatureToMap = false;

		// Variable to store map features
		std::vector<MapFeature> mapFeatures;

		// The beginning of the sequence
		if (frameCounter == 0) {
			processFirstFrame(currentSensorFrame, cameraPoseId);

			addFeatureToMap = true;
		}
		// The next pose in the sequence
		else {

			// Running VO - matching or tracking depending on parameters
			// TODO:
			// - if no motion than skip frame
			std::vector<cv::DMatch> inlierMatches;
			Eigen::Matrix4f poseIncrement = runVO(currentSensorFrame, inlierMatches);

			VOPoseEstimate = VOPoseEstimate * poseIncrement;

			if (!onlyVO) {

				Stopwatch<> mapTime;
				mapTime.start();

				// Prediction from map
				//poseIncrement = getPoseIncrementFromMap(frameCounter);


				addPoseToMap(currentSensorFrame, poseIncrement, cameraPoseId);

				// Getting the pose
				Mat34 cameraPose = getMapPoseEstimate();

				// Get and filter the visible features
				// Find the ids of frames for which feature observations have the most similar angle
				std::vector<int> frameIds;
				std::vector<float_type> angles;
				mapFeatures = getAndFilterFeaturesFromMap(currentSensorFrame, cameraPose, frameIds, angles);


				if (verbose > 0)
					std::cout << "Returned visible map feature size: "
							<< mapFeatures.size() << std::endl;

				// Show map features
				if (matcher->matcherParameters.verbose > 0)
					showMapFeatures(currentSensorFrame.rgbImage, mapFeatures, 1);

				// Perform RANSAC matching and return measurements for found inliers in map compatible format
				// Remember! The match returns the list of inlier features from current pose!
				std::vector<MapFeature> measurementList;
				Eigen::Matrix4f mapEstimatedTransformation;

				// Map matching based only on descriptors
				double mapMatchingInlierRatio = 0.0f;

				Stopwatch<> tmp;
				tmp.start();
				if (matcher->matcherParameters.MapMatchingVersion
						== Matcher::MatcherParameters::MAPMATCH_DESCRIPTORS) {

					mapMatchingInlierRatio = matcher->Matcher::match(
							mapFeatures, cameraPoseId, measurementList,
							mapEstimatedTransformation);

				}
				// Map matching based on descriptors, but in a sphere around feature with set radius
				else if (matcher->matcherParameters.MapMatchingVersion
						== Matcher::MatcherParameters::MAPMATCH_XYZ_DESCRIPTORS) {

					bool newDetection = false;
					mapMatchingInlierRatio = matcher->Matcher::matchXYZ(
							mapFeatures, cameraPoseId, measurementList,
							mapEstimatedTransformation, newDetection, frameIds);

				} else {
					std::cout
							<< "Unrecognized map matching version -- double check matcherOpenCVParameters.xml"
							<< std::endl;
				}
				tmp.stop();
				timeMeasurement.mapMatchingTimes.push_back(tmp.elapsed());

				MapMatchingRansacInlierRatioLog.push_back(
						mapMatchingInlierRatio);

				// TESTING VO with map corrections
				VoMapPose = VoMapPose * poseIncrement
						* mapEstimatedTransformation;

				if (verbose > 0)
					std::cout << "Measurement to features in graph size : "
							<< measurementList.size() << std::endl;

				// Compare VO and VOMap estimate -> decide whether to add measurements to map
				float distanceDiff = mapEstimatedTransformation.block<3, 1>(0,
						3).norm();
//				std::cout << "Difference between VO and Map : " << distanceDiff
//						<< " meters" << std::endl;

				tmp.start();
//				if (addPoseToPoseEdges) {
//					Mat34 cameraPoseIncrement = Mat34(poseIncrement.cast<double>());
//					map->addMeasurement(cameraPoseId - 1, cameraPoseId,
//							cameraPoseIncrement);
//				}

				// Add pose-feature constrain
				measurementToMapSizeLog.push_back(measurementList.size());
				if (measurementList.size()
						> minMeasurementsToAddPoseToFeatureEdge) {
					if (map->useUncertainty()) {
						matcher->computeNormals(currentSensorFrame.depthImage,
								measurementList,
								currentSensorFrame.depthImageScale);
						matcher->computeRGBGradients(
								currentSensorFrame.rgbImage,
								currentSensorFrame.depthImage, measurementList,
								currentSensorFrame.depthImageScale);
					}
					map->addMeasurements(measurementList);
				}
				// Add pose-pose constrain - depends on config file
				//if (addPoseToPoseEdges) {
				else {
					Mat34 cameraPoseIncrement = Mat34(
							poseIncrement.cast<double>());
					map->addMeasurement(cameraPoseId - 1, cameraPoseId,
							cameraPoseIncrement);
				}

				tmp.stop();
				timeMeasurement.mapAddMeasurementTimes.push_back(tmp.elapsed());

				// Insufficient number of features -> time to add some features
				if (mapFeatures.size() < addFeaturesWhenMapSizeLessThan
						|| (measurementList.size()
								< addFeaturesWhenMeasurementSizeLessThan
								&& mapFeatures.size()
										< addNoFeaturesWhenMapSizeGreaterThan)) {
					addFeatureToMap = true;
				}

				mapTime.stop();
				timeMeasurement.mapTimes.push_back(mapTime.elapsed());
			}
		}

		// Should we add some features to the map?
		if (addFeatureToMap && !onlyVO) {
			if (verbose > 0)
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
			if (map->useUncertainty()) {
				matcher->computeNormals(currentSensorFrame.depthImage,
						mapFeaturesToAdd, currentSensorFrame.depthImageScale);
				matcher->computeRGBGradients(currentSensorFrame.rgbImage,
						currentSensorFrame.depthImage, mapFeaturesToAdd,
						currentSensorFrame.depthImageScale);
			}

			// Finally, adding to map
			map->addFeatures(mapFeaturesToAdd, cameraPoseId);

			if (verbose > 0)
				std::cout << "map->addFeatures -> added " << addedCounter
						<< " features" << std::endl;

			addFeatureToMap = false;
		}

		// Saving features for Dominik
		//		Matcher::featureSet features = matcher->getFeatures();
		//		saveFeaturesToFile(features, currentSensorFrame.timestamp);

		// Save trajectory
		saveTrajectoryFreiburgFormat(VOPoseEstimate, trajectoryFreiburgStream,
				currentSensorFrame.timestamp);

		saveTrajectoryFreiburgFormat(VoMapPose, trajectoryVOMapStream,
				currentSensorFrame.timestamp);

		mapSize.push_back(map->getNumberOfFeatures());

		frameCounter++;
		std::cout << frameCounter << " " << std::flush;

		lastSensorFrame = currentSensorFrame;
	}
	auto elapsed = std::chrono::duration_cast < std::chrono::milliseconds
			> (std::chrono::system_clock::now() - startMainLoop);
	saveFPS(double(frameCounter) / (elapsed.count() / 1000.0));

	saveStatistics();

	std::cout << "Job finished! Good bye :)" << std::endl;
}

void PUTSLAM::removeMapFeaturesWithoutGoodObservationAngle(
		std::vector<MapFeature> &mapFeatures, std::vector<int> &frameIds,
		std::vector<float_type> &angles) {
	auto mapFeaturesIter = mapFeatures.begin();
	auto frameIdsIter = frameIds.begin();
	auto anglesIter = angles.begin();

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
}

// At the end

void PUTSLAM::saveStatistics() {

	map->save2file("createdMapFile.map", "preOptimizedGraphFile.g2o");

	// Wait for management thread to finish
	if (mapManagmentThreadVersion == MAPTHREAD_ON)
		map->finishManagementThr();  // Wait for optimization thread to finish

	// thread for geometric loop closure
	if (loopClosureThreadVersion == LCTHREAD_ON)
		map->finishLoopClosureThr();

	// We optimize only at the end if that version is chosen
	if (optimizationThreadVersion == OPTTHREAD_ATEND)
        map->startOptimizationThread(1, 1);

	// Wait for optimization thread to finish
	if (optimizationThreadVersion != OPTTHREAD_OFF)
		map->finishOptimization("graph_trajectory.res",
				"optimizedGraphFile.g2o");

	///for inverse SLAM problem
	//    for (int i=0; i<traj.size();i++){
	//        VertexSE3 vert(i, traj[i], i);
	//        ((FeaturesMap*) map)->updatePose(vert, true);
	//    }
	if (optimizationThreadVersion != OPTTHREAD_OFF)
		map->exportOutput("graph_trajectory.res", "optimizedGraphFile.g2o");

	// Save times
	std::cout << "Saving times" << std::endl;
	timeMeasurement.saveToFile();

	// Save statistics
	std::cout << "Saving logs to file" << std::endl;
	saveLogs();

	// Close trajectory stream
	trajectoryFreiburgStream.close();
	trajectoryVOMapStream.close();
	trajectoryMotionModelStream.close();

	// Run statistics
	std::cout << "Evaluating trajectory" << std::endl;
	evaluateResults(((FileGrabber*) grabber)->parameters.basePath,
			((FileGrabber*) grabber)->parameters.datasetName);

	// Save map
	std::cout << "Saving to octomap" << std::endl;
	if (octomap > 0)
		createAndSaveOctomap(depthImageScale);
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

	// VOFeaturesSizeLog
	statisticsLogStream << "VOFeaturesSizeLog = np.array([";
	for (int a = 0; a < VOFeaturesSizeLog.size(); a++) {
		statisticsLogStream << VOFeaturesSizeLog[a] << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream
			<< "plt.plot(VOFeaturesSizeLog)"
			<< endl;
	statisticsLogStream
			<< "fig.suptitle('Number of features used in VO', fontsize=20)" << endl;
	statisticsLogStream << "plt.xlabel('Frame counter', fontsize=18)" << endl;
	statisticsLogStream << "plt.ylabel('Number of features', fontsize=16)" << endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('VOFeaturesSize.png')" << endl;


	// mapSize
	statisticsLogStream << "mapSize = np.array([";
		for (int a = 0; a < mapSize.size(); a++) {
			statisticsLogStream << mapSize[a] << ", ";
		}
		statisticsLogStream << "]);" << std::endl;

		statisticsLogStream << "fig = plt.figure()" << endl;
		statisticsLogStream
				<< "plt.plot(mapSize)"
				<< endl;
		statisticsLogStream
				<< "fig.suptitle('Number of features in map', fontsize=20)" << endl;
		statisticsLogStream << "plt.xlabel('Frame counter', fontsize=18)" << endl;
		statisticsLogStream << "plt.ylabel('Features counter', fontsize=16)" << endl;
		statisticsLogStream << "plt.legend() " << endl;
		statisticsLogStream << "plt.savefig('mapSize.png')" << endl;


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


	// LC matches
	vector<double> lcMatchingRatiosLog = map->getLoopClosureMatchingRatiosLog();
	statisticsLogStream << "lcMatchingRatiosLog = np.array([";
	for (int a = 0; a < lcMatchingRatiosLog.size(); a++) {
		statisticsLogStream << lcMatchingRatiosLog[a] << ", ";
	}
	statisticsLogStream << "]);" << std::endl;

	statisticsLogStream << "fig = plt.figure()" << endl;
	statisticsLogStream << "plt.plot(lcMatchingRatiosLog)" << endl;
	statisticsLogStream << "fig.suptitle('Matching ratio in LC', fontsize=20)"
			<< endl;
	statisticsLogStream << "plt.xlabel('Compared pair no.', fontsize=18)"
			<< endl;
	statisticsLogStream << "plt.ylabel('Ratio', fontsize=16)" << endl;
	statisticsLogStream << "plt.legend() " << endl;
	statisticsLogStream << "plt.savefig('LCMatchingRatio.png')" << endl;

	statisticsLogStream.close();

	vector<pair<int,int>> lcAnalyzedPairsLog = map->getLoopClosureAnalyzedPairsLog();
	std::ofstream lcAnalyzedPairsStream("LCAnalyzedPairs.log");
	for ( auto &p : lcAnalyzedPairsLog) {
		lcAnalyzedPairsStream << p.first << " " << p.second << std::endl;
	}
	lcAnalyzedPairsStream.close();

}

void PUTSLAM::evaluateResults(std::string basePath, std::string datasetName) {

	std::string fullPath = basePath + "/" + datasetName + "/";

//	std::string evalATEVO =
//			"python2 ../../scripts/evaluate_ate.py " + fullPath
//					+ "groundtruth.txt VO_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot VOAte.png > VOAte.res";
//	std::string evalATEMap =
//			"python2 ../../scripts/evaluate_ate.py " + fullPath
//					+ "groundtruth.txt graph_trajectory.res --verbose --scale 1 --save_associations ate_association.res --plot g2oAte.png > g2oAte.res";
//	std::string evalRPEVO =
//			"python2 ../../scripts/evaluate_rpe.py " + fullPath
//					+ "groundtruth.txt VO_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot VORpe.png > VORpe.res";
//	std::string evalRPEMap =
//			"python2 ../../scripts/evaluate_rpe.py " + fullPath
//					+ "groundtruth.txt graph_trajectory.res --verbose --delta_unit 'f' --fixed_delta --plot g2oRpe.png > g2oRpe.res";
//	try {
//		int tmp = std::system(evalATEVO.c_str());
//		tmp = std::system(evalATEMap.c_str());
//		tmp = std::system(evalRPEVO.c_str());
//		tmp = std::system(evalRPEMap.c_str());
//	} catch (std::system_error& error) {
//		std::cout << "Error: " << error.code() << " - " << error.what() << '\n';
//	}

	ofstream datasetNameStream("DatasetName");
	datasetNameStream << fullPath;
	datasetNameStream.close();

}

void PUTSLAM::showMapFeatures(cv::Mat rgbImage,
		std::vector<MapFeature> mapFeatures, int wait, string windowName) {
	std::vector<cv::KeyPoint> mapFeatures2D(mapFeatures.size());
	std::transform(mapFeatures.begin(), mapFeatures.end(),
			mapFeatures2D.begin(),
			[](const MapFeature& m) {return cv::KeyPoint(m.u, m.v, 3);});

	cv::Mat img2draw;
	cv::drawKeypoints(rgbImage, mapFeatures2D, img2draw, cv::Scalar(0, 255, 0));

	cv::imshow(windowName.c_str(), img2draw);
	if ( wait )
		cv::waitKey(10000);
	else
		cv::waitKey(1);
}

