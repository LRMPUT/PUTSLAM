#include "../include/Map/featuresMap.h"
#include "../include/PoseGraph/graph.h"
#include <memory>
#include <stdexcept>
#include <chrono>

using namespace putslam;

/// A single instance of Elevation Map
FeaturesMap::Ptr map;

FeaturesMap::FeaturesMap(void) :
		featureIdNo(FATURES_START_ID), lastOptimizedPose(0), Map("Features Map",
				MAP_FEATURES) {
	poseGraph = createPoseGraphG2O();
}

/// Construction
FeaturesMap::FeaturesMap(std::string configMap, std::string sensorConfig) :
		config(configMap), featureIdNo(FATURES_START_ID), sensorModel(
				sensorConfig), lastOptimizedPose(0), Map("Features Map",
				MAP_FEATURES) {
	tinyxml2::XMLDocument config;
	std::string filename = "../../resources/" + configMap;
	config.LoadFile(filename.c_str());
	if (config.ErrorID())
		std::cout << "unable to load config file.\n";
	poseGraph = createPoseGraphG2O();

	// set that map is currently empty
	emptyMap = true;
}

/// Destruction
FeaturesMap::~FeaturesMap(void) {
}

const std::string& FeaturesMap::getName() const {
	return name;
}

/// Add NEW features to the map
/// Position of features in relation to camera pose
void FeaturesMap::addFeatures(const std::vector<RGBDFeature>& features,
		int poseId) {
	mtxCamTraj.lock();
	int camTrajSize = camTrajectory.size();
	Mat34 cameraPose =
			(poseId >= 0) ?
					camTrajectory[poseId].pose : camTrajectory.back().pose;
	mtxCamTraj.unlock();

	bufferMapFrontend.mtxBuffer.lock();
	for (std::vector<RGBDFeature>::const_iterator it = features.begin();
			it != features.end(); it++) { //.. and the graph
		//feature pose in the global frame
		Mat34 featurePos((*it).position);
		featurePos = cameraPose.matrix() * featurePos.matrix();

		// Add pose id
		std::vector<unsigned int> poseIds;
		poseIds.push_back(poseId);

		//add each feature to map structure...
		Vec3 featurePositionInGlobal(featurePos.translation());
		bufferMapFrontend.features2add.push_back(
				MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
						poseIds, (*it).descriptors));
		//add measurement to the graph
		Mat33 info(Mat33::Identity());
		if (config.useUncertainty)
			info = sensorModel.informationMatrixFromImageCoordinates(it->u,
					it->v, (*it).position.z());

		Edge3D e((*it).position, info, camTrajSize - 1, featureIdNo);
		poseGraph->addVertexFeature(
				Vertex3D(featureIdNo,
						Vec3(featurePos(0, 3), featurePos(1, 3),
								featurePos(2, 3))));
		poseGraph->addEdge3D(e);
		featureIdNo++;
	}
	bufferMapFrontend.mtxBuffer.unlock();

	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);

	emptyMap = false;
}

/// add new pose of the camera, returns id of the new pose
int FeaturesMap::addNewPose(const Mat34& cameraPoseChange,
		float_type timestamp) {
	//add camera pose to the map
	mtxCamTraj.lock();
	int trajSize = camTrajectory.size();
	if (trajSize == 0) {
		odoMeasurements.push_back(Mat34::Identity());
		VertexSE3 camPose(trajSize, cameraPoseChange, timestamp);
		camTrajectory.push_back(camPose);

		mtxCamTraj.unlock();

		//add camera pose to the graph
		poseGraph->addVertexPose(camPose);

	} else {
		odoMeasurements.push_back(cameraPoseChange);
		VertexSE3 camPose(trajSize,
				camTrajectory.back().pose * cameraPoseChange, timestamp);
		camTrajectory.push_back(camPose);

		mtxCamTraj.unlock();

		//add camera pose to the graph
		poseGraph->addVertexPose(camPose);
	}
	return trajSize;
}

/// add measurements (features measured from the last camera pose)
void FeaturesMap::addMeasurements(const std::vector<MapFeature>& features,
		int poseId) {
	mtxCamTraj.lock();
	int camTrajSize = camTrajectory.size();
	mtxCamTraj.unlock();
	unsigned int _poseId = (poseId >= 0) ? poseId : (camTrajSize - 1);
	for (std::vector<MapFeature>::const_iterator it = features.begin();
			it != features.end(); it++) {
		//add measurement
		Mat33 info(Mat33::Identity());

//		info = sensorModel.informationMatrix((*it).position.x(),
//				(*it).position.y(), (*it).position.z());
        if (config.useUncertainty){
			info = sensorModel.informationMatrixFromImageCoordinates(it->u,
					it->v, (*it).position.z());
        }

		Edge3D e((*it).position, info, _poseId, (*it).id);
		poseGraph->addEdge3D(e);
	}
}

/// Get all features
std::vector<MapFeature> FeaturesMap::getAllFeatures(void) {
	mtxMapFrontend.lock();
	std::vector<MapFeature> featuresSet(featuresMapFrontend);
	mtxMapFrontend.unlock();
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
	return featuresSet;
}

/// Get feature position
Vec3 FeaturesMap::getFeaturePosition(unsigned int id) {
	mtxMapFrontend.lock();
	Vec3 feature(featuresMapFrontend[FATURES_START_ID + id].position);
	mtxMapFrontend.unlock();
	return feature;
}

/// get all visible features
std::vector<MapFeature> FeaturesMap::getVisibleFeatures(
		const Mat34& cameraPose) {
	std::vector<MapFeature> visibleFeatures;
	mtxMapFrontend.lock();
	for (std::vector<MapFeature>::iterator it = featuresMapFrontend.begin();
			it != featuresMapFrontend.end(); it++) {
		Mat34 featurePos((*it).position);
		Mat34 featureCam = cameraPose.inverse() * featurePos;
		Eigen::Vector3d pointCam = sensorModel.inverseModel(featureCam(0, 3),
				featureCam(1, 3), featureCam(2, 3));
		//std::cout << pointCam(0) << " " << pointCam(1) << " " << pointCam(2) << "\n";
		if (pointCam(0) != -1) {
			visibleFeatures.push_back(*it);
		}
	}
	mtxMapFrontend.unlock();
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
	return visibleFeatures;
}

/// get pose of the sensor (default: last pose)
Mat34 FeaturesMap::getSensorPose(int poseId) {
	mtxCamTraj.lock();
	Mat34 pose;
	if (poseId < 0)
		poseId = camTrajectory.size() - 1;
	if (poseId < lastOptimizedPose)
		pose = camTrajectory[poseId].pose;
	else {
		pose = camTrajectory[lastOptimizedPose].pose;
		for (int i = lastOptimizedPose + 1; i < odoMeasurements.size(); i++) {
			pose.matrix() = pose.matrix() * odoMeasurements[i].matrix();
		}
	}
	mtxCamTraj.unlock();
	return pose;
}

/// start optimization thread
void FeaturesMap::startOptimizationThread(unsigned int iterNo, int verbose,
		std::string RobustKernelName, float_type kernelDelta) {
	optimizationThr.reset(
			new std::thread(&FeaturesMap::optimize, this, iterNo, verbose,
					RobustKernelName, kernelDelta));
}

/// Wait for optimization thread to finish
void FeaturesMap::finishOptimization(std::string trajectoryFilename,
		std::string graphFilename) {
	continueOpt = false;
	optimizationThr->join();
	poseGraph->export2RGBDSLAM(trajectoryFilename);
	poseGraph->save2file(graphFilename);
}

/// optimization thread
void FeaturesMap::optimize(unsigned int iterNo, int verbose,
		std::string RobustKernelName, float_type kernelDelta) {
	// graph optimization
	continueOpt = true;

	// Wait for some information in map
	while (continueOpt && emptyMap) {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	while (continueOpt) {
		if (verbose)
			std::cout << "start optimization\n";
		if (!RobustKernelName.empty()) {
			setRobustKernel(RobustKernelName, kernelDelta);
		} else
			disableRobustKernel();
		poseGraph->optimize(iterNo, verbose);
		std::vector<MapFeature> optimizedFeatures;
		((PoseGraphG2O*) poseGraph)->getOptimizedFeatures(optimizedFeatures);
		bufferMapFrontend.mtxBuffer.lock();
		bufferMapFrontend.features2update.insert(
				bufferMapFrontend.features2update.begin(),
				optimizedFeatures.begin(), optimizedFeatures.end());
		bufferMapFrontend.mtxBuffer.unlock();
		//try to update the map
		updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
		//update camera trajectory
		std::vector<VertexSE3> optimizedPoses;
		((PoseGraphG2O*) poseGraph)->getOptimizedPoses(optimizedPoses);
		updateCamTrajectory(optimizedPoses);
        if (config.fixVertices)
            ((PoseGraphG2O*)poseGraph)->fixOptimizedVertices();
		if (verbose)
			std::cout << "end optimization\n";
	}

	// Final optimization
	if (!RobustKernelName.empty())
		setRobustKernel(RobustKernelName, kernelDelta);
	else
		disableRobustKernel();
	std::cout << "Starting final after trajectory optimization" << std::endl;
    if (config.fixVertices)
        ((PoseGraphG2O*)poseGraph)->releaseFixedVertices();
	//poseGraph->optimize(-1, verbose, 0.0001);
	poseGraph->optimize(100, verbose);

	std::vector<MapFeature> optimizedFeatures;
	((PoseGraphG2O*) poseGraph)->getOptimizedFeatures(optimizedFeatures);
	bufferMapFrontend.mtxBuffer.lock();
	bufferMapFrontend.features2update.insert(
			bufferMapFrontend.features2update.begin(),
			optimizedFeatures.begin(), optimizedFeatures.end());
	bufferMapFrontend.mtxBuffer.unlock();
//    std::cout<<"features 2 update2 " << bufferMapFrontend.features2update.size() <<"\n";
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
	//update camera trajectory
	std::vector<VertexSE3> optimizedPoses;
	((PoseGraphG2O*) poseGraph)->getOptimizedPoses(optimizedPoses);
	updateCamTrajectory(optimizedPoses);
}

/// Update map
void FeaturesMap::updateMap(MapModifier& modifier,
		std::vector<MapFeature>& featuresMap, std::recursive_mutex& mutex) {
	if (mutex.try_lock()) {    //try to lock graph
		modifier.mtxBuffer.lock();
		if (modifier.addFeatures()) {
			featuresMap.insert(featuresMap.end(), modifier.features2add.begin(),
					modifier.features2add.end());
			modifier.features2add.clear();
		}
		if (modifier.updateFeatures()) {
			for (std::vector<MapFeature>::iterator it =
					modifier.features2update.begin();
					it != modifier.features2update.end(); it++) {
				updateFeature(featuresMap, *it);
			}
			modifier.features2update.clear();
		}
		modifier.mtxBuffer.unlock();
		mutex.unlock();
	}
}

/// Update feature
void FeaturesMap::updateFeature(std::vector<MapFeature>& featuresMap,
		MapFeature& newFeature) {
	for (std::vector<MapFeature>::iterator it = featuresMap.begin();
			it != featuresMap.end(); it++) {
		if (it->id == newFeature.id) {
			it->position = newFeature.position;
		}
	}
}

/// Update camera trajectory
void FeaturesMap::updateCamTrajectory(std::vector<VertexSE3>& poses2update) {
	for (std::vector<VertexSE3>::iterator it = poses2update.begin();
			it != poses2update.end(); it++) {
		updatePose(*it);
	}
}

/// Update pose
void FeaturesMap::updatePose(VertexSE3& newPose) {
	if (newPose.vertexId > lastOptimizedPose)
		lastOptimizedPose = newPose.vertexId;
	mtxCamTraj.lock();
	for (std::vector<VertexSE3>::iterator it = camTrajectory.begin();
			it != camTrajectory.end(); it++) {
		if (it->vertexId == newPose.vertexId) {
			it->pose = newPose.pose;
		}
	}
	mtxCamTraj.unlock();
}

/// Save map to file
void FeaturesMap::save2file(std::string mapFilename,
		std::string graphFilename) {
	poseGraph->save2file(graphFilename);
	std::ofstream file(mapFilename);
	mtxMapFrontend.lock();
	file << "#Legend:\n";
	file << "#Pose pose_id pose(0,0) pose(1,0) ... pose(2,3)\n";
	file
			<< "#Feature feature_id feature_x feature_y feature_z feature_u feature_v\n";
	file << "#FeaturePosesIds pose_id1 pose_id2 ...\n";
	file
			<< "#FeatureExtendedDescriptors size pose_id1 descriptor.cols descriptor.rows desc1(0,0) desc1(1,0)...\n";
	for (std::vector<VertexSE3>::iterator it = camTrajectory.begin();
			it != camTrajectory.end(); it++) {
		file << "Pose " << it->vertexId;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				file << " " << it->pose(i, j);
		file << "\n";
	}
	for (std::vector<MapFeature>::iterator it = featuresMapFrontend.begin();
			it != featuresMapFrontend.end(); it++) {
		file << "Feature " << it->id << " " << it->position.x() << " "
				<< it->position.y() << " " << it->position.z() << " " << it->u
				<< " " << it->v << "\n";
		file << "FeaturePoseIds";
		for (std::vector<unsigned int>::iterator iter = it->posesIds.begin();
				iter != it->posesIds.end(); iter++) {
			file << " " << *iter;
		}
		file << "\n";
		file << "FeatureExtendedDescriptors " << it->descriptors.size() << " ";
		for (std::vector<ExtendedDescriptor>::iterator iter =
				it->descriptors.begin(); iter != it->descriptors.end();
				iter++) {
			file << iter->poseId << " " << iter->descriptor.cols << " "
					<< iter->descriptor.rows;
			for (int i = 0; i < iter->descriptor.cols; i++)
				for (int j = 0; j < iter->descriptor.rows; j++) {
					file << " " << iter->descriptor.at<double>(i, j);
				}
			file << "\n";
		}
		file << "\n";
	}
	mtxMapFrontend.unlock();
	file.close();
}

/// set Robust Kernel
void FeaturesMap::setRobustKernel(std::string name, float_type delta) {
	((PoseGraphG2O*) poseGraph)->setRobustKernel(name, delta);
}

/// disable Robust Kernel
void FeaturesMap::disableRobustKernel(void) {
	((PoseGraphG2O*) poseGraph)->disableRobustKernel();
}

putslam::Map* putslam::createFeaturesMap(void) {
	map.reset(new FeaturesMap());
	return map.get();
}

putslam::Map* putslam::createFeaturesMap(std::string configFileGrabber,
		std::string configSensor) {
	map.reset(new FeaturesMap(configFileGrabber, configSensor));
	return map.get();
}
