#include "../include/Map/featuresMap.h"
#include "../include/PoseGraph/graph.h"
#include <memory>
#include <stdexcept>
#include <chrono>

using namespace putslam;

/// A single instance of Elevation Map
FeaturesMap::Ptr map;

FeaturesMap::FeaturesMap(void) :
		featureIdNo(FATURES_START_ID), Map("Features Map", MAP_FEATURES) {
	poseGraph = createPoseGraphG2O();
}

/// Construction
FeaturesMap::FeaturesMap(std::string configMap,
        std::string sensorConfig) : config(configMap),
		featureIdNo(FATURES_START_ID), sensorModel(sensorConfig), Map(
				"Features Map", MAP_FEATURES) {
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
	Mat34 cameraPose =
			(poseId >= 0) ? camTrajectory[poseId] : camTrajectory.back();

    bufferMapFrontend.mtxBuffer.lock();
    for (std::vector<RGBDFeature>::const_iterator it = features.begin(); it != features.end(); it++) {//.. and the graph
        //feature pose in the global frame
        Mat34 featurePos((*it).position);
        featurePos = cameraPose.matrix() * featurePos.matrix();

        //add each feature to map structure...
        Vec3 featurePositionInGlobal(featurePos.translation());
        bufferMapFrontend.features2add.push_back(
                    MapFeature(featureIdNo, 0, 0, featurePositionInGlobal,
                               std::vector<unsigned int>(), (*it).descriptors));
        //add measurement to the graph
        Mat33 info(Mat33::Identity());
        if (config.useUncertainty)
            info = sensorModel.informationMatrixFromImageCoordinates(it->u, it->v, (*it).position.z());

        Edge3D e((*it).position, info, camTrajectory.size() - 1, featureIdNo);
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
int FeaturesMap::addNewPose(const Mat34& cameraPose, float_type timestamp) {
	//add camera pose to the map
	camTrajectory.push_back(cameraPose);
	//add camera pose to the graph
	poseGraph->addVertexPose(
			VertexSE3(camTrajectory.size() - 1,
					Vec3(cameraPose(0, 3), cameraPose(1, 3), cameraPose(2, 3)),
					Quaternion(cameraPose.rotation()), timestamp));
	return camTrajectory.size() - 1;
}

/// add measurements (features measured from the last camera pose)
void FeaturesMap::addMeasurements(const std::vector<MapFeature>& features,
		int poseId) {
	unsigned int _poseId = (poseId >= 0) ? poseId : (camTrajectory.size() - 1);
	for (std::vector<MapFeature>::const_iterator it = features.begin();
			it != features.end(); it++) {
		//add measurement
        Mat33 info(Mat33::Identity());

//		info = sensorModel.informationMatrix((*it).position.x(),
//				(*it).position.y(), (*it).position.z());
        if (config.useUncertainty)
            info = sensorModel.informationMatrixFromImageCoordinates(it->u,
					it->v, (*it).position.z());

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
	return (poseId >= 0) ? camTrajectory[poseId] : camTrajectory.back();
}

/// start optimization thread
void FeaturesMap::startOptimizationThread(unsigned int iterNo, int verbose) {
	optimizationThr.reset(
			new std::thread(&FeaturesMap::optimize, this, iterNo, verbose));
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
void FeaturesMap::optimize(unsigned int iterNo, int verbose) {
	// graph optimization
	continueOpt = true;

	// Wait for some information in map
	while (continueOpt && emptyMap) {
		 std::this_thread::sleep_for(  std::chrono::milliseconds( 200 ) );
	}

	while (continueOpt) {
		if (verbose)
			std::cout << "start optimization\n";
		poseGraph->optimize(iterNo, verbose);
		if (verbose)
			std::cout << "end optimization\n";
	}

	// Final optimization
	std::cout<<"Starting final after trajectory optimization"<<std::endl;
	//poseGraph->optimize(-1, verbose, 0.0001);
	poseGraph->optimize(50, verbose);
	poseGraph->optimize(100, verbose);
}

/// Update map
void FeaturesMap::updateMap(MapModifier& modifier, std::vector<MapFeature>& featuresMap, std::recursive_mutex& mutex){
    if (mutex.try_lock()){//try to lock graph
        modifier.mtxBuffer.lock();
        if (modifier.addFeatures()){
            featuresMap.insert(featuresMap.end(), modifier.features2add.begin(), modifier.features2add.end());
            modifier.features2add.clear();
        }
        modifier.mtxBuffer.unlock();
        mutex.unlock();
    }
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
