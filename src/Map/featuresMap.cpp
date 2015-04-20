#include "../include/Map/featuresMap.h"
#include "../include/PoseGraph/graph.h"
#include <memory>
#include <stdexcept>
#include <chrono>

using namespace putslam;

/// A single instance of Features Map
FeaturesMap::Ptr map;

FeaturesMap::FeaturesMap(void) :
        featureIdNo(FEATURES_START_ID), lastOptimizedPose(0), Map("Features Map",
				MAP_FEATURES) {
	poseGraph = createPoseGraphG2O();
}

/// Construction
FeaturesMap::FeaturesMap(std::string configMap, std::string sensorConfig) :
        config(configMap), featureIdNo(FEATURES_START_ID), sensorModel(
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
    Mat34 cameraPose = getSensorPose(poseId);
    mtxCamTraj.lock();
    int camTrajSize = camTrajectory.size();
	mtxCamTraj.unlock();

	bufferMapFrontend.mtxBuffer.lock();
	for (std::vector<RGBDFeature>::const_iterator it = features.begin();
            it != features.end(); it++) { // update the graph

        mtxCamTraj.lock();
        camTrajectory[poseId].featuresIds.insert(featureIdNo);
        mtxCamTraj.unlock();

		//feature pose in the global frame
		Mat34 featurePos((*it).position);
		featurePos = cameraPose.matrix() * featurePos.matrix();

		// Add pose id
		std::vector<unsigned int> poseIds;
        poseIds.push_back(poseId);

		//add each feature to map structure...
		Vec3 featurePositionInGlobal(featurePos.translation());
        bufferMapFrontend.features2add[featureIdNo] = MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
                        poseIds, (*it).descriptors);
        bufferMapManagement.mtxBuffer.lock();
        bufferMapManagement.features2add[featureIdNo] =
                MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
                        poseIds, (*it).descriptors);
        bufferMapManagement.mtxBuffer.unlock();
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

    bufferMapManagement.mtxBuffer.unlock();
    //try to update the map
    updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);

	emptyMap = false;
}

/// add new pose of the camera, returns id of the new pose
int FeaturesMap::addNewPose(const Mat34& cameraPoseChange,
        float_type timestamp, cv::Mat image, cv::Mat depthImage) {
    //add camera pose to the map
    imageSeq.push_back(image);
    depthSeq.push_back(depthImage);

	int trajSize = camTrajectory.size();
	if (trajSize == 0) {
		odoMeasurements.push_back(Mat34::Identity());
		VertexSE3 camPose(trajSize, cameraPoseChange, timestamp);

        mtxCamTraj.lock();
		camTrajectory.push_back(camPose);
		mtxCamTraj.unlock();

        //add camera pose to the graph
		poseGraph->addVertexPose(camPose);

    } else {
		odoMeasurements.push_back(cameraPoseChange);
		VertexSE3 camPose(trajSize,
                getSensorPose() * cameraPoseChange, timestamp);
        mtxCamTraj.lock();
		camTrajectory.push_back(camPose);
		mtxCamTraj.unlock();

        //add camera pose to the graph
		poseGraph->addVertexPose(camPose);
    }
	return trajSize;
}

/// get n-th image and depth image from the sequence
void FeaturesMap::getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage){
    if (poseNo<imageSeq.size()){
        image = imageSeq[poseNo];
        depthImage = depthSeq[poseNo];
    }
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

        mtxCamTraj.lock();
        camTrajectory[_poseId].featuresIds.insert(featureIdNo);
        mtxCamTraj.unlock();

        //add measurement
		Mat33 info(Mat33::Identity());

//		info = sensorModel.informationMatrix((*it).position.x(),
//				(*it).position.y(), (*it).position.z());
        if (config.useUncertainty){
			info = sensorModel.informationMatrixFromImageCoordinates(it->u,
					it->v, (*it).position.z());
        }

        featuresMapFrontend[it->id].posesIds.push_back(_poseId);

		Edge3D e((*it).position, info, _poseId, (*it).id);
		poseGraph->addEdge3D(e);
    }
}

/// add measurement between two poses
void FeaturesMap::addMeasurement(int poseFrom, int poseTo, Mat34 transformation){
    EdgeSE3 e(transformation, Mat66::Identity(), poseFrom, poseTo);
    poseGraph->addEdgeSE3(e);
}

/// Get all features
std::vector<MapFeature> FeaturesMap::getAllFeatures(void) {
	mtxMapFrontend.lock();
    std::vector<MapFeature> featuresSet;
    featuresSet.reserve(featuresMapFrontend.size());
    std::for_each(featuresMapFrontend.begin(),featuresMapFrontend.end(),
                    [&featuresSet](const std::map<int,MapFeature>::value_type& p)
                    { featuresSet.push_back(p.second); });
	mtxMapFrontend.unlock();
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
	return featuresSet;
}

/// Get feature position
Vec3 FeaturesMap::getFeaturePosition(unsigned int id) {
	mtxMapFrontend.lock();
    Vec3 feature(featuresMapFrontend[id].position);
	mtxMapFrontend.unlock();
	return feature;
}

/// get all visible features and reduce results
std::vector<MapFeature> FeaturesMap::getVisibleFeatures(
        const Mat34& cameraPose, int graphDepthThreshold, float_type distanceThreshold) {
    std::vector<int> neighborsIds;
    //we don't have to use graph since SE3 vertex have nly one following vertex (all vertices create camera trajectory)
    //poseGraph->findNearestNeighbors(camTrajectory.size()-1, graphDepthThreshold, neighborsIds);
    for (int i=camTrajectory.size()-1;i>=0;i--){
        if (i>=int(camTrajectory.size()-graphDepthThreshold)||graphDepthThreshold==-1)
            neighborsIds.push_back(i);
        else
            break;
    }
    distanceThreshold = pow(distanceThreshold,2.0);
    // Euclidean distance threshold
    for (std::vector<int>::iterator it = neighborsIds.begin();it!=neighborsIds.end();){
        Mat34 sensorPose = getSensorPose(*it);
        float_type dist = pow(cameraPose(0,3)-sensorPose(0,3),2.0) + pow(cameraPose(1,3)-sensorPose(1,3),2.0) + pow(cameraPose(2,3)-sensorPose(2,3),2.0);
        std::cout << "id: " << *it <<  " dist: " << dist << " thresh " << distanceThreshold << "\n";
        if (dist>distanceThreshold){
            std::cout << "erase\n";
            it = neighborsIds.erase(it);
        }
        else
            it++;
    }
    // Remove features which are not connnected to poses in neighborsIds
    std::set<int> featuresIds;
    // get ids of features observed from selected poses
    for (std::vector<int>::iterator it = neighborsIds.begin();it!=neighborsIds.end();it++){
        featuresIds.insert(camTrajectory[*it].featuresIds.begin(), camTrajectory[*it].featuresIds.end());
    }
    std::vector<MapFeature> visibleFeatures;
    mtxMapFrontend.lock();
    for (std::set<int>::iterator it=featuresIds.begin(); it!=featuresIds.end();it++) {
        Mat34 featurePos(featuresMapFrontend[*it].position);
        Mat34 featureCam = cameraPose.inverse() * featurePos;
        Eigen::Vector3d pointCam = sensorModel.inverseModel(featureCam(0, 3),
                featureCam(1, 3), featureCam(2, 3));
        //std::cout << pointCam(0) << " " << pointCam(1) << " " << pointCam(2) << "\n";
        if (pointCam(0) != -1) {
            visibleFeatures.push_back(featuresMapFrontend[*it]);
        }
    }
    mtxMapFrontend.unlock();
    return visibleFeatures;
}
/// get all visible features
std::vector<MapFeature> FeaturesMap::getVisibleFeatures(
		const Mat34& cameraPose) {
	std::vector<MapFeature> visibleFeatures;
	mtxMapFrontend.lock();
    for (std::map<int,MapFeature>::iterator it = featuresMapFrontend.begin();
			it != featuresMapFrontend.end(); it++) {
        Mat34 featurePos((it->second).position);
		Mat34 featureCam = cameraPose.inverse() * featurePos;
		Eigen::Vector3d pointCam = sensorModel.inverseModel(featureCam(0, 3),
				featureCam(1, 3), featureCam(2, 3));
        //std::cout << pointCam(0) << " " << pointCam(1) << " " << pointCam(2) << "\n";
		if (pointCam(0) != -1) {
            visibleFeatures.push_back(it->second);
		}
	}
	mtxMapFrontend.unlock();
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
	return visibleFeatures;
}

/// find nearest id of the image frame taking into acount the current angle of view and the view from the history
void FeaturesMap::findNearestFrame(const std::vector<MapFeature>& features, std::vector<int>& imageIds, std::vector<float_type>& angles, float_type maxAngle){
    Mat34 currentCameraPose = getSensorPose();
    imageIds.resize(features.size(),-1);
    angles.resize(features.size());
    for (size_t i = 0; i<features.size();i++){
            //compute position of feature in current camera pose
            Mat34 featureGlob(Vec3(features[i].position.x(), features[i].position.y(), features[i].position.z())*Quaternion(1,0,0,0));
            Mat34 featureInCamCurr = featureGlob.inverse()*currentCameraPose;
            Eigen::Vector3f featureViewCurr(featureInCamCurr(0,2), featureInCamCurr(1,2), featureInCamCurr(2,2));
            float_type minRot=10; int idMin=-1;
            //find the smallest angle between two views (max dot product)
            imageIds[i]=-1;
            for (size_t j=0; j<features[i].posesIds.size();j++){
                //compute position of feature in the camera pose
                Mat34 camPose = getSensorPose(features[i].posesIds[j]);
                Mat34 featureInCam = featureGlob.inverse()*camPose;
                Eigen::Vector3f featureView(featureInCam(0,2), featureInCam(1,2), featureInCam(2,2));
                float_type angle = acos(featureView.dot(featureViewCurr)/(featureView.norm()*featureViewCurr.norm()));
                if (fabs(angle)<minRot){
                    minRot = angle;
                    idMin = j;
                    angles[i] = fabs(angle);
                }
            }
            if (angles[i]>maxAngle)
                imageIds[i] =-1;
            else
                imageIds[i] = idMin;
    }
}

/// removes features which are too far from current camera pose (distant in graph)
void FeaturesMap::removeDistantFeatures(std::vector<MapFeature>& mapFeatures, int graphDepthThreshold, float_type distanceThreshold){
    std::vector<int> neighborsIds;
    //we don't have to use graph since SE3 vertex have nly one following vertex (all vertices create camera trajectory)
    //poseGraph->findNearestNeighbors(camTrajectory.size()-1, graphDepthThreshold, neighborsIds);
    for (int i=camTrajectory.size()-1;i>=0;i--){
        if (i>=int(camTrajectory.size()-graphDepthThreshold)||graphDepthThreshold==-1)
            neighborsIds.push_back(i);
        else
            break;
    }
    Mat34 currPose = getSensorPose();
    distanceThreshold = pow(distanceThreshold,2.0);
    // Euclidean distance threshold
    for (std::vector<int>::iterator it = neighborsIds.begin();it!=neighborsIds.end();){
        Mat34 sensorPose = getSensorPose(*it);
        float_type dist = pow(currPose(0,3)-sensorPose(0,3),2.0) + pow(currPose(1,3)-sensorPose(1,3),2.0) + pow(currPose(2,3)-sensorPose(2,3),2.0);
        if (dist>distanceThreshold){
            it = neighborsIds.erase(it);
        }
        else
            it++;
    }
    // Remove features which are not connnected to poses in neighborsIds
    for (std::vector<MapFeature>::iterator it = mapFeatures.begin(); it!=mapFeatures.end();){
        bool keep = false;
        for (std::vector<unsigned int>::iterator featurePoseIt = (*it).posesIds.begin(); featurePoseIt != (*it).posesIds.end(); featurePoseIt++){
            for (std::vector<int>::iterator poseIt = neighborsIds.begin();poseIt!=neighborsIds.end();poseIt++){
                if (*featurePoseIt == *poseIt){
                    keep=true;
                    break;
                }
            }
            if (keep) break;
        }
        if (keep)
            it++;
        else {
            it = mapFeatures.erase(it);
        }
    }
}

/// get pose of the sensor (default: last pose)
Mat34 FeaturesMap::getSensorPose(int poseId) {
    mtxCamTraj.lock();
    Mat34 pose;
    if (poseId < 0){
        poseId = camTrajectory.size() - 1;
    }
    if (poseId <= lastOptimizedPose){
        pose = camTrajectory[poseId].pose;
    }
    else {
        pose = camTrajectory[lastOptimizedPose].pose;
        for (int i = lastOptimizedPose + 1; i <= poseId; i++) {
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

/// start map management thread
void FeaturesMap::startMapManagerThread(int verbose){
    managementThr.reset(
            new std::thread(&FeaturesMap::manage, this, verbose));
}

/// Wait for optimization thread to finish
void FeaturesMap::finishOptimization(std::string trajectoryFilename,
		std::string graphFilename) {
	continueOpt = false;
    optimizationThr->join();
	poseGraph->export2RGBDSLAM(trajectoryFilename);
	poseGraph->save2file(graphFilename);
    if (config.exportMap){
        std::cout << "save map to file\n";
        plotFeatures(config.filenameMap,config.filenameData);
        std::cout << "save map to file end\n";
    }
}

/// Wait for map management thread to finish
void FeaturesMap::finishManagementThr(void){
    continueManagement = false;
    managementThr->join();
}

/// map management method
void FeaturesMap::manage(int verbose){
    // graph optimization
    continueManagement = true;

    // Wait for some information in map
    while (continueManagement && featuresMapManagement.size()==0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Wait for some information in map
    while (continueManagement) {
        auto start = std::chrono::system_clock::now();
        if (verbose>0){
            std::cout << "Graph management: start new iteration\n";
        }
        mtxMapManagement.lock();
        //compute Euclidean distance
        for (std::map<int,MapFeature>::iterator itFeature1 = featuresMapManagement.begin(); itFeature1!=featuresMapManagement.end(); itFeature1++){
            for (std::map<int,MapFeature>::iterator itFeature2 = featuresMapManagement.begin(); itFeature2!=featuresMapManagement.end(); itFeature2++){
                if (itFeature1->first!=itFeature2->first){
                    float_type dist = sqrt(pow(itFeature1->second.position.x()-itFeature2->second.position.x(),2.0) + pow(itFeature1->second.position.y()-itFeature2->second.position.y(),2.0) + pow(itFeature1->second.position.z()-itFeature2->second.position.z(),2.0));
                    if (dist<config.distThreshold)
                        std::cout << "features " << itFeature1->second.id << " and " << itFeature2->second.id << " are too close\n";
                }
            }
        }
        mtxMapManagement.unlock();
        if (verbose>0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
            std::cout << "Graph management finished (t = " << elapsed.count() << "ms)\n";
        }
    }
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
        bufferMapFrontend.mtxBuffer.lock(); // update frontend buffer
        for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
            bufferMapFrontend.features2update[it->id] = *it;
		bufferMapFrontend.mtxBuffer.unlock();
        bufferMapManagement.mtxBuffer.lock(); // update management buffer
        for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
            bufferMapManagement.features2update[it->id] = *it;
        bufferMapManagement.mtxBuffer.unlock();
		//try to update the map
		updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
        if (config.edges3DPrunningThreshold>0)
            ((PoseGraphG2O*) poseGraph)->prune3Dedges(config.edges3DPrunningThreshold);//pruning
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
    if (config.weakFeatureThr>0)
        ((PoseGraphG2O*)poseGraph)->removeWeakFeatures(config.weakFeatureThr);
    if (config.fixVertices)
        ((PoseGraphG2O*)poseGraph)->releaseFixedVertices();
    //poseGraph->optimize(-1, verbose, 0.0001);

    if (config.edges3DPrunningThreshold>0)
        ((PoseGraphG2O*) poseGraph)->prune3Dedges(config.edges3DPrunningThreshold);//pruning
    poseGraph->optimize(10, verbose);

	std::vector<MapFeature> optimizedFeatures;
	((PoseGraphG2O*) poseGraph)->getOptimizedFeatures(optimizedFeatures);
	bufferMapFrontend.mtxBuffer.lock();

    // update buffer frontend
    for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
        bufferMapFrontend.features2update[it->id] = *it;
    bufferMapFrontend.mtxBuffer.unlock();
    bufferMapManagement.mtxBuffer.lock(); // update management buffer
    for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
        bufferMapManagement.features2update[it->id] = *it;
    bufferMapManagement.mtxBuffer.unlock();

//    std::cout<<"features 2 update2 " << bufferMapFrontend.features2update.size() <<"\n";
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
	//update camera trajectory
	std::vector<VertexSE3> optimizedPoses;
	((PoseGraphG2O*) poseGraph)->getOptimizedPoses(optimizedPoses);
	updateCamTrajectory(optimizedPoses);
}

/// Update map
void FeaturesMap::updateMap(MapModifier& modifier,
        std::map<int,MapFeature>& featuresMap, std::recursive_mutex& mutex) {
	if (mutex.try_lock()) {    //try to lock graph
		modifier.mtxBuffer.lock();
		if (modifier.addFeatures()) {
            featuresMap.insert(modifier.features2add.begin(),
					modifier.features2add.end());
			modifier.features2add.clear();
		}
		if (modifier.updateFeatures()) {
            for (auto it =
					modifier.features2update.begin();
                    it != modifier.features2update.end(); it++) {
                updateFeature(featuresMap, it->second);
			}
            modifier.features2update.clear();
		}
		modifier.mtxBuffer.unlock();
		mutex.unlock();
	}
}

/// Update feature
void FeaturesMap::updateFeature(std::map<int,MapFeature>& featuresMap,
        MapFeature& newFeature) {
    featuresMap[newFeature.id].position = newFeature.position;
}

/// Update camera trajectory
void FeaturesMap::updateCamTrajectory(std::vector<VertexSE3>& poses2update) {
	for (std::vector<VertexSE3>::iterator it = poses2update.begin();
			it != poses2update.end(); it++) {
		updatePose(*it);
	}
}

/// Update pose
void FeaturesMap::updatePose(VertexSE3& newPose, bool updateGraph) {
	if (newPose.vertexId > lastOptimizedPose)
		lastOptimizedPose = newPose.vertexId;
	mtxCamTraj.lock();
	for (std::vector<VertexSE3>::iterator it = camTrajectory.begin();
			it != camTrajectory.end(); it++) {
		if (it->vertexId == newPose.vertexId) {
			it->pose = newPose.pose;
		}
	}
    if (updateGraph)
        poseGraph->updateVertex(newPose);
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
    for (auto it = featuresMapFrontend.begin();
			it != featuresMapFrontend.end(); it++) {
        file << "Feature " << it->second.id << " " << it->second.position.x() << " "
                << it->second.position.y() << " " << it->second.position.z() << " " << it->second.u
                << " " << it->second.v << "\n";
		file << "FeaturePoseIds";
        for (std::vector<unsigned int>::iterator iter = it->second.posesIds.begin();
                iter != it->second.posesIds.end(); iter++) {
			file << " " << *iter;
		}
		file << "\n";
        file << "FeatureExtendedDescriptors " << it->second.descriptors.size() << " ";
		for (std::vector<ExtendedDescriptor>::iterator iter =
                it->second.descriptors.begin(); iter != it->second.descriptors.end();
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

/// computes std and mean from float vector
void FeaturesMap::computeMeanStd(const std::vector<float_type>& v, float_type& mean, float_type& std, float_type& max){
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();
    max=-10;
    for (auto it=v.begin();it!=v.end();it++){
        if (*it>max)
            max=*it;
    }

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    std = std::sqrt(sq_sum / v.size() - mean * mean);
    if (std::isnan(std))
            std=0;
}

/// plot all features
void FeaturesMap::plotFeatures(std::string filenamePlot, std::string filenameData){
    std::ofstream file(filenamePlot);
    file << "close all;\nclear all;\nhold on;\n";
    //std::vector<float_type> meanX, meanY, meanZ;
    std::vector<float_type> meanDist, stdevDist;
    std::vector<float_type> maxDist;
    std::vector<float_type> measurementsNo;
    for (int i=FEATURES_START_ID;i<featureIdNo;i++){
        std::vector<Edge3D> features;
        Vec3 estimation;
        ((PoseGraphG2O*)poseGraph)->getMeasurements(i, features, estimation);
        file << "%feature no " << i << "\n";
        file << "%measured from frames ";
        MapFeature tmpFeature = featuresMapFrontend[i];
        for (auto it = tmpFeature.posesIds.begin(); it!=tmpFeature.posesIds.end(); it++){
            file << *it << ", ";
        }
        file << "\n";
        std::vector<float_type> dist4estim;
        measurementsNo.push_back(features.size());
        if (features.size()>0){ //how come?
            //std::cout << "dist:\n";
            float_type sumPos[3]={0,0,0};
            // compute the center of mass
            for (int j=0;j<features.size();j++){
                sumPos[0]+=features[j].trans.x(); sumPos[1]+=features[j].trans.y(); sumPos[2]+=features[j].trans.z();
            }
            estimation.x()=sumPos[0]/features.size(); estimation.y()=sumPos[1]/features.size(); estimation.z()=sumPos[2]/features.size();
            file << "plot3(" << estimation.x() << "," << estimation.y() << "," << estimation.z() << ",'ro');\n";
            for (int j=0;j<features.size();j++){
                float_type dist = sqrt(pow(features[j].trans.x()-estimation.x(),2.0)+pow(features[j].trans.y()-estimation.y(),2.0)+pow(features[j].trans.z()-estimation.z(),2.0));
                //std::cout << dist << ", ";
                dist4estim.push_back(dist);
                file << "plot3(" << features[j].trans.x() << "," << features[j].trans.y() << "," << features[j].trans.z() << ",'bx');\n";
            }
            float_type mean, std, max;
            computeMeanStd(dist4estim, mean, std, max); meanDist.push_back(mean); stdevDist.push_back(std); maxDist.push_back(max);
            if (max>0.2){
                std::cout << "dist:\n";
                for (auto it = dist4estim.begin(); it!=dist4estim.end();it++){
                    std::cout << *it << ", ";
                }
                std::cout << "\n";
                std::cout << "estimation: " << estimation.x() << " " << estimation.y() << " " << estimation.z() << "\n";
                for (int j=0;j<features.size();j++){
                    std::cout << "feature: " << features[j].fromVertexId << "->" <<features[j].toVertexId << " " << features[j].trans.x() << " " << features[j].trans.y() << " " << features[j].trans.z() << "\n";
                }
                getchar();
            }

            //std::cout << "\nmean:" << mean << " std: " << std << " max:" << max << "\n";
            //getchar();
            /*for (int j=0;j<features.size();j++){
                Mat33 unc = features[j].info.inverse();
                file << "C = [" << unc(0,0) << ", " << unc(0,1) << ", " << unc(0,2) << "; " << unc(1,0) << ", " << unc(1,1) << ", " << unc(1,2) << "; " << unc(2,0) << ", " << unc(2,1) << ", " << unc(2,2) << ", " << "];\n";
                file << "M = [" << features[j].trans.x() << "," << features[j].trans.y() << "," << features[j].trans.z() << "];\n";
                file << "error_ellipse(C, M);\n";
            }*/
        }
    }
    file.close();
    std::ofstream fileData(filenameData);
    fileData << "\nclear all;\n";
    float_type mean, std, max;
    computeMeanStd(measurementsNo, mean, std, max);
    fileData << "featuresNo = " << featureIdNo - FEATURES_START_ID << "\n";
    fileData << "meanMeasurementsNo = " << mean << "\n";
    fileData << "stdMeasurementsNo = " << std << "\n";
    fileData << "maxMeasurementsNo = " << max << "\n";
    int singleMeasurementsNo = 0;
    for (auto it = measurementsNo.begin(); it!= measurementsNo.end();it++){
        if (*it<2) singleMeasurementsNo++;
    }
    fileData << "featuresMeasuredLessThanOnce = " << (double(singleMeasurementsNo)/double(measurementsNo.size()))*100 << "%[%]\n";
    fileData << "measurementsNo = [";
    for (auto it = measurementsNo.begin(); it!=measurementsNo.end();it++)
            fileData << *it << ", ";
    fileData << "];\n plot(measurementsNo,'r'); ylabel('measurementsNo'); xlabel('featureNo');\n";
    fileData << "print -color -djpg measurementsNo.jpg\n";
    fileData << "meanDist = [";
    for (auto it = meanDist.begin(); it!=meanDist.end();it++)
        fileData << *it << ", ";
    fileData << "];\n figure(); plot(meanDist,'k'); ylabel('meanDist'); xlabel('featureNo');";
    fileData << "print -color -djpg meanDist.jpg\n";
    fileData << "stdevDist = [";
    for (auto it = stdevDist.begin(); it!=stdevDist.end();it++)
        fileData << *it << ", ";
    fileData << "];\n meanMeanDist = mean(meanDist)\n; meanStdDist = mean(stdevDist)\n";
    fileData << "figure(); plot(stdevDist,'g'); ylabel('stdevDist'); xlabel('featureNo'); \n";
    fileData << "maxDist = [";
    for (auto it = maxDist.begin(); it!=maxDist.end();it++)
        fileData << *it << ", ";
    fileData << "];\nmeanMaxDist=mean(maxDist)\n stdMaxDist = std(maxDist)\n";
    fileData << "figure(); plot(maxDist,'b');\n ylabel('maxDist'); xlabel('featureNo');";
    fileData << "print -color -djpg maxDist.jpg\n";
    fileData << "meanDist(meanDist<1e-5) = [];";
    fileData << "\nmeanDistWithoutZeros=mean(meanDist)\n stdMeanDistWithoutZeros = std(meanDist)\n";
    fileData << "maxDist(maxDist==0) = [];";
    fileData << "\nmeanMaxDistWithoutZeros=mean(maxDist)\n stdMaxDistWithoutZeros = std(maxDist)\n";
    fileData << "stdevDist(stdevDist==0) = [];";
    fileData << "\nmeanStdDistWithoutZeros=mean(stdevDist)\n stdStdDistWithoutZeros = std(stdevDist)";
    fileData.close();
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
