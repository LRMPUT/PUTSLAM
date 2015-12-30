#include "../include/Map/featuresMap.h"
#include "../include/PoseGraph/graph.h"
#include "../include/Grabber/xtionGrabber.h"
#include "../include/TransformEst/g2oEst.h"
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
    if (poseId==-1) poseId = camTrajectory.size() - 1;

    std::vector<Edge3D> features2visualization;
	for (std::vector<RGBDFeature>::const_iterator it = features.begin();
            it != features.end(); it++) { // update the graph

        mtxCamTraj.lock();
        camTrajectory[poseId].featuresIds.insert(featureIdNo);
        mtxCamTraj.unlock();
        mtxCamTrajLC.lock();
        camTrajectoryLC[poseId].featuresIds.insert(featureIdNo);
        mtxCamTrajLC.unlock();

		//feature pose in the global frame
		Mat34 featurePos((*it).position);
		featurePos = cameraPose.matrix() * featurePos.matrix();

		// Add pose id
		std::vector<unsigned int> poseIds;
        poseIds.push_back(poseId);

        /// image coordinates: std::map<index_of_the_frame,<u,v>>
        std::map<unsigned int, ImageFeature> imageCoordinates;
        imageCoordinates.insert(std::make_pair(poseId,ImageFeature(it->u, it->v,it->position.z())));

        if ((*it).descriptors.size()<=0)
            std::cout << "descriptor\n";
		//add each feature to map structure...
		Vec3 featurePositionInGlobal(featurePos.translation());
        bufferMapFrontend.mtxBuffer.lock();
        bufferMapFrontend.features2add[featureIdNo] = MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
                        poseIds, (*it).descriptors, imageCoordinates);
        bufferMapFrontend.mtxBuffer.unlock();
        if (continueManagement){
            bufferMapManagement.mtxBuffer.lock();
            bufferMapManagement.features2add[featureIdNo] =
                    MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
                            poseIds, (*it).descriptors, imageCoordinates);
            bufferMapManagement.mtxBuffer.unlock();
        }
        if (continueLoopClosure){
            bufferMapLoopClosure.mtxBuffer.lock();
            bufferMapLoopClosure.features2add[featureIdNo] =
                    MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
                            poseIds, (*it).descriptors, imageCoordinates);
            if (bufferMapLoopClosure.features2add[featureIdNo].descriptors.size()<=0)
                std::cout << "descriptor lc\n";
            bufferMapLoopClosure.mtxBuffer.unlock();
        }
        if (config.visualize){
            bufferMapVisualization.mtxBuffer.lock();
            bufferMapVisualization.features2add[featureIdNo] =
                    MapFeature(featureIdNo, it->u, it->v, featurePositionInGlobal,
                            poseIds, (*it).descriptors, imageCoordinates);
            bufferMapVisualization.mtxBuffer.unlock();
        }

		//add measurement to the graph
        Mat33 info(Mat33::Identity());
        if (config.useUncertainty){
            if (config.uncertaintyModel==0){
                info = sensorModel.informationMatrixFromImageCoordinates(it->u, it->v, (*it).position.z());
            }
            else if (config.uncertaintyModel==1){
                info = sensorModel.uncertinatyFromNormal(it->normal).inverse();
            }
            else if (config.uncertaintyModel==2){
                info = sensorModel.uncertinatyFromRGBGradient(it->RGBgradient).inverse();
            }
        }

        Edge3D e((*it).position, info, camTrajSize - 1, featureIdNo);
		poseGraph->addVertexFeature(
				Vertex3D(featureIdNo,
						Vec3(featurePos(0, 3), featurePos(1, 3),
								featurePos(2, 3))));
		poseGraph->addEdge3D(e);
        if (config.visualize)
            features2visualization.push_back(e);
		featureIdNo++;
    }

    //try to update the map
    updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    if (continueManagement)
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
    if (continueLoopClosure)
        updateMap(bufferMapLoopClosure, featuresMapLoopClosure, mtxMapLoopClosure);

    emptyMap = false;
    if (config.visualize){
        notify(bufferMapVisualization);
        notify(features2visualization);
    }
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
        mtxCamTrajLC.lock();
        camTrajectoryLC.push_back(camPose);
        mtxCamTrajLC.unlock();

        if (config.visualize){
            bufferMapVisualization.mtxBuffer.unlock();
            bufferMapVisualization.poses2add.push_back(camPose);
            bufferMapVisualization.mtxBuffer.unlock();
        }

        //add camera pose to the graph
		poseGraph->addVertexPose(camPose);

    } else {
		odoMeasurements.push_back(cameraPoseChange);
		VertexSE3 camPose(trajSize,
                getSensorPose() * cameraPoseChange, timestamp);
        mtxCamTraj.lock();
		camTrajectory.push_back(camPose);
		mtxCamTraj.unlock();
        mtxCamTrajLC.lock();
        camTrajectoryLC.push_back(camPose);
        mtxCamTrajLC.unlock();

        if (config.visualize){
            bufferMapVisualization.mtxBuffer.unlock();
            bufferMapVisualization.poses2add.push_back(camPose);
            bufferMapVisualization.mtxBuffer.unlock();
        }

        //add camera pose to the graph
		poseGraph->addVertexPose(camPose);
        if (continueLoopClosure){
            bufferMapLoopClosure.mtxBuffer.lock();
            bufferMapLoopClosure.poses2add.push_back(camPose);
            bufferMapLoopClosure.mtxBuffer.unlock();
        }
    }
    if (config.visualize){
        if (config.frameNo2updatePointCloud>=0){
            if (trajSize%config.frameNo2updatePointCloud==0){
                this->notify(image, depthImage, trajSize);
            }
        }
        notify(bufferMapVisualization);
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
    std::vector<Edge3D> features2visualization;
	for (std::vector<MapFeature>::const_iterator it = features.begin();
			it != features.end(); it++) {

        mtxCamTraj.lock();
        camTrajectory[_poseId].featuresIds.insert(it->id);
        mtxCamTraj.unlock();
        mtxCamTrajLC.lock();
        camTrajectoryLC[_poseId].featuresIds.insert(it->id);
        mtxCamTrajLC.unlock();

        //add measurement
		Mat33 info(Mat33::Identity());

        if (config.useUncertainty){
            if (config.uncertaintyModel==0){
                info = sensorModel.informationMatrixFromImageCoordinates(it->u, it->v, (*it).position.z());
            }
            else if (config.uncertaintyModel==1){
                info = sensorModel.uncertinatyFromNormal(it->normal).inverse();
            }
            else if (config.uncertaintyModel==2){
                info = sensorModel.uncertinatyFromRGBGradient(it->RGBgradient).inverse();
            }
        }

        mtxMapFrontend.lock();
        featuresMapFrontend[it->id].posesIds.push_back(_poseId);
        featuresMapFrontend[it->id].imageCoordinates.insert(std::make_pair(_poseId,ImageFeature(it->u, it->v, it->position.z())));
        mtxMapFrontend.unlock();

        mtxMapLoopClosure.lock();
        featuresMapLoopClosure[it->id].posesIds.push_back(_poseId);
        featuresMapLoopClosure[it->id].imageCoordinates.insert(std::make_pair(_poseId,ImageFeature(it->u, it->v, it->position.z())));
        mtxMapLoopClosure.unlock();

        Edge3D e((*it).position, info, _poseId, (*it).id);
		poseGraph->addEdge3D(e);
        if (config.visualize)
            features2visualization.push_back(e);
    }
    if (config.visualize)
        notify(features2visualization);
}

/// add measurement between two poses
void FeaturesMap::addMeasurement(int poseFrom, int poseTo, Mat34 transformation){
    EdgeSE3 e(transformation, Mat66::Identity(), poseFrom, poseTo);
    poseGraph->addEdgeSE3(e);
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
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
    if (continueManagement)
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
    if (continueLoopClosure)
        updateMap(bufferMapLoopClosure, featuresMapLoopClosure, mtxMapLoopClosure);
	return featuresSet;
}

/// Get feature position
Vec3 FeaturesMap::getFeaturePosition(unsigned int id) const {
	mtxMapFrontend.lock();
    Vec3 feature = featuresMapFrontend.at(id).position;
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
       // std::cout << "id: " << *it <<  " dist: " << dist << " thresh " << distanceThreshold << "\n";
        if (dist>distanceThreshold){
         //   std::cout << "erase\n";
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
    if (continueManagement)
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
    if (continueLoopClosure)
        updateMap(bufferMapLoopClosure, featuresMapLoopClosure, mtxMapLoopClosure);
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
Mat34 FeaturesMap::getSensorPose(int poseId) const {
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

int FeaturesMap::getPoseCounter() {
	mtxCamTraj.lock();
	int size = camTrajectory.size();
	mtxCamTraj.unlock();
	return size;
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

/// start loop closure thread
void FeaturesMap::startLoopClosureThread(int verbose, Matcher* matcher){
    loopClosureThr.reset(
            new std::thread(&FeaturesMap::loopClosure, this, verbose, matcher));
}

/// Wait for optimization thread to finish
void FeaturesMap::finishOptimization(std::string trajectoryFilename,
		std::string graphFilename) {
	continueOpt = false;
    optimizationThr->join();
    exportOutput(trajectoryFilename, graphFilename);
}

/// Export graph and trajectory
void FeaturesMap::exportOutput(std::string trajectoryFilename,
        std::string graphFilename) {
    poseGraph->export2RGBDSLAM(trajectoryFilename);
    poseGraph->save2file(graphFilename);
    if (config.exportMap){
        std::cout << "save map to file\n";
        plotFeatures(config.filenameMap,config.filenameData);
        std::cout << "save map to file end\n";
    }
    if (config.exportDistribution){
        plotFeaturesOnImage(config.filenameFeatDistr, config.frameNo);
    }
}

/// Wait for map management thread to finish
void FeaturesMap::finishManagementThr(void){
    continueManagement = false;
    managementThr->join();
}

/// Wait for loop closure thread to finish
void FeaturesMap::finishLoopClosureThr(void){
    usleep(config.waitUntilFinishedLC*1000000);
    continueLoopClosure = false;
    loopClosureThr->join();
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

/// geometric loop closure method
void FeaturesMap::loopClosure(int verbose, Matcher* matcher){
    // graph optimization
    continueLoopClosure = true;
    // Wait for some information in map
    while (continueLoopClosure && featuresMapLoopClosure.size()==0) {
//        std::cout << "wait\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    int poseId=config.minFrameDist;
    // Wait for some information in map
    auto start = std::chrono::system_clock::now();
    while (continueLoopClosure) {
        if (verbose>0){
            std::cout << "Loop closure: start new iteration\n";
        }
        for (int i=poseId;i<camTrajectoryLC.size();i++){
            if (updateQueueLC(poseId))
                poseId++;
        }
        if (priorityQueueLC.size()>0){
            LCElement element = priorityQueueLC.top();
            std::vector<MapFeature> featureSetA, featureSetB;
            mtxCamTrajLC.lock();
            if (camTrajectoryLC[element.posesIds.second].featuresIds.size()==0){
                mtxCamTrajLC.unlock();
                //std::cout << "element.posesIds.second " << element.posesIds.second <<"\n";
               //std::cout << "(camTrajectoryLC[element.posesIds.second].featuresIds.size() zero\n";
               usleep(100000);
               //std::cout << camTrajectoryLC[element.posesIds.second].featuresIds.size() << "\n";
               if (camTrajectoryLC[element.posesIds.second].featuresIds.size()==0)
                   priorityQueueLC.pop();
            }
            else if ((camTrajectoryLC[element.posesIds.first].featuresIds.size()>config.minNumberOfFeaturesLC)&&(camTrajectoryLC[element.posesIds.second].featuresIds.size()>config.minNumberOfFeaturesLC)){
                if (!config.useImagesLC){
                    Mat34 poseAinv = camTrajectoryLC[element.posesIds.first].pose.inverse();
                    Mat34 poseBinv = camTrajectoryLC[element.posesIds.second].pose.inverse();
                    for (auto & featureId : camTrajectoryLC[element.posesIds.first].featuresIds){
                        mtxMapLoopClosure.lock();
                        featureSetA.push_back(featuresMapLoopClosure[featureId]);
                        mtxMapLoopClosure.unlock();
                        //set relative position
                        Mat34 featurePose(Mat34::Identity());
                        featurePose(0,3)=featureSetA.back().position.x();
                        featurePose(1,3)=featureSetA.back().position.y();
                        featurePose(2,3)=featureSetA.back().position.z();
                        featurePose=poseAinv*featurePose;
                        featureSetA.back().position = Vec3(featurePose(0,3),featurePose(1,3),featurePose(2,3));
                        //std::cout << "feature A " << featureId << " " << Vec3(featurePose(0,3),featurePose(1,3),featurePose(2,3)).vector().transpose() << "\n";
                    }
                    mtxCamTrajLC.unlock();
                    mtxCamTrajLC.lock();
                    for (auto & featureId : camTrajectoryLC[element.posesIds.second].featuresIds){
                        mtxMapLoopClosure.lock();
                        featureSetB.push_back(featuresMapLoopClosure[featureId]);
                        mtxMapLoopClosure.unlock();
                        //set relative position
                        Mat34 featurePose(Mat34::Identity());
                        featurePose(0,3)=featureSetB.back().position.x();
                        featurePose(1,3)=featureSetB.back().position.y();
                        featurePose(2,3)=featureSetB.back().position.z();
                        featurePose=poseBinv*featurePose;
                        featureSetB.back().position = Vec3(featurePose(0,3),featurePose(1,3),featurePose(2,3));
                        //std::cout << "feature B " << featureId << " " << Vec3(featurePose(0,3),featurePose(1,3),featurePose(2,3)).vector().transpose() << "\n";
                    }
                }
                mtxCamTrajLC.unlock();
                std::vector<MapFeature> featureSet[2] = {featureSetA, featureSetA};
                std::vector<std::pair<int, int>> pairedFeatures;
                Eigen::Matrix4f estimatedTransformation;
                double matchingRatio;
                if (!config.useImagesLC){
                    // find ids of the frames where features were observed
                    std::vector<int> frameIds[2];
                    frameIds[0].resize(featureSetA.size()); frameIds[1].resize(featureSetA.size());
                    for (int i=0;i<2;i++){
                        int featureNo=0;
                        for (auto& feature : featureSet[i]){
                            frameIds[i][featureNo] = feature.posesIds[0];
                            featureNo++;
                        }
                    }
                    matchingRatio = matcher->matchPose2Pose(featureSet, frameIds, pairedFeatures, estimatedTransformation);
                }
                else{
                    SensorFrame sensorFrames[2];
                    // be careful: todo: lock image and depth
                    sensorFrames[0].depthImageScale=sensorModel.config.depthImageScale;
                    sensorFrames[1].depthImageScale=sensorModel.config.depthImageScale;
                    getImages(element.posesIds.first, sensorFrames[0].rgbImage, sensorFrames[0].depthImage);
                    getImages(element.posesIds.second, sensorFrames[1].rgbImage, sensorFrames[1].depthImage);
                    matchingRatio = matcher->matchPose2Pose(sensorFrames, pairedFeatures, estimatedTransformation);
                    std::cout << "matchingRatio" << matchingRatio << "between frames: " << element.posesIds.first << "->" << element.posesIds.second << "\n";
                }
                if (matchingRatio>config.matchingRatioThresholdLC){
//                    std::cout << "matched: " << element.posesIds.first << ", " << element.posesIds.second << "\n";
//                    std::cout << "matchingRatio " << matchingRatio << "\n";
//                    std::cout << "featureSetA.size(): " << featureSetA.size() << ", " << featureSetB.size() << "\n";
//                    std::cout << "estimated transformation: \n" << estimatedTransformation << "\n";
//                    std::cout << "graph transformation: \n" << (camTrajectoryLC[element.posesIds.first].pose.inverse()*camTrajectoryLC[element.posesIds.second].pose).matrix() << "\n";
//                    std::cout << "priorityQueueLC.size " << priorityQueueLC.size() << "\n";
                    if (config.measurementTypeLC==0){//pose-pose
                        Mat34 trans(estimatedTransformation.cast<double>());
                        addMeasurement(element.posesIds.first, element.posesIds.second, trans);
                    }
                    else if (config.measurementTypeLC==1){//pose-features
                        std::vector<MapFeature> measuredFeatures;
                        for (auto& pairFeat : pairedFeatures){
                            for (auto featureB : featureSetB){
                                if (featureB.id == pairFeat.second){
                                    MapFeature featTmp = featureB;
                                    featTmp.id = pairFeat.first;
                                    measuredFeatures.push_back(featTmp);
                                }
                            }
                        }
//                        std::cout << "measurements no " << measuredFeatures.size() << " from pose " << element.posesIds.second << "\n";
                        addMeasurements(measuredFeatures,element.posesIds.second);
                    }
                }
                priorityQueueLC.pop();
            }
            else{
                mtxCamTrajLC.unlock();
                priorityQueueLC.pop();
            }
        }
        else{
//            std::cout << "wait LC\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    std::cout << "priorityQueueLC.size " << priorityQueueLC.size() << "\n";
    if (verbose>0) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
        std::cout << "Loop closure finished (t = " << elapsed.count() << "ms)\n";
    }
}

///update priority queue for the loop closure
bool FeaturesMap::updateQueueLC(int frameId){
    if (config.minFrameDist>=camTrajectoryLC.size())
        return false;
    else{
        for (int i=0;i<frameId-config.minFrameDist;i++){
            mtxCamTrajLC.lock();
            double dotprod = (double)(1.0-camTrajectoryLC[frameId].pose.matrix().block<3,1>(0,2).adjoint()*camTrajectoryLC[i].pose.matrix().block<3,1>(0,2))/2.0;
            Vec3 p1(camTrajectoryLC[i].pose(0,3),camTrajectoryLC[i].pose(1,3), camTrajectoryLC[i].pose(2,3));
            Vec3 p2(camTrajectoryLC[frameId].pose(0,3),camTrajectoryLC[frameId].pose(1,3), camTrajectoryLC[frameId].pose(2,3));
            double euclDist = sqrt(pow(p1.x()-p2.x(),2.0)+pow(p1.y()-p2.y(),2.0)+pow(p1.z()-p2.z(),2.0));
            LCElement element;
            element.distance = dotprod*euclDist;
            if ((element.distance<config.distThresholdLC)&&(acos(1-dotprod*2)<config.rotThresholdLC)){
                //std::cout << "add to queque " << i << "->" << frameId << "\n";
                element.posesIds = std::make_pair(i,frameId);
                priorityQueueLC.push(element);
            }
            mtxCamTrajLC.unlock();
        }
        return true;
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
        if (continueManagement){
            bufferMapManagement.mtxBuffer.lock(); // update management buffer
            for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
                bufferMapManagement.features2update[it->id] = *it;
            bufferMapManagement.mtxBuffer.unlock();
        }
        if (config.visualize){
            bufferMapVisualization.mtxBuffer.lock(); // update management buffer
            for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
                bufferMapVisualization.features2update[it->id] = *it;
            bufferMapVisualization.mtxBuffer.unlock();
        }
        if (continueLoopClosure){
            bufferMapLoopClosure.mtxBuffer.lock(); // update management buffer
            for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
                bufferMapLoopClosure.features2update[it->id] = *it;
            bufferMapLoopClosure.mtxBuffer.unlock();
        }

		//try to update the map
		updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
        if (continueManagement)
            updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
        if (continueLoopClosure)
            updateMap(bufferMapLoopClosure, featuresMapLoopClosure, mtxMapLoopClosure);
        if (config.edges3DPrunningThreshold>0)
            ((PoseGraphG2O*) poseGraph)->prune3Dedges(config.edges3DPrunningThreshold);//pruning
		//update camera trajectory
		std::vector<VertexSE3> optimizedPoses;
		((PoseGraphG2O*) poseGraph)->getOptimizedPoses(optimizedPoses);
		updateCamTrajectory(optimizedPoses);

        if (config.visualize){
            bufferMapVisualization.mtxBuffer.lock(); // update visualization buffer
            for (auto it = optimizedPoses.begin(); it!=optimizedPoses.end();it++)
                bufferMapVisualization.poses2update.push_back(*it);
            bufferMapVisualization.mtxBuffer.unlock();
        }
        /*if (continueLoopClosure){
            bufferMapLoopClosure.mtxBuffer.lock(); // update visualization buffer
            for (auto it = optimizedPoses.begin(); it!=optimizedPoses.end();it++)
                bufferMapLoopClosure.poses2update.push_back(*it);
            bufferMapLoopClosure.mtxBuffer.unlock();
        }*/

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

    bufferMapFrontend.mtxBuffer.lock();// update buffer frontend
    for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
        bufferMapFrontend.features2update[it->id] = *it;
    bufferMapFrontend.mtxBuffer.unlock();
    if (continueManagement){
        bufferMapManagement.mtxBuffer.lock(); // update management buffer
        for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
            bufferMapManagement.features2update[it->id] = *it;
        bufferMapManagement.mtxBuffer.unlock();
    }
    if (continueLoopClosure){
        bufferMapLoopClosure.mtxBuffer.lock(); // update LC buffer
        for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
            bufferMapLoopClosure.features2update[it->id] = *it;
        bufferMapLoopClosure.mtxBuffer.unlock();
    }
    if (config.visualize){
        bufferMapVisualization.mtxBuffer.lock(); // update visualization buffer
        for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
            bufferMapVisualization.features2update[it->id] = *it;
        bufferMapVisualization.mtxBuffer.unlock();
    }

//    std::cout<<"features 2 update2 " << bufferMapFrontend.features2update.size() <<"\n";
	//try to update the map
	updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    if (continueManagement)
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
    if (continueLoopClosure)
        updateMap(bufferMapLoopClosure, featuresMapLoopClosure, mtxMapLoopClosure);
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
    //update cam traj LC
    mtxCamTrajLC.lock();
    for (std::vector<VertexSE3>::iterator it = camTrajectoryLC.begin();
            it != camTrajectoryLC.end(); it++) {
        if (it->vertexId == newPose.vertexId) {
            it->pose = newPose.pose;
        }
    }
    mtxCamTrajLC.unlock();
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
        /*for (std::vector<ExtendedDescriptor>::iterator iter =
                it->second.descriptors.begin(); iter != it->second.descriptors.end();
                iter++) {
			file << iter->poseId << " " << iter->descriptor.cols << " "
                    << iter->descriptor.rows;
            for (int i = 0; i < iter->descriptor.cols; i++)
                for (int j = 0; j < iter->descriptor.rows; j++) {
                    file << " " << iter->descriptor.at<double>(i, j);
                }
			file << "\n";
        }*/
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

/// plot all features on the i-th image
void FeaturesMap::plotFeaturesOnImage(std::string filename, unsigned int frameId){
    std::ofstream file(filename);
    file << "close all;\nclear all;\nhold on;\n";
    file << "I=imread('rgb_00000.png');\n";
    file << "imshow(I);\n";
    Mat34 camPose = getSensorPose(frameId);
    //std::cout << "camPose\n" << camPose.matrix() << "\n";
    std::default_random_engine generator(time(NULL));
    std::uniform_real_distribution<double> distribution(0,1);
    for (int i=FEATURES_START_ID;i<featureIdNo;i++){
        MapFeature tmpFeature = featuresMapFrontend[i];
        // for each frame position
        file << "%feature id: " << i << "\n";
        if (tmpFeature.posesIds.size()>10){
            double red = distribution(generator); double green = distribution(generator); double blue = distribution(generator);
            for (auto it = tmpFeature.posesIds.begin(); it!=tmpFeature.posesIds.end(); it++){
                Mat34 currCamPose = getSensorPose(*it);
                //std::cout << "CurrCamPose\n" << currCamPose.matrix() << "\n";
                Eigen::Vector3d point;
                sensorModel.getPoint(tmpFeature.imageCoordinates[*it].u, tmpFeature.imageCoordinates[*it].v, tmpFeature.imageCoordinates[*it].depth, point);
                //compute point coordinates in camera frame
                Mat34 point3d(Quaternion(1,0,0,0)*Vec3(point.x(), point.y(), point.z()));
                Mat34 cam2cam = (camPose.inverse()*currCamPose)*point3d;
                //std::cout << "cam2cam\n" << cam2cam.matrix() << "\n";
                Eigen::Vector3d pointUV = sensorModel.inverseModel(cam2cam(0,3),cam2cam(1,3),cam2cam(2,3));
                if (pointUV(0)!=-1){
                    file << "plot(" << pointUV(0) << ", " << pointUV(1) << ",'o','MarkerEdgeColor', [" << red << ", " << green << ", " << blue
                    << "],'MarkerFaceColor',[" << red << ", " << green << ", " << blue << "]);\n";
                }
            }
        }
    }
    file << "%imwrite (I, \"my_output_image.img\");\n";
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
            file << *it << "(" << tmpFeature.imageCoordinates[*it].u << "," << tmpFeature.imageCoordinates[*it].v << "), ";
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
//            if (max>0.2){
//                std::cout << "dist:\n";
//                for (auto it = dist4estim.begin(); it!=dist4estim.end();it++){
//                    std::cout << *it << ", ";
//                }
//                std::cout << "\n";
//                std::cout << "estimation: " << estimation.x() << " " << estimation.y() << " " << estimation.z() << "\n";
//                for (int j=0;j<features.size();j++){
//                    std::cout << "feature: " << features[j].fromVertexId << "->" <<features[j].toVertexId << " " << features[j].trans.x() << " " << features[j].trans.y() << " " << features[j].trans.z() << "\n";
//                }
//              //  getchar();
//            }

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
    ///move along trajectory and compute projection of features
    std::vector<float_type> meanDistU; std::vector<float_type> meanDistV;
    std::vector<float_type> maxDistU; std::vector<float_type> maxDistV;
    int iterFrame=0;
    for (auto it = camTrajectory.begin(); it!=camTrajectory.end();it++){
        float_type meanU = 0; float_type meanV = 0;
        float_type maxU = 0; float_type maxV = 0;
        int featuresNo = 0;
        for (auto itFeat=camTrajectory[iterFrame].featuresIds.begin();itFeat!=camTrajectory[iterFrame].featuresIds.end();itFeat++){
            std::vector<Edge3D> features;
            Vec3 estimation;
            ((PoseGraphG2O*)poseGraph)->getMeasurements(*itFeat, features, estimation);
            // compute the center of mass
            float_type sumPos[3]={0,0,0};
            for (int j=0;j<features.size();j++){
                sumPos[0]+=features[j].trans.x(); sumPos[1]+=features[j].trans.y(); sumPos[2]+=features[j].trans.z();
            }
            estimation.x()=sumPos[0]/features.size(); estimation.y()=sumPos[1]/features.size(); estimation.z()=sumPos[2]/features.size();
            //estimation pose in camera frame
            Mat34 feature(Quaternion(1,0,0,0)*estimation);
            Mat34 featInCam = camTrajectory[iterFrame].pose.inverse()*feature;
            Eigen::Vector3d featCam = sensorModel.inverseModel(featInCam(0,3), featInCam(1,3), featInCam(2,3));
            float_type meanFeatU = 0; float_type meanFeatV = 0;
            float_type maxFeatU = -10; float_type maxFeatV = -10;
            if (featCam(0)>0&&featCam(1)>1){/// for all measurements of the feature
                int measNo=0;
                for (int j=0;j<features.size();j++){
                    Mat34 featureLocal(Quaternion(1,0,0,0)*features[j].trans);
                    Mat34 featLocalInCam = camTrajectory[iterFrame].pose.inverse()*featureLocal;
                    Eigen::Vector3d featLocalCam = sensorModel.inverseModel(featLocalInCam(0,3), featLocalInCam(1,3), featLocalInCam(2,3));
                    if (featLocalCam(0)>0&&featLocalCam(1)>1){
                        float_type distU = fabs(featLocalCam(0)-featCam(0));
                        float_type distV = fabs(featLocalCam(1)-featCam(1));
                        meanFeatU+=distU; meanFeatV+=distV;
                        if (distU>maxFeatU) maxFeatU = distU;
                        if (distV>maxFeatV) maxFeatV = distV;
                        measNo++;
                    }
                }
                meanFeatU/=double(measNo); meanFeatV/=double(measNo);
            }
            meanU += meanFeatU; meanV += meanFeatV;
            featuresNo++;
            if (maxFeatU>maxU) maxU = maxFeatU;
            if (maxFeatV>maxV) maxV = maxFeatV;
        }
        meanU/=double(featuresNo); meanV/=double(featuresNo);
        meanDistU.push_back(meanU); meanDistV.push_back(meanV);
        maxDistU.push_back(maxU); maxDistV.push_back(maxV);
        iterFrame++;
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
    fileData << "maxDistU = [";
    for (auto it = maxDistU.begin(); it!=maxDistU.end();it++)
        fileData << *it << ", ";
    fileData << "];\n figure(); plot(maxDistU,'k'); ylabel('maxDistU'); xlabel('frameNo');";
    fileData << "print -color -djpg maxDistU.jpg\n";
    fileData << "maxDistV = [";
    for (auto it = maxDistV.begin(); it!=maxDistV.end();it++)
        fileData << *it << ", ";
    fileData << "];\n figure(); plot(maxDistV,'k'); ylabel('maxDistV'); xlabel('frameNo');";
    fileData << "print -color -djpg maxDistV.jpg\n";
    fileData << "meanDistU = [";
    for (auto it = meanDistU.begin(); it!=meanDistU.end();it++)
        fileData << *it << ", ";
    fileData << "];\n figure(); plot(meanDistU,'k'); ylabel('meanDistU'); xlabel('frameNo');";
    fileData << "print -color -djpg meanDistU.jpg\n";
    fileData << "meanDistV = [";
    for (auto it = meanDistV.begin(); it!=meanDistV.end();it++)
        fileData << *it << ", ";
    fileData << "];\n figure(); plot(meanDistV,'k'); ylabel('meanDistV'); xlabel('frameNo');";
    fileData << "print -color -djpg meanDistV.jpg\n";
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

/// set drawing options
void FeaturesMap::setDrawOptions(bool _draw){
    config.visualize = _draw;
}

/// use uncertainty
bool FeaturesMap::useUncertainty(void){
    return config.useUncertainty;
}

/// disable Robust Kernel
void FeaturesMap::disableRobustKernel(void) {
	((PoseGraphG2O*) poseGraph)->disableRobustKernel();
}

/// get uncertainty of the pose
Mat66 FeaturesMap::getPoseUncertainty(unsigned int id) const{
    Mat66 incCov = ((PoseGraphG2O*) poseGraph)->getPoseIncrementCovariance(id);
    Mat66 unc = G2OEst::computeCovarianceMatrix(incCov, getSensorPose(id).inverse());
    return unc;
}

/// get uncertainty of the feature
Mat33 FeaturesMap::getFeatureUncertainty(unsigned int id) const{
    Mat33 incCov = ((PoseGraphG2O*) poseGraph)->getFeatureIncrementCovariance(id);
    Mat33 unc = G2OEst::computeCovarianceMatrix(incCov, getFeaturePosition(id).inverse());
    return unc;
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
