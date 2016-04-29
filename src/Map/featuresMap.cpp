/** @file featuresMap.cpp
 *
 * implementation - Elevation Map
 * \author Dominik Belter
 */

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
		featureIdNo(FEATURES_START_ID), lastOptimizedPose(0), Map("Features Map",MAP_FEATURES), lastKeyframeId(0), frames2marginalize(std::make_pair(0,0)) {
	poseGraph = createPoseGraphG2O();
}

/// Construction
FeaturesMap::FeaturesMap(std::string configMap, std::string sensorConfig) :
        config(configMap), featureIdNo(FEATURES_START_ID), sensorModel(sensorConfig), lastOptimizedPose(0),
        Map("Features Map",	MAP_FEATURES), lastKeyframeId(0), frames2marginalize(std::make_pair(0,0)) {
	poseGraph = createPoseGraphG2O();
    if (config.searchPairsTypeLC==0)
        localLC = createLoopClosureLocal(config.configFilenameLC);
    //else if (config.searchPairsTypeLC==1)
    //    localLC = createLoopClosureFABMAP(config.configFilenameLC);
	// set that map is currently empty
	emptyMap = true;
	loopClosureSuccess = false;
}


/// Destruction
FeaturesMap::~FeaturesMap(void) {
}

const std::string& FeaturesMap::getName() const {
	return name;
}

/// Add NEW features to the map
/// Position of features in relation to camera pose
void FeaturesMap::addFeatures(const std::vector<RGBDFeature>& features, int poseId) {
    Mat34 cameraPose = getSensorPose(poseId);
    mtxCamTraj.lock();
    int camTrajSize = camTrajectory.size();
	mtxCamTraj.unlock();
    if (poseId==-1) poseId = camTrajectory.size() - 1;
    std::vector<Edge> features2visualization;
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

        poseGraph->addVertexFeature(Vertex3D(featureIdNo, Vec3(featurePos(0, 3), featurePos(1, 3), featurePos(2, 3))));
        if ( config.optimizationErrorType == Config::OptimizationErrorType::EUCLIDEAN) {
            //std::cout<<"Edge 3D -- Euclidean error" << std::endl;
			Edge3D e((*it).position, info, camTrajSize - 1, featureIdNo);
			poseGraph->addEdge3D(e);

		    if (config.visualize)
		            features2visualization.push_back(e);
		}
        else if ( config.optimizationErrorType == Config::OptimizationErrorType::REPROJECTION) {
			//std::cout<<"Edge 3DReproj -- Reprojection error" << std::endl;
			Edge3DReproj e(it->u, it->v, Eigen::Matrix<float_type, 2, 2>::Identity(), camTrajSize - 1, featureIdNo);
			poseGraph->addEdge3DReproj(e);
		}
        else{
			std::cout<<"Wrong error chosen" << std::endl;
		}

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
int FeaturesMap::addNewPose(const Mat34& cameraPoseChange, float_type timestamp, cv::Mat image, cv::Mat depthImage) {
    int trajSize = camTrajectory.size();

    //add camera pose to the map
    if (config.keepCameraFrames){
        imageSeq.insert(std::make_pair(trajSize,image));
        depthSeq.insert(std::make_pair(trajSize,depthImage));
    }

    Mat34 cameraPose(cameraPoseChange);
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
        if (config.compressMap){
            covisibilityGraph.addVertex(0);
            camTrajectory[0].isKeyframe=true;//set keyframe
        }
    } else {
		odoMeasurements.push_back(cameraPoseChange);
        cameraPose = getSensorPose() * cameraPoseChange;
        VertexSE3 camPose(trajSize, cameraPose, timestamp);
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
    }
    if (config.visualize){
        if (config.frameNo2updatePointCloud>=0){
            if (trajSize%config.frameNo2updatePointCloud==0){
                this->notify(image, depthImage, trajSize);
            }
        }
        notify(bufferMapVisualization);
    }

    // TODO!!!! It shopuld be based on keyframes!
    if (continueLoopClosure && trajSize % 2 == 0){
        localLC->addPose(cameraPose,image, trajSize);
    }
	return trajSize;
}

/// get n-th image and depth image from the sequence
void FeaturesMap::getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage){
    image = imageSeq.at(poseNo);
    depthImage = depthSeq.at(poseNo);
    //check if poseNo is keyframe!!!
}

/// add measurements (features measured from the last camera pose)
void FeaturesMap::addMeasurements(const std::vector<MapFeature>& features, int poseId) {
	mtxCamTraj.lock();
	int camTrajSize = camTrajectory.size();
	mtxCamTraj.unlock();
    unsigned int _poseId = (poseId >= 0) ? poseId : (camTrajSize - 1);
    std::vector<Edge> features2visualization;
    for (std::vector<MapFeature>::const_iterator it = features.begin(); it != features.end(); it++) {
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
        featuresMapFrontend[it->id].lifeValue += 2 ;
        mtxMapFrontend.unlock();

        mtxMapLoopClosure.lock();
        featuresMapLoopClosure[it->id].posesIds.push_back(_poseId);
        featuresMapLoopClosure[it->id].imageCoordinates.insert(std::make_pair(_poseId,ImageFeature(it->u, it->v, it->position.z())));
        mtxMapLoopClosure.unlock();

        // !!!! TODO !!!!
        Edge3D e((*it).position, info, _poseId, (*it).id);
        poseGraph->addEdge3D(e);
        if (config.visualize)
            features2visualization.push_back(e);
    }
    if (config.compressMap){
        //std::cout << "posesNo : " << camTrajectory.size() << "\n";
        std::set<int> intersect;
        mtxCamTraj.lock();
        std::set_intersection(camTrajectory[lastKeyframeId].featuresIds.begin(),camTrajectory[lastKeyframeId].featuresIds.end(),camTrajectory.back().featuresIds.begin(),camTrajectory.back().featuresIds.end(),
                              std::inserter(intersect,intersect.begin()));
        double covisibility = double(intersect.size())/double(camTrajectory[lastKeyframeId].featuresIds.size());
        mtxCamTraj.unlock();
        //std::cout << "covisibility: " << covisibility*100.0 << "\n";
        if (covisibility<config.covisibilityKeyframes){
            std::vector<std::pair<int,double>> covisibilityKeyframes;
            //double maxCovisibility = computeCovisibility(features, covisibilityKeyframes);
            int previousKeyframe = lastKeyframeId;
            lastKeyframeId = camTrajectory.size()-1;
            covisibilityGraph.addVertex(lastKeyframeId);
            camTrajectory.back().isKeyframe=true;
            covisibilityGraph.addEdge(WeightedEdge(covisibility,std::make_pair(previousKeyframe, lastKeyframeId)));
            //check covisibility between previous keyframes
            for (int frameId = previousKeyframe-1;frameId>0;frameId--){
                if (camTrajectory[frameId].isKeyframe){
                    intersect.clear();
                    std::set_intersection(camTrajectory[lastKeyframeId].featuresIds.begin(),camTrajectory[lastKeyframeId].featuresIds.end(),camTrajectory[frameId].featuresIds.begin(),camTrajectory[frameId].featuresIds.end(),
                                          std::inserter(intersect,intersect.begin()));
                    covisibility = double(intersect.size())/double(camTrajectory[lastKeyframeId].featuresIds.size());
                    if ((covisibility<=config.marginalizationThr)&&(frames2marginalize.second<=frameId)){
                        frames2marginalize.second = frameId;
                    }
                    if (covisibility>0){
                        std::cout << "add edge between: " << lastKeyframeId << ", " << frameId << "\n";
                        std::cout << "covisibility " << covisibility << "\n";
                        covisibilityGraph.addEdge(WeightedEdge(covisibility,std::make_pair(lastKeyframeId, frameId)));
                    }
                    else
                        break;
                }
            }
        }
        // check max-without-compresion criterion
        //std::cout << "\n max no frames " << (camTrajectory.size()-frames2marginalize.first) << " -> " << config.maxFramesNo << "\n";
        if ((frames2marginalize.first==frames2marginalize.second)&&((camTrajectory.size()-frames2marginalize.first)>config.maxFramesNo)){
            frames2marginalize.second = camTrajectory.size()-int((config.minFramesNo+config.maxFramesNo)/2);
        }
    }
    if (config.visualize)
        notify(features2visualization);
}

/// compute covisibility between current frame and previous keyframes, returns max covisibility
double FeaturesMap::computeCovisibility(const std::vector<MapFeature>& features, std::vector<std::pair<int,double>>& covisibilityKeyframes) const{
    std::set<int> posesIds;
    std::set<int>featuresSet;
    for (auto feature : features){
        mtxMapFrontend.lock();
        posesIds.insert(featuresMapFrontend.at(feature.id).posesIds.begin(), featuresMapFrontend.at(feature.id).posesIds.end());
        mtxMapFrontend.unlock();
        featuresSet.insert(feature.id);
    }
    covisibilityKeyframes.clear();
    double maxCovisibility = std::numeric_limits<double>::min();
    for (auto poseId : posesIds){
        if (camTrajectory[poseId].isKeyframe){
            std::set<int> intersect;
            mtxCamTraj.lock();
            std::set_intersection(camTrajectory[poseId].featuresIds.begin(),camTrajectory[poseId].featuresIds.end(),featuresSet.begin(),featuresSet.end(),
                                  std::inserter(intersect,intersect.begin()));
            mtxCamTraj.unlock();
            double covisibility = double(intersect.size())/double(camTrajectory[poseId].featuresIds.size());
            covisibilityKeyframes.push_back(std::make_pair(poseId,covisibility));
            if (maxCovisibility<covisibility)
                maxCovisibility = covisibility;
        }
    }
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
		if (pointCam(0) != -1 && it->second.lifeValue > 0) {
            visibleFeatures.push_back(it->second);
            it->second.lifeValue -- ;
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
    managementThr.reset(new std::thread(&FeaturesMap::manage, this, verbose));
}

/// start loop closure thread
void FeaturesMap::startLoopClosureThread(int verbose, Matcher* matcher){
    if (!config.keepCameraFrames)
        throw std::runtime_error(std::string("Camera frames are not used (keepCameraFrames==false). LC is not available.\nModify config files.\n"));
    localLC->startLCsearchingThread();
    loopClosureThr.reset(new std::thread(&FeaturesMap::loopClosure, this, verbose, matcher));
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
        std::cout << "save map to file TEST\n";
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
   // usleep(config.waitUntilFinishedLC*1000000);
    continueLoopClosure = false;
    localLC->finishLCsearchingThr();
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
            for (std::map<int,MapFeature>::iterator itFeature2 = itFeature1; itFeature2!=featuresMapManagement.end(); itFeature2++){
                if (itFeature1->first!=itFeature2->first){
                    float_type dist = sqrt(pow(itFeature1->second.position.x()-itFeature2->second.position.x(),2.0) + pow(itFeature1->second.position.y()-itFeature2->second.position.y(),2.0) + pow(itFeature1->second.position.z()-itFeature2->second.position.z(),2.0));
                    if (dist<config.distThreshold)
                    {
                        std::cout << "features " << itFeature1->second.id << " and " << itFeature2->second.id << " are too close\n";
                    }
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
        std::cout << "wait\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Wait for some information in map
    auto start = std::chrono::system_clock::now();
    while (continueLoopClosure) {

        if (verbose>0){
            std::cout << "Loop closure: start new iteration\n";
        }
        std::pair<int,int> candidatePoses;
        if (localLC->getLCPair(candidatePoses)){
            std::vector<MapFeature> featureSetA, featureSetB;

			if (verbose > 0) {
				std::cout << "Loop closure: pair to analyze - " << candidatePoses.first << " "
						<< candidatePoses.second << std::endl;
			}

            //mtxCamTrajLC.lock();
            //if (camTrajectoryLC[candidatePoses.first].featuresIds.size()==0){//no measurements
            //    mtxCamTrajLC.unlock();
            //}
            Eigen::Matrix4f estimatedTransformation;
            std::vector<std::pair<int, int>> pairedFeatures;
            double matchingRatio;
            if (config.typeLC == 0){ // use rgb frame
                SensorFrame sensorFrames[2];
                // be careful: todo: lock image and depth
                sensorFrames[0].depthImageScale=sensorModel.config.depthImageScale;
                sensorFrames[1].depthImageScale=sensorModel.config.depthImageScale;
                getImages(candidatePoses.first, sensorFrames[0].rgbImage, sensorFrames[0].depthImage);
                getImages(candidatePoses.second, sensorFrames[1].rgbImage, sensorFrames[1].depthImage);
                matchingRatio = matcher->matchPose2Pose(sensorFrames, estimatedTransformation);



                loopClosureMatchingRatiosLog.push_back(matchingRatio);
                loopClosureAnalyzedPairsLog.push_back(candidatePoses);

               // std::cout << "Loop closure: matchingRatio: " << matchingRatio << ", between frames: " << candidatePoses.first << "->" << candidatePoses.second << "\n";
              //  std::cout << "Loop closure: paired features " << pairedFeatures.size() << "\n";

            }
            mtxCamTrajLC.lock();
            if (config.typeLC == 1){ // use map features
                if ((camTrajectoryLC[candidatePoses.first].featuresIds.size()>config.minNumberOfFeaturesLC)&&(camTrajectoryLC[candidatePoses.second].featuresIds.size()>config.minNumberOfFeaturesLC)){
                    Mat34 poseAinv = camTrajectoryLC[candidatePoses.first].pose.inverse();
                    Mat34 poseBinv = camTrajectoryLC[candidatePoses.second].pose.inverse();
                    for (auto & featureId : camTrajectoryLC[candidatePoses.first].featuresIds){
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
                     for (auto & featureId : camTrajectoryLC[candidatePoses.second].featuresIds){
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
                     mtxCamTrajLC.unlock();
                     std::vector<MapFeature> featureSet[2] = {featureSetA, featureSetA};
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
            }
            else{
                mtxCamTrajLC.unlock();
            }
            if (matchingRatio>config.matchingRatioThresholdLC){

            	if (verbose > 0) {

					std::cout << "Loop closure: matched: " << candidatePoses.first << ", " << candidatePoses.second << "\n";
					std::cout << "Loop closure: matchingRatio " << matchingRatio << "\n";
					std::cout << "Loop closure: features sets size(): " << featureSetA.size() << ", " << featureSetB.size() << "\n";
					std::cout << "Loop closure: estimated transformation: \n" << estimatedTransformation << "\n";
					std::cout << "Loop closure: graph transformation: \n" << (camTrajectoryLC[candidatePoses.first].pose.inverse()*camTrajectoryLC[candidatePoses.second].pose).matrix() << "\n";
            	}
                loopClosureSuccess = true;

                if (config.measurementTypeLC==0){//pose-pose
                    Mat34 trans(estimatedTransformation.cast<double>());
                    addMeasurement(candidatePoses.first, candidatePoses.second, trans);
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
                    addMeasurements(measuredFeatures,candidatePoses.second);
                }
            }
        }
        else{
            if (verbose>0)
                std::cout << "Loop closure: priority queue is empty\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    if (verbose>0) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
        std::cout << "Loop closure finished (t = " << elapsed.count() << "ms)\n";
    }
}

/// store camera frames
void FeaturesMap::setStoreImages(bool storeImages){
    config.keepCameraFrames = storeImages;
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
        int marginalizeToPose = frames2marginalize.second;
        //std::cout << "\n\n\n\n" << frames2marginalize.first << "->" << frames2marginalize.second << "\n";
        if (frames2marginalize.first!=marginalizeToPose&&(frames2marginalize.second<(camTrajectory.size()-config.minFramesNo)||((camTrajectory.size()-frames2marginalize.first)>config.maxFramesNo))){
            marginalizeMeasurements(frames2marginalize.first, frames2marginalize.second);
            //std::cout << "\nmarginalize " << frames2marginalize.first << "->" << frames2marginalize.second << "\n";
            //getchar();
            frames2marginalize.first = marginalizeToPose;
        }
        //std::cout << "features in Map: " << featuresMapFrontend.size() << "\n";
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

/// marginalize measurements between frames
void FeaturesMap::marginalizeMeasurements(int frameBegin, int frameEnd){
    std::vector<int> keyframes;
    for (int frameNo=frameBegin;frameNo<frameEnd;frameNo++){
        if (camTrajectory[frameNo].isKeyframe)
            keyframes.push_back(frameNo);
    }
    keyframes.push_back(frameEnd);
    //find features that should be removed from the map
    std::set<int> featuresKeyframes;
    std::set<int> featuresAll;
    std::set<int> features2remove;
    std::set<int> features2removeGraph;
    //std::set<int> features2removeGraph2;
    for (int frameNo=frameBegin;frameNo<frameEnd;frameNo++){
        featuresAll.insert(camTrajectory[frameNo].featuresIds.begin(), camTrajectory[frameNo].featuresIds.end());
        if (camTrajectory[frameNo].isKeyframe){
            featuresKeyframes.insert(camTrajectory[frameNo].featuresIds.begin(), camTrajectory[frameNo].featuresIds.end());
            if (continueLoopClosure){
                localLC->addPose(this->getSensorPose(frameNo),imageSeq.at(frameNo), frameNo);
            }
        }
        if (config.keepCameraFrames&&!camTrajectory[frameNo].isKeyframe){
            imageSeq.erase(frameNo);
            depthSeq.erase(frameNo);
        }
    }
    std::set<int> featuresFullOpt;
    for (int frameNo=frameEnd;frameNo<camTrajectory.size();frameNo++){
        featuresFullOpt.insert(camTrajectory[frameNo].featuresIds.begin(), camTrajectory[frameNo].featuresIds.end());
    }
    std::set_difference(featuresAll.begin(), featuresAll.end(), featuresKeyframes.begin(), featuresKeyframes.end(), std::inserter(features2remove, features2remove.end()));
    std::set_difference(features2remove.begin(), features2remove.end(), featuresFullOpt.begin(), featuresFullOpt.end(), std::inserter(features2remove, features2remove.end()));
    //std::set_difference(featuresAll.begin(), featuresAll.end(), camTrajectory[frameEnd].featuresIds.begin(), camTrajectory[frameEnd].featuresIds.end(), std::inserter(features2removeGraph, features2removeGraph.end()));
    std::set_difference(featuresAll.begin(), featuresAll.end(), featuresFullOpt.begin(), featuresFullOpt.end(), std::inserter(features2removeGraph, features2removeGraph.end()));
    std::set_difference(features2removeGraph.begin(), features2removeGraph.end(), featuresKeyframes.begin(), featuresKeyframes.end(), std::inserter(features2removeGraph, features2removeGraph.end()));
    //std::cout << "all " << featuresAll.size() << "\n";
    //std::cout << "graph " << features2removeGraph.size() << "\n\n\n\n\n\n\n\n\n\n\n\n";

    bufferMapFrontend.mtxBuffer.lock();
    bufferMapFrontend.removeIds.insert(bufferMapFrontend.removeIds.begin(),features2remove.begin(), features2remove.end());
    bufferMapFrontend.mtxBuffer.unlock();
    updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    if (continueManagement){
        bufferMapManagement.mtxBuffer.lock();
        bufferMapManagement.removeIds.insert(bufferMapManagement.removeIds.begin(),features2remove.begin(), features2remove.end());
        bufferMapManagement.mtxBuffer.unlock();
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
    }
//    bufferMapLoopClosure.mtxBuffer.lock();
//    bufferMapLoopClosure.removeIds.insert(bufferMapLoopClosure.removeIds.begin(),features2remove.begin(), features2remove.end());
//    bufferMapLoopClosure.mtxBuffer.unlock();
//    updateMap(bufferMapLoopClosure, featuresMapLoopClosure, mtxMapLoopClosure);
    poseGraph->marginalize(keyframes, features2removeGraph);
}

/// Update map
void FeaturesMap::updateMap(MapModifier& modifier, std::map<int,MapFeature>& featuresMap, std::recursive_mutex& mutex) {
	if (mutex.try_lock()) {    //try to lock graph
		modifier.mtxBuffer.lock();
		if (modifier.addFeatures()) {
            featuresMap.insert(modifier.features2add.begin(), modifier.features2add.end());
            modifier.features2add.clear();
		}
		if (modifier.updateFeatures()) {
            for (auto feature : modifier.features2update) {
                updateFeature(featuresMap, feature.second);
			}
            modifier.features2update.clear();
		}
        if (modifier.updateFeatures()) {
            for (auto feature : modifier.features2update) {
                updateFeature(featuresMap, feature.second);
            }
            modifier.features2update.clear();
        }
        if (modifier.removeFeatures()) {
            for (auto feature : modifier.removeIds) {
                featuresMap.erase(feature);
            }
            modifier.features2update.clear();
        }
		modifier.mtxBuffer.unlock();
		mutex.unlock();
	}
}

/// Update feature
void FeaturesMap::updateFeature(std::map<int,MapFeature>& featuresMap, MapFeature& newFeature) {
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

int FeaturesMap::getNumberOfFeatures() {
	mtxMapFrontend.lock();
	int val = featuresMapFrontend.size();
	mtxMapFrontend.unlock();
	return val;
}


std::vector<double> FeaturesMap::getLoopClosureMatchingRatiosLog() {
	return loopClosureMatchingRatiosLog;
}

std::vector<std::pair<int,int>> FeaturesMap::getLoopClosureAnalyzedPairsLog() {
	return loopClosureAnalyzedPairsLog;
}

bool FeaturesMap::getAndResetLoopClosureSuccesful() {
	if ( loopClosureSuccess ) {
		loopClosureSuccess = false;
		return true;
	}
	return false;
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
