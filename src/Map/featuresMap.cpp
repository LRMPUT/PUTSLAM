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
		Map("Features Map", MAP_FEATURES), lastKeyframeId(0), activeKeyframesNo(
				0), lastFullyMarginalizedFrame(-1), frames2marginalize(
				std::make_pair(0, 0)), featureIdNo(
		FEATURES_START_ID), lastOptimizedPose(0) {

	poseGraph = createPoseGraphG2O();
    continueLoopClosure = false;
}

/// Construction
FeaturesMap::FeaturesMap(std::string configMap, std::string sensorConfig) :
		Map("Features Map", MAP_FEATURES), config(configMap), lastKeyframeId(0), activeKeyframesNo(
				0), lastFullyMarginalizedFrame(-1), frames2marginalize(
				std::make_pair(0, 0)), sensorModel(sensorConfig), featureIdNo(
				FEATURES_START_ID), lastOptimizedPose(0) {

	std::cout << "FeaturesMap" << std::endl;

	poseGraph = createPoseGraphG2O();
	if (config.searchPairsTypeLC == 0)
		localLC = createLoopClosureLocal(config.configFilenameLC);
	//else if (config.searchPairsTypeLC==1)
	//    localLC = createLoopClosureFABMAP(config.configFilenameLC);
	// set that map is currently empty
	emptyMap = true;
	loopClosureSuccess = false;
	updateMapSuccess = true;
    continueLoopClosure = false;
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
    int camTrajSize = getPoseCounter();
    if (poseId==-1) poseId = camTrajSize - 1;
    std::vector<Edge> features2visualization;
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
			//Edge3D e((*it).position, info, camTrajSize - 1, featureIdNo);
        	Edge3D e((*it).position, info, poseId, featureIdNo);
        	poseGraph->addEdge3D(e);

		    if (config.visualize)
		            features2visualization.push_back(e);
		}
        else if ( config.optimizationErrorType == Config::OptimizationErrorType::REPROJECTION) {
			//std::cout<<"Edge 3DReproj -- Reprojection error" << std::endl;
            //Edge3DReproj e(it->u, it->v, Eigen::Matrix<double, 2, 2>::Identity(), camTrajSize - 1, featureIdNo);
            Edge3DReproj e(it->u, it->v, Eigen::Matrix<double, 2, 2>::Identity(), poseId, featureIdNo);
        	poseGraph->addEdge3DReproj(e);
		}
        else{
			std::cout<<"Wrong error chosen" << std::endl;
		}

		featureIdNo++;
    }


    //try to update the map
    updateMaps();

    emptyMap = false;
    if (config.visualize){
        notify(bufferMapVisualization);
        notify(features2visualization);
    }
}

/// add new pose of the camera, returns id of the new pose
int FeaturesMap::addNewPose(const Mat34& cameraPoseChange, double timestamp, cv::Mat image, cv::Mat depthImage) {
    int trajSize = getPoseCounter();

    // When keepCameraFrames:
    // - true 	- we store all images in map
    // - false 	- we store only the images from new pose for LC purposes
    if (!config.keepCameraFrames){
        imageSeq.clear();
        depthSeq.clear();
    }
    imageSeq.insert(std::make_pair(trajSize,image));
    depthSeq.insert(std::make_pair(trajSize,depthImage));

    Mat34 cameraPose(cameraPoseChange);
	if (trajSize == 0) {
		odoMeasurements.push_back(Mat34::Identity());
		VertexSE3 camPose(trajSize, cameraPoseChange, timestamp);

        mtxCamTraj.lock();
		camTrajectory.push_back(camPose);
        mtxCamTraj.unlock();

        if (config.visualize){
            bufferMapVisualization.mtxBuffer.unlock();
            bufferMapVisualization.poses2add.push_back(camPose);
            bufferMapVisualization.mtxBuffer.unlock();
        }


        //add camera pose to the graph
        poseGraph->addVertexPose(camPose);
        covisibilityGraph.addVertex(0);
        mtxCamTraj.lock();
        camTrajectory[0].isKeyframe=true;//set keyframe
        mtxCamTraj.unlock();
    } else {
		odoMeasurements.push_back(cameraPoseChange);
        cameraPose = getSensorPose() * cameraPoseChange;
        VertexSE3 camPose(trajSize, cameraPose, timestamp);
        mtxCamTraj.lock();
		camTrajectory.push_back(camPose);
        mtxCamTraj.unlock();

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

	return trajSize;
}

void FeaturesMap::removeFeatures(std::vector<int> featureIdsToRemove) {
	// TODO: DB

	// Glowne zastosowanie to usuniecie cech po ich zlaczeniu wynikajacym z loop closure
	// Istotne, aby usunac cechy o podanych id
	// Według mnie nalezy tez przeprowadzic aktualizacje dla wszystkich podzbiorow cech oraz optymlizacji, a nie tylko cech dla loop closure
    bufferMapFrontend.mtxBuffer.lock();
    bufferMapFrontend.removeIds.insert(bufferMapFrontend.removeIds.begin(),featureIdsToRemove.begin(), featureIdsToRemove.end());
    bufferMapFrontend.mtxBuffer.unlock();
    bufferMapManagement.mtxBuffer.lock();
    bufferMapManagement.removeIds.insert(bufferMapManagement.removeIds.begin(),featureIdsToRemove.begin(), featureIdsToRemove.end());
    bufferMapManagement.mtxBuffer.unlock();
    bufferMapVisualization.mtxBuffer.lock();
    bufferMapVisualization.removeIds.insert(bufferMapVisualization.removeIds.begin(),featureIdsToRemove.begin(), featureIdsToRemove.end());
    bufferMapVisualization.mtxBuffer.unlock();
    poseGraph->setFeatures2remove(std::set<int>(featureIdsToRemove.begin(),featureIdsToRemove.end()));
    updateMaps();
}

/// get n-th image and depth image from the sequence
void FeaturesMap::getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage){
	if (config.keepCameraFrames){
	  image = imageSeq.at(poseNo);
	  depthImage = depthSeq.at(poseNo);
	}
	//check if poseNo is keyframe!!!
}

/// add measurements (features measured from the last camera pose)
void FeaturesMap::addMeasurements(const std::vector<MapFeature>& features, int poseId) {
    int camTrajSize = getPoseCounter();
    unsigned int _poseId = (poseId >= 0) ? poseId : (camTrajSize - 1);
    std::vector<Edge> features2visualization;
    for (std::vector<MapFeature>::const_iterator it = features.begin(); it != features.end(); it++) {
        mtxCamTraj.lock();
        camTrajectory[_poseId].featuresIds.insert(it->id);
        mtxCamTraj.unlock();

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


        bufferMapFrontend.mtxBuffer.lock();
        bufferMapFrontend.measurements2update[it->id] = std::make_pair(_poseId,*it);
        bufferMapFrontend.mtxBuffer.unlock();

        Edge3D e((*it).position, info, _poseId, (*it).id);
        poseGraph->addEdge3D(e);
        if (config.visualize)
            features2visualization.push_back(e);
    }

    // TODO: Czy to nie powinno byc updateMap dla takze cech z frontendu?
    // TODO: Czy teraz w ogole dodatkowe deskryptory sa dodawanie dla danej cechy? Nie sa
    // TODO: To w ogole moze duplikowac wpisy powyzej
    updateMaps();

    ///keyframes management
    std::set<int> intersect;
    mtxCamTraj.lock();
    std::set_intersection(camTrajectory[lastKeyframeId].featuresIds.begin(),camTrajectory[lastKeyframeId].featuresIds.end(),camTrajectory.back().featuresIds.begin(),camTrajectory.back().featuresIds.end(),
                          std::inserter(intersect,intersect.begin()));
    double covisibility = double(intersect.size())/double(camTrajectory[lastKeyframeId].featuresIds.size());
    mtxCamTraj.unlock();
    //std::cout << "covisibility: " << covisibility*100.0 << "\n";
    if (covisibility<config.covisibilityKeyframes){
        std::vector<std::pair<int,double>> covisibilityKeyframes;
        double maxCovisibility = computeCovisibility(features, covisibilityKeyframes);
        if (maxCovisibility<config.covisibilityKeyframes){
            mtxCamTraj.lock();
            camTrajectory.back().isKeyframe=true;
            activeKeyframesNo++;
            lastKeyframeId = (int) camTrajectory.size()-1;
            //std::cout << "\n lastKeyframeId " << lastKeyframeId << "\n";
            mtxCamTraj.unlock();

            if (continueLoopClosure){
                localLC->addPose(this->getSensorPose(lastKeyframeId),imageSeq.at(lastKeyframeId), lastKeyframeId);
            }
            covisibilityGraph.addVertex(lastKeyframeId);
            for (auto covKey : covisibilityKeyframes){
                //std::cout << "covisibility between " << covKey.first << "->" << lastKeyframeId << " weight " << covKey.second << "\n";
                covisibilityGraph.addEdge(WeightedEdge(covKey.second,std::make_pair(lastKeyframeId, covKey.first)));
                if (config.compressMap){
                    if ((covKey.second<=config.marginalizationThr)&&(frames2marginalize.second<=covKey.first)){
                        frames2marginalize.second = covKey.first;
                    }
                }
            }
        }
        if (config.compressMap){
            // check max-without-compresion criterion
            mtxCamTraj.lock();
            if ((frames2marginalize.first==frames2marginalize.second)&&(((int)camTrajectory.size()-frames2marginalize.first)>config.maxFramesNo)){
                frames2marginalize.second = (int) camTrajectory.size()-int((config.minFramesNo+config.maxFramesNo)/2);
            }
            mtxCamTraj.unlock();
        }
    }
    if (config.visualize)
        notify(features2visualization);
}

/// update maps (frontend, loop closure, management)
void FeaturesMap::updateMaps(void){
    updateMap(bufferMapFrontend, featuresMapFrontend, mtxMapFrontend);
    if (continueManagement)
        updateMap(bufferMapManagement, featuresMapManagement, mtxMapManagement);
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
        mtxCamTraj.lock();
        if (camTrajectory[poseId].isKeyframe){
            std::set<int> intersect;
            std::set_intersection(camTrajectory[poseId].featuresIds.begin(),camTrajectory[poseId].featuresIds.end(),featuresSet.begin(),featuresSet.end(),
                                  std::inserter(intersect,intersect.begin()));
            double covisibility = double(intersect.size())/double(camTrajectory[poseId].featuresIds.size());
            mtxCamTraj.unlock();
            if (covisibility>0){
                covisibilityKeyframes.push_back(std::make_pair(poseId,covisibility));
                if (covisibility>maxCovisibility)
                    maxCovisibility = covisibility;
            }
        }
        else
            mtxCamTraj.unlock();
    }
    return maxCovisibility;
}

/// add measurement between two poses
void FeaturesMap::addMeasurement(int poseFrom, int poseTo, Mat34 transformation){
    EdgeSE3 e(transformation, Mat66::Identity(), poseFrom, poseTo);
    poseGraph->addEdgeSE3(e);
    //std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
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

	// TODO: Czy takie operacje powinny miec miejsce w watku frontendu?
	//try to update the map
    updateMaps();
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
        const Mat34& cameraPose, int graphDepthThreshold, double distanceThreshold) {
    std::vector<int> neighborsIds;
    //we don't have to use graph since SE3 vertex have only one following vertex (all vertices create camera trajectory)
    //poseGraph->findNearestNeighbors(camTrajectory.size()-1, graphDepthThreshold, neighborsIds);
    int trajSize = getPoseCounter();
    for (int i=(int)trajSize-1;i>=0;i--){
        if (i>=int(trajSize-graphDepthThreshold)||graphDepthThreshold==-1)
            neighborsIds.push_back(i);
        else
            break;
    }
    distanceThreshold = pow(distanceThreshold,2.0);

    // Euclidean distance threshold
    for (std::vector<int>::iterator it = neighborsIds.begin();it!=neighborsIds.end();){
        Mat34 sensorPose = getSensorPose(*it);
        double dist = pow(cameraPose(0,3)-sensorPose(0,3),2.0) + pow(cameraPose(1,3)-sensorPose(1,3),2.0) + pow(cameraPose(2,3)-sensorPose(2,3),2.0);
       // std::cout << "id: " << *it <<  " dist: " << dist << " thresh " << distanceThreshold << "\n";
        if (dist>distanceThreshold){
          //   std::cout << "erase\n";
          //  it = neighborsIds.erase(it);
        	std::iter_swap(it, std::prev(neighborsIds.end()));
        	neighborsIds.pop_back();
        }
        else
            it++;
    }
    // Remove features which are not connnected to poses in neighborsIds
    std::set<int> featuresIds;
    // get ids of features observed from selected poses
    for (std::vector<int>::iterator it = neighborsIds.begin();it!=neighborsIds.end();it++){
        mtxCamTraj.lock();
        featuresIds.insert(camTrajectory[*it].featuresIds.begin(), camTrajectory[*it].featuresIds.end());
        mtxCamTraj.unlock();
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
std::vector<MapFeature> FeaturesMap::getVisibleFeatures(const Mat34& cameraPose) {
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
    updateMaps();
	return visibleFeatures;
}

/// get all covisible features using covisibility graph
std::vector<MapFeature> FeaturesMap::getCovisibleFeatures(void) {
    std::vector<MapFeature> visibleFeatures;
    std::set<int> verticesIds;
    //std::cout << "get neighbours " << lastKeyframeId << "\n";
    covisibilityGraph.findNeighbouringNodes(lastKeyframeId,0.0, verticesIds);
    //add last n frames
    int trajSize = getPoseCounter();
    for (int i = lastKeyframeId; i < (int)trajSize-1; i++){
        verticesIds.insert(verticesIds.end(), i);
    }
    std::set<int> featuresIds;
    if (config.useEuclideanCrit&&verticesIds.size()>0){
        std::set<int> euclVerts = getNearbyPoses((int)trajSize-1, *verticesIds.begin());
        verticesIds.insert(euclVerts.begin(), euclVerts.end());
    }
    //std::cout << "poses:\n";
    for (auto poseId : verticesIds){
        mtxCamTraj.lock();
        featuresIds.insert(camTrajectory[poseId].featuresIds.begin(), camTrajectory[poseId].featuresIds.end());
        mtxCamTraj.unlock();
        //std::cout << poseId << ", ";
    }
    //std::cout << "\n";
    mtxMapFrontend.lock();
    for (auto featureId : featuresIds){

    	auto featPointer = featuresMapFrontend.find(featureId);
    	if ( featPointer != featuresMapFrontend.end() && featPointer->second.lifeValue > 0) {
    		visibleFeatures.push_back(featPointer->second);
    		featPointer->second.lifeValue--;
    	}

//    	else
    }
    mtxMapFrontend.unlock();
    //try to update the map
    updateMaps();
    return visibleFeatures;
}

/// find nearest id of the image frame taking into acount the current angle of view and the view from the history
/// TODO: Think if we may have to think about additionally using the vector from image origin to feature as it may greatly change current comparison
void FeaturesMap::findNearestFrame(const std::vector<MapFeature>& features, std::vector<int>& imageIds, std::vector<double>& angles, double maxAngle){
    Mat34 currentCameraPose = getSensorPose();
    imageIds.resize(features.size(),-1);
    angles.resize(features.size());
    for (size_t i = 0; i<features.size();i++){
            //compute position of feature in current camera pose
            Mat34 featureGlob(Vec3(features[i].position.x(), features[i].position.y(), features[i].position.z())*Quaternion(1,0,0,0));
            Mat34 featureInCamCurr = featureGlob.inverse()*currentCameraPose;
            Eigen::Vector3f featureViewCurr((float)featureInCamCurr(0,2), (float)featureInCamCurr(1,2), (float)featureInCamCurr(2,2));
            double minRot=10; int idMin=-1;


            //find the smallest angle between two views (max dot product)
            imageIds[i]=-1;

            for (auto it = features[i].descriptors.begin(); it!= features[i].descriptors.end(); ++it)
            {
            	int poseId = it->first;

            	//compute position of feature in the camera pose
            	Mat34 camPose = getSensorPose(poseId);
                Mat34 featureInCam = featureGlob.inverse()*camPose;
                Eigen::Vector3f featureView((float)featureInCam(0,2), (float)featureInCam(1,2), (float)featureInCam(2,2));
                double angle = acos(featureView.dot(featureViewCurr)/(featureView.norm()*featureViewCurr.norm()));
                if (fabs(angle)<minRot){
                    minRot = angle;
                    idMin =  poseId;
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
void FeaturesMap::removeDistantFeatures(std::vector<MapFeature>& mapFeatures, int graphDepthThreshold, double distanceThreshold){
    std::vector<int> neighborsIds;
    //we don't have to use graph since SE3 vertex have nly one following vertex (all vertices create camera trajectory)
    //poseGraph->findNearestNeighbors(camTrajectory.size()-1, graphDepthThreshold, neighborsIds);
    int trajSize = getPoseCounter();
    for (int i=(int)trajSize-1;i>=0;i--){
        if (i>=int(trajSize-graphDepthThreshold)||graphDepthThreshold==-1)
            neighborsIds.push_back(i);
        else
            break;
    }
    Mat34 currPose = getSensorPose();
    distanceThreshold = pow(distanceThreshold,2.0);
    // Euclidean distance threshold
    for (std::vector<int>::iterator it = neighborsIds.begin();it!=neighborsIds.end();){
        Mat34 sensorPose = getSensorPose(*it);
        double dist = pow(currPose(0,3)-sensorPose(0,3),2.0) + pow(currPose(1,3)-sensorPose(1,3),2.0) + pow(currPose(2,3)-sensorPose(2,3),2.0);
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
                if ((int) *featurePoseIt == *poseIt){
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
        poseId = (int) camTrajectory.size() - 1;
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
	int size = (int) camTrajectory.size();
	mtxCamTraj.unlock();
	return size;
}

/// start optimization thread
void FeaturesMap::startOptimizationThread(unsigned int iterNo, int verbose,
        std::string RobustKernelName, double kernelDelta) {
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
    localLC->startLCsearchingThread();
    loopClosureThr.reset(new std::thread(&FeaturesMap::loopClosure, this, verbose, matcher));
}

/// Wait for optimization thread to finish
void FeaturesMap::finishOptimization(std::string trajectoryFilename,
		std::string graphFilename) {
    std::cout << "finish optimization\n";
	continueOpt = false;
    optimizationThr->join();
    std::cout << "finished optimization\n";
    exportOutput(trajectoryFilename, graphFilename);
    std::cout << "export output\n";
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
                    double dist = sqrt(pow(itFeature1->second.position.x()-itFeature2->second.position.x(),2.0) + pow(itFeature1->second.position.y()-itFeature2->second.position.y(),2.0) + pow(itFeature1->second.position.z()-itFeature2->second.position.z(),2.0));
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
    mtxMapFrontend.lock();
    size_t mapSize = featuresMapFrontend.size();
    mtxMapFrontend.unlock();
    while (continueLoopClosure && mapSize==0) {
        std::cout << "wait\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        mtxMapFrontend.lock();
        mapSize = featuresMapFrontend.size();
        mtxMapFrontend.unlock();
    }

	// Wait for some information in map
	auto start = std::chrono::system_clock::now();
	while (continueLoopClosure) {

		// Is there any pair that we should check?
		LoopClosure::LCMatch lcMatch;
		if (localLC->getLCPair(lcMatch)) {


			// Variables to store data and resultss
			int frameIds[2] =
					{ lcMatch.posesIds.first, lcMatch.posesIds.second };
			Eigen::Matrix4f estimatedTransformation;
			std::vector<std::pair<int, int>> pairedFeatures;
			std::vector<MapFeature> featureSets[2];
			double matchingRatio = 0.0;

			// Information about current analysis
			if (verbose > 0) {
				std::cout << "Loop closure: pair to analyze - " << frameIds[0]
						<< " " << frameIds[1] << std::endl;
			}

			// We perform loop closure on features
            mtxCamTraj.lock();

			// Check that each frame has sufficient number of observations
            if (((int) camTrajectory[frameIds[0]].featuresIds.size()
					> config.minNumberOfFeaturesLC)
                    && ((int) camTrajectory[frameIds[1]].featuresIds.size()
							> config.minNumberOfFeaturesLC)) {

				// Fill structures with features observed in both poses
				for (int i = 0; i < 2; i++) {
					int & currentFrameId = frameIds[i];

                    for (auto & featureId : camTrajectory[currentFrameId].featuresIds) {
                        mtxMapFrontend.lock();
                        featureSets[i].push_back(
                                featuresMapFrontend[featureId]);
                        mtxMapFrontend.unlock();
					}
				}

				// Call loop closure matching
				matchingRatio = matcher->matchFeatureLoopClosure(featureSets,
						frameIds, pairedFeatures, estimatedTransformation);
			}

            mtxCamTraj.unlock();

			// We log all analyzed pairs
			lcMatch.matchingRatio = matchingRatio;
			loopClosureLog.push_back(lcMatch);
			loopClosureMatchingRatiosLog.push_back(matchingRatio);

			// If geometric matching confirmed a loop closure
			if (matchingRatio > config.matchingRatioThresholdLC) {
				loopClosureSuccess = true;

				if (verbose > 0) {
					std::cout << "Loop closure: matchingRatio " << matchingRatio
							<< " > " << config.matchingRatioThresholdLC << "\n";
					std::cout << "Loop closure: inlier matches size(): "
							<< pairedFeatures.size() << "\n";
				}
				// Pose - feature measurements

				std::vector<MapFeature> measuredFeatures;
				std::vector<int> featureIdsToRemove;

				// For each matched pair (inlier), we need to merge measurements for those features and there are two options:
				// - merge measurements from featureA into featureB and erase feature A
				// - otherwise
				// We decided to also erase feature with greater id
				for (auto& pairFeat : pairedFeatures) {

					if (pairFeat.first < pairFeat.second) {
						// Look for a feature in set 2
						for (auto featureB : featureSets[1]) {
							// If found add new Id
							if ((int) featureB.id == pairFeat.second) {
								MapFeature featTmp = featureB;
								featTmp.id = pairFeat.first;

								featureIdsToRemove.push_back(pairFeat.second);

								// It should add all of those new descriptors
								measuredFeatures.push_back(featTmp);
							}
						}
					} else {
						// Look for a feature in set 1
						for (auto featureA : featureSets[0]) {
							// If found add new Id
							if ((int) featureA.id == pairFeat.first) {
								MapFeature featTmp = featureA;
								featTmp.id = pairFeat.second;

								featureIdsToRemove.push_back(pairFeat.first);

								// It should add all of those new descriptors
								measuredFeatures.push_back(featTmp);
							}
						}

					}
				}
				addMeasurements(measuredFeatures, frameIds[1]);
				std::sort(featureIdsToRemove.begin(), featureIdsToRemove.end(),
						std::greater<int>());
				removeFeatures(featureIdsToRemove);

			}
		} else {
			if (verbose > 1)
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
        std::string RobustKernelName, double kernelDelta) {
	// graph optimization
	continueOpt = true;

	// Wait for some information in map
	while (continueOpt && emptyMap) {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

    /// optimization clock
    Stopwatch<std::chrono::microseconds> clockTimestamp;
	while (continueOpt) {
        Stopwatch<std::chrono::microseconds> clockOpt;
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

		//try to update the map
        updateMaps();
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

        if (config.fixVertices)
            ((PoseGraphG2O*)poseGraph)->fixOptimizedVertices();
		if (verbose)
            std::cout << "end optimization\n";
        if (config.compressMap){
            int marginalizeToPose = frames2marginalize.second;
            //std::cout << "\n\n\n\n" << frames2marginalize.first << "->" << frames2marginalize.second << "\n";
            mtxCamTraj.lock();
            if (frames2marginalize.first!=marginalizeToPose&&(frames2marginalize.second<((int)camTrajectory.size()-config.minFramesNo)||(((int)camTrajectory.size()-frames2marginalize.first)>config.maxFramesNo))){
                mtxCamTraj.unlock();
                marginalizeMeasurements(frames2marginalize.first, frames2marginalize.second);
                //std::cout << "\nmarginalize " << frames2marginalize.first << "->" << frames2marginalize.second << "\n";
                //getchar();
                frames2marginalize.first = marginalizeToPose;
            }
            else
                mtxCamTraj.unlock();
            if (activeKeyframesNo>config.maxFramesNo&&config.compressMap){
                while (activeKeyframesNo>config.maxFramesNo){
                    lastFullyMarginalizedFrame++;
                    mtxCamTraj.lock();
                    if (camTrajectory[lastFullyMarginalizedFrame].isKeyframe){
                        mtxCamTraj.unlock();
                        fixMeasurementsFromPose(lastFullyMarginalizedFrame);
                        activeKeyframesNo--;
                    }
                    else
                        mtxCamTraj.unlock();
                }
            }
        }
        double timestamp = (double)clockTimestamp.stop()/1000000.0;
        double optTime = (double)clockOpt.stop()/1000000.0;
        optimizationTime.push_back(std::make_pair(timestamp,optTime));
        //std::cout << "features in Map: " << featuresMapFrontend.size() << "\n";
    }

    saveOptimizationTime(optimizationTime,"optimizationTime.m");

	// Final optimization
	if (!RobustKernelName.empty())
		setRobustKernel(RobustKernelName, kernelDelta);
	else
        disableRobustKernel();

    std::cout << "Starting final trajectory optimization" << std::endl;
    if (config.weakFeatureThr>0)
        ((PoseGraphG2O*)poseGraph)->removeWeakFeatures(config.weakFeatureThr);
    if (config.fixVertices)
        ((PoseGraphG2O*)poseGraph)->releaseFixedVertices();
    //poseGraph->optimize(-1, verbose, 0.0001);

    if (config.edges3DPrunningThreshold>0)
        ((PoseGraphG2O*) poseGraph)->prune3Dedges(config.edges3DPrunningThreshold);//pruning
    poseGraph->optimize(1, verbose);

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
    if (config.visualize){
        bufferMapVisualization.mtxBuffer.lock(); // update visualization buffer
        for (auto it = optimizedFeatures.begin(); it!=optimizedFeatures.end();it++)
            bufferMapVisualization.features2update[it->id] = *it;
        bufferMapVisualization.mtxBuffer.unlock();
    }

//    std::cout<<"features 2 update2 " << bufferMapFrontend.features2update.size() <<"\n";
	//try to update the map
   updateMaps();
	//update camera trajectory
	std::vector<VertexSE3> optimizedPoses;
	((PoseGraphG2O*) poseGraph)->getOptimizedPoses(optimizedPoses);
	updateCamTrajectory(optimizedPoses);

    restoreFrames();
}

/// save optimization time
void FeaturesMap::saveOptimizationTime(std::list<std::pair<double,double>>& optimizationTime, std::string filename){
    std::ofstream file(filename);
    file << "close all;\nclear all;\nhold on;\n";
    file << "x=[];\ny=[];\n";
    for (const auto time : optimizationTime){
        file << "x=[x, " << time.first << "];\ny=[y, " << time.second << "];\n";
    }
    file << "plot(x,y,'-or', 'LineWidth',3);\n";
    file << "xlabel('Time [s]');\n";
    file << "ylabel('Optimization time [s]');\n";
    file.close();
}

/// fixMeasurementsFromPose
void FeaturesMap::fixMeasurementsFromPose(int frameId){
    mtxCamTraj.lock();
    std::set<int> featuresIds = camTrajectory[frameId].featuresIds;
    mtxCamTraj.unlock();
    for (auto featureId : featuresIds){
        poseGraph->fixVertex(featureId);
        //std::cout << "fix feature " << featureId << "\n";
    }
    poseGraph->fixVertex(frameId);
    //std::cout << "fix pose " << frameId << "\n";
}

/// marginalize measurements between frames
void FeaturesMap::marginalizeMeasurements(int frameBegin, int frameEnd){
    std::vector<int> keyframes;
    for (int frameNo=frameBegin;frameNo<frameEnd;frameNo++){
        mtxCamTraj.lock();
        if (camTrajectory[frameNo].isKeyframe)
            keyframes.push_back(frameNo);
        mtxCamTraj.unlock();
    }
    keyframes.push_back(frameEnd);
    //find features that should be removed from the map
    std::set<int> featuresKeyframes;
    std::set<int> featuresAll;
    std::set<int> features2remove;
    //std::set<int> features2removeGraph;
    //std::set<int> features2removeGraph2;
    for (int frameNo=frameBegin;frameNo<frameEnd;frameNo++){
        mtxCamTraj.lock();
        featuresAll.insert(camTrajectory[frameNo].featuresIds.begin(), camTrajectory[frameNo].featuresIds.end());
        mtxCamTraj.unlock();
        if (config.keepCameraFrames&&!camTrajectory[frameNo].isKeyframe){
            imageSeq.erase(frameNo);
            depthSeq.erase(frameNo);
        }
    }
    //std::set<int> featuresFullOpt;
    int trajSize = getPoseCounter();
    for (int frameNo=0;frameNo<(int)trajSize;frameNo++){
        //featuresFullOpt.insert(camTrajectory[frameNo].featuresIds.begin(), camTrajectory[frameNo].featuresIds.end());
        mtxCamTraj.lock();
        if (camTrajectory[frameNo].isKeyframe){
            featuresKeyframes.insert(camTrajectory[frameNo].featuresIds.begin(), camTrajectory[frameNo].featuresIds.end());
            mtxCamTraj.unlock();
        }
        else
            mtxCamTraj.unlock();
    }
    std::set_difference(featuresAll.begin(), featuresAll.end(), featuresKeyframes.begin(), featuresKeyframes.end(), std::inserter(features2remove, features2remove.end()));
    //std::set_difference(features2remove.begin(), features2remove.end(), featuresFullOpt.begin(), featuresFullOpt.end(), std::inserter(features2remove, features2remove.end()));
    //std::set_difference(featuresAll.begin(), featuresAll.end(), camTrajectory[frameEnd].featuresIds.begin(), camTrajectory[frameEnd].featuresIds.end(), std::inserter(features2removeGraph, features2removeGraph.end()));
    //std::set_difference(featuresAll.begin(), featuresAll.end(), featuresFullOpt.begin(), featuresFullOpt.end(), std::inserter(features2removeGraph, features2removeGraph.end()));
    //std::set_difference(features2removeGraph.begin(), features2removeGraph.end(), featuresKeyframes.begin(), featuresKeyframes.end(), std::inserter(features2removeGraph, features2removeGraph.end()));
    //std::cout << "all " << featuresAll.size() << "\n";
    //std::cout << "graph " << features2remove.size() << "\n\n\n\n\n\n\n\n\n\n\n\n";


    removeFeatures(std::vector<int>(features2remove.begin(), features2remove.end()));
    updateMaps();
    poseGraph->marginalize(keyframes, features2remove);
}


/// Update map
bool FeaturesMap::updateMap(MapModifier& modifier, std::map<int,MapFeature>& featuresMap, std::recursive_mutex& mutex) {
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
        if (modifier.updateMeasurements()) {
            for (auto feature : modifier.measurements2update) {
                updateMeasurements(featuresMap, feature.second);
            }
            modifier.features2update.clear();
        }
        if (modifier.removeFeatures()) {
            for (auto feature : modifier.removeIds) {
                featuresMap.erase(feature);
            }
            modifier.removeIds.clear();
        }
		modifier.mtxBuffer.unlock();
		mutex.unlock();

		return true;
	}
	return false;
}


/// Update feature
void FeaturesMap::updateFeature(std::map<int,MapFeature>& featuresMap, const MapFeature& newFeature) {
	// Update position - used after optimization TODO: We shouldn't update position when adding descriptors
	featuresMap[newFeature.id].position = newFeature.position;
//    // The measurement contains new extended descriptor and we add to the feature information
//    if (featuresMap.count(newFeature.id) > 0 && newFeature.descriptors.size() > 0) {
//		// Analyze the one of the new descriptors
//        for (auto & descToAdd : newFeature.descriptors) {
//			// This adds new descriptor if there is not descriptor with provided key
//			featuresMap[newFeature.id].descriptors[descToAdd.first] = descToAdd.second;
//		}
//	}
}

Eigen::Vector3d FeaturesMap::getFeatureVectorInLCS(int poseId, Mat34 featureInGCS) {

	// Getting camera pose in GCS
	Mat34 camPoseNew = getSensorPose(poseId);

	// Computing feature pose in LCS
	Mat34 featureInCamNew = featureInGCS.inverse() * camPoseNew;

	// Getting feature translation vector in LCS
	Eigen::Vector3d featureViewNew(featureInCamNew(0, 2), featureInCamNew(1, 2),
			featureInCamNew(2, 2));

	return featureViewNew;
}

/// Update measurements
void FeaturesMap::updateMeasurements(std::map<int,MapFeature>& featuresMap, const std::pair<int, MapFeature>& newFeature) {
    featuresMap[newFeature.second.id].posesIds.push_back(newFeature.first);
    featuresMap[newFeature.second.id].imageCoordinates.insert(std::make_pair(newFeature.first,ImageFeature(newFeature.second.u, newFeature.second.v, newFeature.second.position.z())));


    // Feature position in global coordinate system
    Mat34 featureInGCS(featuresMap[newFeature.second.id].position*Quaternion(1,0,0,0));

    // For all new possible descriptors
    for (auto &desc : newFeature.second.descriptors) {

    	// Lets assume that we will add new descriptor
    	bool shouldWeAddDescriptor = true;

    	// Getting the vector for new feature
    	Eigen::Vector3d featureViewNew = getFeatureVectorInLCS(desc.first, featureInGCS);

    	for (auto &existingDesc : featuresMap[newFeature.second.id].descriptors){

    		// Getting the vector for already existing descriptors
    		Eigen::Vector3d featureView = getFeatureVectorInLCS(existingDesc.first, featureInGCS);

			// Computing the angle between vectors
			double angle = acos(
					featureView.dot(featureViewNew)
							/ (featureView.norm() * featureViewNew.norm()));
			double angleDeg = angle * M_PI / 180;

			// If it is less than 30 deg, we do not add new descriptor
			if (angleDeg < 30) {
				shouldWeAddDescriptor = false;
				break;
			}
    	}

    	if (shouldWeAddDescriptor)
    		featuresMap[newFeature.second.id].descriptors[desc.first] = desc.second;
    }


    featuresMap[newFeature.second.id].lifeValue += 5 ;
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
	if ((int)newPose.vertexId > lastOptimizedPose)
		lastOptimizedPose = (int)newPose.vertexId;
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
    mtxCamTraj.lock();
    for (std::vector<VertexSE3>::iterator it = camTrajectory.begin();
            it != camTrajectory.end(); it++) {
		file << "Pose " << it->vertexId;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				file << " " << it->pose(i, j);
		file << "\n";
	}
    mtxCamTraj.unlock();
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
void FeaturesMap::computeMeanStd(const std::vector<double>& v, double& mean, double& std, double& max){
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / (double)v.size();
    max=-10;
    for (auto it=v.begin();it!=v.end();it++){
        if (*it>max)
            max=*it;
    }

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    std = std::sqrt(sq_sum / (double)v.size() - mean * mean);
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
    for (int i=FEATURES_START_ID;i<(int)featureIdNo;i++){
        MapFeature tmpFeature = featuresMapFrontend[i];
        // for each frame position
        file << "%feature id: " << i << "\n";
        if (tmpFeature.posesIds.size()>10){
            double red = distribution(generator); double green = distribution(generator); double blue = distribution(generator);
            for (auto it = tmpFeature.posesIds.begin(); it!=tmpFeature.posesIds.end(); it++){
                Mat34 currCamPose = getSensorPose(*it);
                //std::cout << "CurrCamPose\n" << currCamPose.matrix() << "\n";
				Eigen::Vector3d point;
				sensorModel.getPoint(tmpFeature.imageCoordinates[*it].u,
						tmpFeature.imageCoordinates[*it].v,
						tmpFeature.imageCoordinates[*it].depth, point);
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
    //std::vector<double> meanX, meanY, meanZ;
    std::vector<double> meanDist, stdevDist;
    std::vector<double> maxDist;
    std::vector<double> measurementsNo;
    for (int i=FEATURES_START_ID;i<(int)featureIdNo;i++){
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
        std::vector<double> dist4estim;
        measurementsNo.push_back((double)features.size());
        if (features.size()>0){ //how come?
            //std::cout << "dist:\n";
            double sumPos[3]={0,0,0};
            // compute the center of mass
            for (int j=0;j<(int)features.size();j++){
                sumPos[0]+=features[j].trans.x(); sumPos[1]+=features[j].trans.y(); sumPos[2]+=features[j].trans.z();
            }
            estimation.x()=sumPos[0]/(double)features.size(); estimation.y()=sumPos[1]/(double)features.size(); estimation.z()=sumPos[2]/(double)features.size();
            file << "plot3(" << estimation.x() << "," << estimation.y() << "," << estimation.z() << ",'ro');\n";
            for (int j=0;j<(int)features.size();j++){
                double dist = sqrt(pow(features[j].trans.x()-estimation.x(),2.0)+pow(features[j].trans.y()-estimation.y(),2.0)+pow(features[j].trans.z()-estimation.z(),2.0));
                //std::cout << dist << ", ";
                dist4estim.push_back(dist);
                file << "plot3(" << features[j].trans.x() << "," << features[j].trans.y() << "," << features[j].trans.z() << ",'bx');\n";
            }
            double mean, std, max;
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
    std::vector<double> meanDistU; std::vector<double> meanDistV;
    std::vector<double> maxDistU; std::vector<double> maxDistV;
    int iterFrame=0;
    mtxCamTraj.lock();
    for (auto it = camTrajectory.begin(); it!=camTrajectory.end();it++){
        double meanU = 0; double meanV = 0;
        double maxU = 0; double maxV = 0;
        int featuresNo = 0;
        for (auto itFeat=camTrajectory[iterFrame].featuresIds.begin();itFeat!=camTrajectory[iterFrame].featuresIds.end();itFeat++){
            std::vector<Edge3D> features;
            Vec3 estimation;
            ((PoseGraphG2O*)poseGraph)->getMeasurements(*itFeat, features, estimation);
            // compute the center of mass
            double sumPos[3]={0,0,0};
            for (int j=0;j<(int)features.size();j++){
                sumPos[0]+=features[j].trans.x(); sumPos[1]+=features[j].trans.y(); sumPos[2]+=features[j].trans.z();
            }
            estimation.x()=sumPos[0]/(double)features.size(); estimation.y()=sumPos[1]/(double)features.size(); estimation.z()=sumPos[2]/(double)features.size();
            //estimation pose in camera frame
            Mat34 feature(Quaternion(1,0,0,0)*estimation);
            Mat34 featInCam = camTrajectory[iterFrame].pose.inverse()*feature;
            Eigen::Vector3d featCam = sensorModel.inverseModel(featInCam(0,3), featInCam(1,3), featInCam(2,3));
            double meanFeatU = 0; double meanFeatV = 0;
            double maxFeatU = -10; double maxFeatV = -10;
            if (featCam(0)>0&&featCam(1)>1){/// for all measurements of the feature
                int measNo=0;
                for (int j=0;j<(int)features.size();j++){
                    Mat34 featureLocal(Quaternion(1,0,0,0)*features[j].trans);
                    Mat34 featLocalInCam = camTrajectory[iterFrame].pose.inverse()*featureLocal;
                    Eigen::Vector3d featLocalCam = sensorModel.inverseModel(featLocalInCam(0,3), featLocalInCam(1,3), featLocalInCam(2,3));
                    if (featLocalCam(0)>0&&featLocalCam(1)>1){
                        double distU = fabs(featLocalCam(0)-featCam(0));
                        double distV = fabs(featLocalCam(1)-featCam(1));
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
    mtxCamTraj.unlock();
    file.close();
    std::ofstream fileData(filenameData);
    fileData << "\nclear all;\n";
    double mean, std, max;
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
void FeaturesMap::setRobustKernel(std::string name, double delta) {
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
/*Mat66 FeaturesMap::getPoseUncertainty(unsigned int id) const{
    Mat66 incCov = ((PoseGraphG2O*) poseGraph)->getPoseIncrementCovariance(id);
    Mat66 unc = G2OEst::computeCovarianceMatrix(incCov, getSensorPose(id).inverse());
    return unc;
}*/

/// get uncertainty of the feature
/*Mat33 FeaturesMap::getFeatureUncertainty(unsigned int id) const{
    Mat33 incCov = ((PoseGraphG2O*) poseGraph)->getFeatureIncrementCovariance(id);
    Mat33 unc = G2OEst::computeCovarianceMatrix(incCov, getFeaturePosition(id).inverse());
    return unc;
}*/

int FeaturesMap::getNumberOfFeatures() {
	mtxMapFrontend.lock();
	int val = (int)featuresMapFrontend.size();
	mtxMapFrontend.unlock();
	return val;
}

/// Restore camera frames (previously marginalized)
void FeaturesMap::restoreFrames(void){
    mtxCamTraj.lock();
    Mat34 prevPose(camTrajectory.front().pose);
    bool first(true);
    for (auto cameraNode : camTrajectory){
        Mat34 camPose = cameraNode.pose;
        if (!cameraNode.isKeyframe&&(int)cameraNode.vertexId<frames2marginalize.second){
            //add camera pose to the graph
            poseGraph->addVertexPose(cameraNode);
        }
        if (!first){
            poseGraph->addEdgeSE3(EdgeSE3(prevPose.inverse()*camPose, Mat66::Identity(), cameraNode.vertexId-1, cameraNode.vertexId));
        }
        else{
            first=false;
        }
        prevPose = camPose;
    }
    mtxCamTraj.unlock();
}


std::vector<double> FeaturesMap::getLoopClosureMatchingRatiosLog() {
	return loopClosureMatchingRatiosLog;
}

std::vector<LoopClosure::LCMatch> FeaturesMap::getLoopClosureAnalyzedPairsLog() {
	return loopClosureLog;
}

bool FeaturesMap::getAndResetLoopClosureSuccesful() {
	if ( loopClosureSuccess ) {
		loopClosureSuccess = false;
		return true;
	}
	return false;
}

/// get nerby camera poses using Euclidean criteria
std::set<int> FeaturesMap::getNearbyPoses(int currentFrameId, int startFrameId){
    std::set<int> posesIds;
    mtxCamTraj.lock();
    Mat34 currentPose(camTrajectory[currentFrameId].pose);
    mtxCamTraj.unlock();
    //std::cout << "currentFrameId" << currentFrameId << "\n";
    //std::cout << "startFrameId" << startFrameId << "\n";
    for (int frameNo = startFrameId-1; frameNo>=0; frameNo--){
        mtxCamTraj.lock();
        if (camTrajectory[frameNo].isKeyframe){
            //std::cout << "check frame " << frameNo << "\n";
            /// compute frameNo in current frame
            Mat34 relTransform = camTrajectory[frameNo].pose.inverse()*currentPose;
            double dist = sqrt(pow(relTransform(0,3),2.0)+pow(relTransform(1,3),2.0)+pow(relTransform(2,3),2.0));
            if (dist>config.maxRadius) {
                mtxCamTraj.unlock();
                break;
            }
            double planarDist = sqrt(pow(relTransform(0,3),2.0)+pow(relTransform(1,3),2.0));
            if (planarDist<config.imagePlaneDistance){
                double dotProd = currentPose.matrix().block<3,1>(0,2).dot(camTrajectory[frameNo].pose.matrix().block<3,1>(0,2));
                double angle = acos(dotProd);
                if (angle<config.maxAngle&&fabs(relTransform(2,3))<config.depthDist){
                    posesIds.insert(frameNo);
                    //std::cout << "found eucl " << currentFrameId << "-> " << frameNo << "\n";
                }
            }
            mtxCamTraj.unlock();
        }
        else
            mtxCamTraj.unlock();
    }
    return posesIds;
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
