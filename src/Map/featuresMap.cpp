#include "../include/Map/featuresMap.h"
#include "../include/PoseGraph/graph.h"
#include <memory>
#include <stdexcept>
#include <chrono>

using namespace putslam;

/// A single instance of Elevation Map
FeaturesMap::Ptr map;

FeaturesMap::FeaturesMap(void) : featureIdNo(FATURES_START_ID), Map("Features Map", MAP_FEATURES) {
    poseGraph = createPoseGraphG2O();
}

/// Construction
FeaturesMap::FeaturesMap(std::string sensorConfig) : sensorModel(sensorConfig), Map("Features Map", MAP_FEATURES){

}

/// Destruction
FeaturesMap::~FeaturesMap(void){
}

const std::string& FeaturesMap::getName() const {
    return name;
}

/// Add NEW features and a NEW camera pose (initial guess) to the map
/// Position of features in relation to camera pose
void FeaturesMap::addFeatures(const std::vector<RGBDFeature>& features, const Mat34& cameraPose) {
    //add camera pose
    poseGraph->addVertexPose(VertexSE3(camTrajectory.size(),Vec3(cameraPose(0,3), cameraPose(1,3), cameraPose(2,3)), Quaternion(cameraPose.rotation())));
    camTrajectory.push_back(cameraPose);

    for (std::vector<RGBDFeature>::const_iterator it = features.begin(); it!=features.end();it++){
        //add each feature to map structure...
        featuresSet.push_back(MapFeature(featureIdNo, (*it).position, (*it).descriptors));
        //.. and the graph
        Mat34 featurePos((*it).position);
        featurePos=cameraPose.matrix()*featurePos.matrix();
        //add measurement
        Mat33 info;
        Edge3D e((*it).position,info,camTrajectory.size()-1,featureIdNo);
        poseGraph->addVertexFeature(Vertex3D(featureIdNo, Vec3(featurePos(0,3),featurePos(1,3),featurePos(2,3))));
        poseGraph->addEdge3D(e);
        featureIdNo++;
    }
    //MapFeature.
}

/// Get all features
std::vector<MapFeature>& FeaturesMap::getAllFeatures(void) {

}

/// Get feature position
Vec3 FeaturesMap::getFeaturePosition(unsigned int id) {

}

/// get all visible features
std::vector<MapFeature>& FeaturesMap::getVisibleFeatures(const Mat34& cameraPose) {

}

/// get current pose of the sensor
Mat34 FeaturesMap::getCurrentPose(void) {
    return camTrajectory.back();
}

/// start optimization thread
void FeaturesMap::startOptimizationThread(unsigned int iterNo){
    optimizationThr.reset(new std::thread(&FeaturesMap::optimize,this,iterNo));
}

/// Wait for optimization thread to finish
void FeaturesMap::finishOptimization(){
    continueOpt = false;
    optimizationThr->join();
}

/// optimization thread
void FeaturesMap::optimize(unsigned int iterNo){
    // graph optimization
    continueOpt = true;
    while (continueOpt){
        std::cout << "start optimization\n";
        poseGraph->optimize(iterNo);
        std::cout << "end optimization\n";
    }
}

putslam::Map* putslam::createFeaturesMap(void) {
    map.reset(new FeaturesMap());
    return map.get();
}

putslam::Map* putslam::createFeaturesMap(std::string configSensor) {
    map.reset(new FeaturesMap(configSensor));
    return map.get();
}
