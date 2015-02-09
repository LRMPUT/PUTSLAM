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
FeaturesMap::FeaturesMap(std::string configFileGrabber, std::string sensorConfig) : featureIdNo(FATURES_START_ID), sensorModel(sensorConfig), Map("Features Map", MAP_FEATURES){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFileGrabber;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load config file.\n";
    poseGraph = createPoseGraphG2O();
}

/// Destruction
FeaturesMap::~FeaturesMap(void){
}

const std::string& FeaturesMap::getName() const {
    return name;
}

/// Add NEW features to the map
/// Position of features in relation to camera pose
void FeaturesMap::addFeatures(const std::vector<RGBDFeature>& features, unsigned int poseId) {
    Mat34 cameraPose = (poseId>0) ? camTrajectory[poseId] : camTrajectory.back();
    for (std::vector<RGBDFeature>::const_iterator it = features.begin(); it!=features.end();it++){
        //add each feature to map structure...
        featuresSet.push_back(MapFeature(featureIdNo,0, 0, (*it).position, std::vector<unsigned int>(), (*it).descriptors));
        //.. and the graph
        Mat34 featurePos((*it).position);
        featurePos=cameraPose.matrix()*featurePos.matrix();
        //add measurement
        Mat33 info; info = sensorModel.informationMatrix((*it).position.x(), (*it).position.y(), (*it).position.z());
        Edge3D e((*it).position,info,camTrajectory.size()-1,featureIdNo);
        poseGraph->addVertexFeature(Vertex3D(featureIdNo, Vec3(featurePos(0,3),featurePos(1,3),featurePos(2,3))));
        poseGraph->addEdge3D(e);
        featureIdNo++;
    }
}

/// add new pose of the camera, returns id of the new pose
int FeaturesMap::addNewPose(const Mat34& cameraPose, float_type timestamp){
    //add camera pose to the map
    camTrajectory.push_back(cameraPose);
    //add camera pose to the graph
    poseGraph->addVertexPose(VertexSE3(camTrajectory.size(),Vec3(cameraPose(0,3), cameraPose(1,3), cameraPose(2,3)), Quaternion(cameraPose.rotation())));
    camTrajectory.push_back(cameraPose);
    return camTrajectory.size()-1;
}

/// add measurements (features measured from the last camera pose)
void FeaturesMap::addMeasurements(const std::vector<MapFeature>& features, unsigned int poseId){
    unsigned int _poseId = (poseId>0) ? poseId : camTrajectory.size()-1;
    for (std::vector<MapFeature>::const_iterator it = features.begin(); it!=features.end();it++){
        //add measurement
        Mat33 info; info = sensorModel.informationMatrix((*it).position.x(), (*it).position.y(), (*it).position.z());
        Edge3D e((*it).position,info,_poseId,(*it).id);
        poseGraph->addEdge3D(e);
    }
}

/// Get all features
std::vector<MapFeature> FeaturesMap::getAllFeatures(void) {
    return featuresSet;
}

/// Get feature position
Vec3 FeaturesMap::getFeaturePosition(unsigned int id) {
    return featuresSet[FATURES_START_ID+id].position;
}

/// get all visible features
std::vector<MapFeature> FeaturesMap::getVisibleFeatures(const Mat34& cameraPose) {
    std::vector<MapFeature> visibleFeatures;
    for (std::vector<MapFeature>::iterator it = featuresSet.begin(); it!=featuresSet.end();it++){
        Mat34 featurePos((*it).position);
        Mat34 featureCam = cameraPose.inverse() * featurePos;
        Eigen::Vector3d pointCam = sensorModel.inverseModel(featureCam(0,3), featureCam(1,3), featureCam(2,3));
        //std::cout << pointCam(0) << " " << pointCam(1) << " " << pointCam(2) << "\n";
        if (pointCam(0)!=-1){
            visibleFeatures.push_back(*it);
        }
    }
    return visibleFeatures;
}

/// get pose of the sensor (default: last pose)
Mat34 FeaturesMap::getSensorPose(unsigned int poseId){
    return (poseId>0) ? camTrajectory[poseId] : camTrajectory.back();
}

/// start optimization thread
void FeaturesMap::startOptimizationThread(unsigned int iterNo){
    optimizationThr.reset(new std::thread(&FeaturesMap::optimize,this,iterNo));
}

/// Wait for optimization thread to finish
void FeaturesMap::finishOptimization(){
    continueOpt = false;
    poseGraph->save2file("map.g2o");
    optimizationThr->join();
}

/// optimization thread
void FeaturesMap::optimize(unsigned int iterNo){
    // graph optimization
    continueOpt = true;
    while (continueOpt){
        std::cout << "start optimization\n";
        poseGraph->optimize(iterNo,0);
        std::cout << "end optimization\n";
    }
}

putslam::Map* putslam::createFeaturesMap(void) {
    map.reset(new FeaturesMap());
    return map.get();
}

putslam::Map* putslam::createFeaturesMap(std::string configFileGrabber, std::string configSensor) {
    map.reset(new FeaturesMap(configFileGrabber, configSensor));
    return map.get();
}
