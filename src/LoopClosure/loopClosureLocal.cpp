#include "../include/LoopClosure/loopClosureLocal.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"

using namespace putslam;

/// A single instance of LoopCLosureLocal
LoopClosureLocal::Ptr localLC;

LoopClosureLocal::LoopClosureLocal(void) : LoopClosure("Local Loop Closure", LC_LOCAL){
}

/// Construction
LoopClosureLocal::LoopClosureLocal(std::string configFilename) : config(configFilename), LoopClosure("Local Loop Closure", LC_LOCAL) {
    currentFrame = config.minFrameDist;
}

/// Destruction
LoopClosureLocal::~LoopClosureLocal(void) {
}

LoopClosureLocal::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load Local Loop Closure config file.\n";
    tinyxml2::XMLElement * model = config.FirstChildElement( "LoopClosure" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("minFrameDist", &minFrameDist);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("distThresholdLC", &distThreshold);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("rotThresholdLC", &rotThreshold);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("useImagesLC", &useImages);
}

/// start loop closure thread (thread updates priority queue)
void LoopClosureLocal::startLCsearchingThread(void){
    continueLCsearchingThread=true;
    loopClosureThr.reset(new std::thread(&LoopClosureLocal::updatePriorityQueue, this));
}

/// Wait for loop closure thread to finish
void LoopClosureLocal::finishLCsearchingThr(void){
    continueLCsearchingThread = false;
    loopClosureThr->join();
}

/// geometric loop closure method
void LoopClosureLocal::updatePriorityQueue(void){
    while (continueLCsearchingThread) {
        if (config.verbose>0){
            std::cout << "Update priority queue: start new iteration\n";
        }
        if (config.minFrameDist>=cameraPoses.size()||currentFrame>=cameraPoses.size()){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        else{
            Mat34 currentPose = cameraPoses[currentFrame];
            for (int i=0;i<currentFrame-config.minFrameDist;i++){
                Mat34 prevPose  = cameraPoses[i];
                double dotprod = (double)(1.0-currentPose.matrix().block<3,1>(0,2).adjoint()*prevPose.matrix().block<3,1>(0,2))/2.0;
                Vec3 p1(prevPose(0,3), prevPose(1,3), prevPose(2,3));
                Vec3 p2(currentPose(0,3), currentPose(1,3), currentPose(2,3));
                double euclDist = sqrt(pow(p1.x()-p2.x(),2.0)+pow(p1.y()-p2.y(),2.0)+pow(p1.z()-p2.z(),2.0));
                LCElement element;
                element.distance = dotprod*euclDist;
                if ((element.distance<config.distThreshold)&&(acos(1-dotprod*2)<config.rotThreshold)){
                    //std::cout << "add to queque " << i << "->" << frameId << "\n";
                    element.posesIds = std::make_pair(i,currentFrame);
                    priorityQueueLC.push(element);
                }
            }
            currentFrame++;
        }
    }
}

putslam::LoopClosure* putslam::createLoopClosureLocal(void) {
    localLC.reset(new LoopClosureLocal());
    return localLC.get();
}

putslam::LoopClosure* putslam::createLoopClosureLocal(std::string configFilename) {
    localLC.reset(new LoopClosureLocal(configFilename));
    return localLC.get();
}
