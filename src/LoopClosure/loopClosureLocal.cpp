#include "../include/LoopClosure/loopClosureLocal.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"

using namespace putslam;

/// A single instance of LoopCLosureLocal
LoopClosureLocal::Ptr localLC;

LoopClosureLocal::LoopClosureLocal(void) : LoopClosure("Local Loop Closure", LC_LOCAL){
}

/// Construction
LoopClosureLocal::LoopClosureLocal(std::string configFilename) : LoopClosure("Local Loop Closure", LC_LOCAL), config(configFilename) {
    currentFrame = config.minFrameDist;
    vpr.reset(new VisualPlaceRecognition());
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
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("distThreshold", &distThreshold);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("rotThreshold", &rotThreshold);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("useImages", &useImages);
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
        if (config.minFrameDist>=(int)cameraPoses.size()||currentFrame>=(int)cameraPoses.size()){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        else{
           // Mat34 currentPose = cameraPoses[currentFrame];


            std::vector<std::pair<int, double>> similarPlaces = vpr.get()->findAddPlace(imagesSeq[currentFrame], currentFrame, true);

            for ( std::pair<int, double> place : similarPlaces)
            {

				if ( place.first >= 0)
				{
					LCElement element;
					element.distance = 0;
					element.probability = place.second;

					int indexA = frameIds[currentFrame];
					int indexB = frameIds[place.first];
					element.posesIds = std::make_pair(indexA, indexB);
					priorityQueueLC.push(element);

					if (config.verbose > 0){
						std::cout<<std::endl<<"FABMAP proposes: " << indexA << " " << indexB << " Probability: " << place.second <<std::endl;
					}
				}
				else if (config.verbose > 0){
					std::cout<<"Fabmap found nothing"<<std::endl;
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
