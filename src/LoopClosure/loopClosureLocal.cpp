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

        if (config.minFrameDist>=(int)cameraPoses.size()||currentFrame>=(int)cameraPoses.size()){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        else{
			if (config.verbose > 1) {
				std::cout << "LoopClosureLocal - we analyze new frame\n";
			}

			// We use FABMAP to find all potentially promissing match to current frame to analyze
			cv::Mat frame = imagesSeq.front();
			imagesSeq.pop_front();
            std::vector<std::pair<int, double>> candidates = vpr.get()->findAddPlace(frame, currentFrame, true);

			// For each candidate (int -> id, double -> probability)
			for (std::pair<int, double> candidate : candidates) {

				// We create match
				LCMatch element;
				element.distance = 0;
				element.probability = candidate.second;

				// Convert from LC ids to putslam ids of frames
				int indexA = frameIds[currentFrame];
				int indexB = frameIds[candidate.first];
				element.posesIds = std::make_pair(indexA, indexB);

				// We add to priority queue
				priorityQueueMtx.lock();
				priorityQueueLC.push(element);
				priorityQueueMtx.unlock();

				if (config.verbose > 0) {
					std::cout << "FABMAP proposes: " << indexA
							<< " " << indexB << " Probability: "
							<< candidate.second << " PQ size: "
							<< priorityQueueLC.size() << std::endl;
				}
			}


            // Keep priority size reasonable - if it is more than 100, we trim to 50
            priorityQueueMtx.lock();
            int size = priorityQueueLC.size();
            if (size > 100) {
            	std::priority_queue<LCMatch, std::vector<LCMatch>, LCMatch > trimmedPQ;

            	for (int i=0;i<size - 50;i++){
					LCMatch element = priorityQueueLC.top();
					trimmedPQ.push(element);
					priorityQueueLC.pop();
            	}
            	priorityQueueLC = trimmedPQ;
            }
            priorityQueueMtx.unlock();

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
