#include "../include/LoopClosure/loopClosureLocal.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"

using namespace putslam;

/// A single instance of LoopCLosureLocal
LoopClosureLocal::Ptr localLC;

LoopClosureLocal::LoopClosureLocal(void) : LoopClosure("Local Loop Closure", LC_LOCAL){
}

/// Construction
LoopClosureLocal::LoopClosureLocal(std::string configFilename) : LoopClosure("Local Loop Closure", LC_LOCAL), config(configFilename) {
    currentFrame = 0;

    vpr.reset(new VisualPlaceRecognition(config.minFeatures, config.tailFramesToSkip, config.minNewPlaceProb));
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
    model->FirstChildElement( "parameters" )->QueryIntAttribute("minFeatures", &minFeatures);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("tailFramesToSkip", &tailFramesToSkip);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("minNewPlaceProb", &minNewPlaceProb);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("maxPQSize", &maxPQSize);
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

    	imageDataMtx.lock();
    	size_t imagesSeqSize = imagesSeq.size();
    	imageDataMtx.unlock();


        if (imagesSeqSize == 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        else{
			if (config.verbose > 1) {
				std::cout << "LoopClosureLocal - we analyze new frame\n";
			}

			// We use FABMAP to find all potentially promissing match to current frame to analyze
			imageDataMtx.lock();
			cv::Mat frame = imagesSeq.front();
			imagesSeq.pop_front();
			imageDataMtx.unlock();

            std::vector<std::pair<int, double>> candidates = vpr.get()->findAddPlace(frame, currentFrame, true);

			// For each candidate (int -> id, double -> probability)
			for (std::pair<int, double> candidate : candidates) {

				// We create match
				LCMatch element;
				element.probability = candidate.second;

				// Convert from LC ids to putslam ids of frames
				imageDataMtx.lock();
				int indexA = frameIds[currentFrame];
				int indexB = frameIds[candidate.first];
				imageDataMtx.unlock();
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
			checkAndTrimPQSize();

            currentFrame++;
        }
    }
}

void LoopClosureLocal::checkAndTrimPQSize() {
	priorityQueueMtx.lock();
	size_t size = priorityQueueLC.size();
	if (size > config.maxPQSize) {
		std::priority_queue<LCMatch, std::vector<LCMatch>, LCMatch> trimmedPQ;

		for (uint i = 0; i < config.maxPQSize / 2; i++) {
			LCMatch element = priorityQueueLC.top();
			trimmedPQ.push(element);
			priorityQueueLC.pop();
		}
		priorityQueueLC = trimmedPQ;
	}
	priorityQueueMtx.unlock();
}

putslam::LoopClosure* putslam::createLoopClosureLocal(void) {
    localLC.reset(new LoopClosureLocal());
    return localLC.get();
}

putslam::LoopClosure* putslam::createLoopClosureLocal(std::string configFilename) {
    localLC.reset(new LoopClosureLocal(configFilename));
    return localLC.get();
}
