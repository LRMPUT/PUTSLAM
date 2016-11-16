/** @file map.h
 *
 * Loop closure interface
 *
 */

#ifndef _LOOPCLOSURE_H_
#define _LOOPCLOSURE_H_

#include "../Defs/putslam_defs.h"
#include <deque>
#include <queue>
#include <thread>
#include <iostream>
#include <mutex>

namespace putslam {

/// LoopClosure interface
class LoopClosure {
public:

    /// LoopClosure type
	enum Type {
        /// for local SLAM
        LC_LOCAL,
        /// FABMAP
        LC_FABMAP
	};

    class LCMatch{
    public:
        /// matched poses
        std::pair<int,int> posesIds;

        /// distance between poses, the smaller the higher priority in the queue
        double distance;

        /// id difference
        int idDifference;

        /// Fabmap probability
        double probability;

        /// Geometric matching ratio
        double matchingRatio;

        bool operator() (const LCMatch& elementA, const LCMatch& elementB) const {
        	 if (elementA.probability < elementB.probability)
                return true;
            else
                return false;
        }
    };

	/// overloaded constructor
    LoopClosure(const std::string _name, Type _type) :
    	type(_type), name(_name) {
    }

    /// Name of the LoopClosure
    virtual const std::string& getName() const {return name;}

	/// Virtual descrutor
    virtual ~LoopClosure() {
	}

    /// add new pose
    virtual void addPose(const Mat34& cameraPose, cv::Mat& imageRGB, int frameId) {

    	imageDataMtx.lock();
        imagesSeq.push_back(imageRGB);
        cameraPoses.push_back(cameraPose);
        frameIds.push_back(frameId);
        imageDataMtx.unlock();
    }

    /// get candidate poses for LC (false -- no candidates)
    virtual bool getLCPair(LCMatch &lcMatch) {

    	if(priorityQueueMtx.try_lock())
    	{
			if (priorityQueueLC.size()>0){
				lcMatch = priorityQueueLC.top();
				priorityQueueLC.pop();
				priorityQueueMtx.unlock();
				return true;
			}
			priorityQueueMtx.unlock();
    	}
        return false;
    }

    /// start loop closure thread (thread updates priority queue)
    virtual void startLCsearchingThread(void) = 0;

    /// Wait for loop closure thread to finish
    virtual void finishLCsearchingThr(void) = 0;

protected:
    /// LC type
	Type type;

    /// LC name
	const std::string name;

	/// mutex to protect imagesSeq, cameraPoses and frameIds
	std::mutex imageDataMtx;

    /// poses (random access, not continous storage)
    std::deque<Mat34> cameraPoses;

    /// images (random access, not continous storage)
    std::deque<cv::Mat> imagesSeq;

    /// images (random access, not continous storage)
    std::deque<int> frameIds;

    /// loop closure priority queue
    std::mutex priorityQueueMtx;
    std::priority_queue<LCMatch, std::vector<LCMatch>, LCMatch > priorityQueueLC;
};
}

#endif // _LOOPCLOSURE_H_
