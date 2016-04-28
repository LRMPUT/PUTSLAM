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

    class LCElement{
    public:
        /// matched poses
        std::pair<int,int> posesIds;

        /// distance between poses, the smaller the higher priority in the queue
        double distance;

<<<<<<< HEAD
        /// id difference
        int idDifference;

        bool operator() (const LCElement& elementA, const LCElement& elementB) const {
            if (elementA.distance>elementB.distance)
//        	if (elementA.idDifference<elementB.idDifference)
=======
        bool operator() (const LCElement& elementA, const LCElement& elementB) const {
            if (elementA.distance>elementB.distance)
>>>>>>> 3d7b5dc4acb7309a57e560af665d2ccc2724bfc0
                return true;
            else
                return false;
        }
    };

	/// overloaded constructor
    LoopClosure(const std::string _name, Type _type) :
			name(_name), type(_type) {
    }

    /// Name of the LoopClosure
    virtual const std::string& getName() const {return name;}

	/// Virtual descrutor
    virtual ~LoopClosure() {
	}

    /// add new pose
    virtual void addPose(const Mat34& cameraPose, cv::Mat& imageRGB) {
        imagesSeq.push_back(imageRGB);
        cameraPoses.push_back(cameraPose);
    }

    /// get candidate poses for LC (false -- no candidates)
    virtual bool getLCPair(std::pair<int,int>& candidates) {
        if (priorityQueueLC.size()>0){
            LCElement element = priorityQueueLC.top();
            candidates = element.posesIds;
            priorityQueueLC.pop();
            return true;
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

    /// poses (random access, not continous storage)
    std::deque<Mat34> cameraPoses;

    /// images (random access, not continous storage)
    std::deque<cv::Mat> imagesSeq;

    /// loop closure priority queue
    std::priority_queue<LCElement, std::vector<LCElement>, LCElement > priorityQueueLC;
};
}

#endif // _LOOPCLOSURE_H_
