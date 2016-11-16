/** @file loopCLosureLocal.h
 *
 * implementation - local loop closure
 *
 */

#ifndef LOOPCLOSURELOCAL_H_INCLUDED
#define LOOPCLOSURELOCAL_H_INCLUDED

#include "loopClosure.h"
#include <iostream>

#include "../VisualPlaceRecognition/visualplacerecognition.h"

namespace putslam {
/// create a single LC
LoopClosure* createLoopClosureLocal(void);
/// create a single LC - overloaded
LoopClosure* createLoopClosureLocal(std::string configFilename);
}

using namespace putslam;

/// LoopClosure implementation
class LoopClosureLocal: public LoopClosure {
public:
	/// Pointer
    typedef std::unique_ptr<LoopClosureLocal> Ptr;

	/// Construction
    LoopClosureLocal(void);

	/// Construction
    LoopClosureLocal(std::string config);

	/// Destruction
    ~LoopClosureLocal(void);

    /// start loop closure thread (thread updates priority queue)
    void startLCsearchingThread(void);

    /// Wait for loop closure thread to finish
    void finishLCsearchingThr(void);

    class Config{
      public:
        Config(){}
        Config(std::string configFilename);

        public:
            //verbose
            int verbose;

            ///
            int minFeatures;

            ///
            int tailFramesToSkip;

            ///
            double minNewPlaceProb;

            ///
            int maxPQSize;
    };

private:
    ///Configuration of the module
    Config config;

    /// Visual Loop Closure FABMAP
    std::unique_ptr<VisualPlaceRecognition> vpr;

    /// Loop closure thread
    std::unique_ptr<std::thread> loopClosureThr;

    /// LC thread flag
    bool continueLCsearchingThread;

    /// geometric loop closure method
    void updatePriorityQueue(void);

    ///
    void checkAndTrimPQSize();

    /// current frame
    int currentFrame;
};

#endif // LOOPCLOSURELOCAL_H_INCLUDED
