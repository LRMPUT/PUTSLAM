#ifndef _MOTIONMODEL_
#define _MOTIONMODEL_

#include "Defs/putslam_defs.h"

class MotionModel
{
public:
	// Motion model predicts new pose after dt time
	virtual putslam::Mat34 predict(float dt) = 0;

	// Current pose is corrected from the SLAM system
	virtual putslam::Mat34 correct(putslam::Mat34 pose) = 0;
};
#endif // _MOTIONMODEL_
