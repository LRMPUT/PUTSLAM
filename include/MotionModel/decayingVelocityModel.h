#ifndef _DECAYING_VELOCITY_MODEL_
#define _DECAYING_VELOCITY_MODEL_

#include "../include/MotionModel/motionModel.h"

class DecayingVelocityModel : public MotionModel
{
public:
	DecayingVelocityModel();

	// Motion model predicts new pose after dt time
	putslam::Mat34 predict(float dt);

	// Current pose is corrected from the SLAM system
	void correct(putslam::Mat34 pose);

private:


};

#endif // _DECAYING_VELOCITY_MODEL_
