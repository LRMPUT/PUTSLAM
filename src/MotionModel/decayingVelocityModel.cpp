#include "../include/MotionModel/decayingVelocityModel.h"

DecayingVelocityModel::DecayingVelocityModel() { };

// Motion model predicts new pose after dt time
putslam::Mat34 DecayingVelocityModel::predict(float dt) {

	putslam::Mat34 estimate;
	return estimate;
}

// Current pose is corrected from the SLAM system
void DecayingVelocityModel::correct(putslam::Mat34 pose) {

}
