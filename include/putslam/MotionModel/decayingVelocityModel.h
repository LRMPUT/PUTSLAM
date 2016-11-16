#ifndef _DECAYING_VELOCITY_MODEL_
#define _DECAYING_VELOCITY_MODEL_

#include "../../include/putslam/MotionModel/motionModel.h"

class DecayingVelocityModel : public MotionModel
{
private:
	// Correct/Predict uncertainty
	Eigen::Matrix<double, 7, 7> R;
	Eigen::Matrix<double, 13, 13> Q;

	// State estimates (apriori and posteriori)
	// (x,y,z, vx,vy,vz, qx,qy,qz,qw, wx,wy,wz)
	Eigen::Matrix<double, 13, 1> x_apriori, x_aposteriori;

	// State estimates uncertainties (apriori and posteriori)
	Eigen::Matrix<double, 13, 13> P_apriori, P_aposteriori;
	Eigen::Matrix<double, 7, 13> H;

	// Additional values to detect estimation start and distinguish between predict/correct order
	bool firstMeasurement;

	// Decaying rate
	double decayingRate;

public:
	// TODO: NO IDEA WHAT ESTIMATES TO PUT (0.9 decaying rate is from PTAM)
	DecayingVelocityModel(double _Q = 1.0f, double _R = 1.0f, double _decayingRate = 0.9);

	// Motion model predicts new pose after dt time
	putslam::Mat34 predict(float dt);

	// Current pose is corrected from the SLAM system
	putslam::Mat34 correct(putslam::Mat34 poseIncrement);

private:
	// Predict based on last estimate
	Eigen::Matrix<double, 13, 1> statePrediction(float dt, Eigen::Matrix<double, 13, 13> &jacobian);

	// Extract estimate from stateVector
	putslam::Mat34 getCurrentEstimate(Eigen::Matrix<double, 13, 1> stateVector);

};

#endif // _DECAYING_VELOCITY_MODEL_
