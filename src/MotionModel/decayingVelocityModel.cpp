#include "../../include/putslam/MotionModel/decayingVelocityModel.h"

DecayingVelocityModel::DecayingVelocityModel(double _Q, double _R, double _decayingRate) {
	this->Q = Eigen::Matrix<double, 13, 13>::Identity() * _Q;
	this->R = Eigen::Matrix<double, 7, 7>::Identity() * _R;
	this->decayingRate = _decayingRate;

	this->x_apriori.setZero();
	this->x_aposteriori.setZero();
	this->P_apriori = this->Q * 10000000000000.0;
	this->P_aposteriori = this->Q * 10000000000000.0;

	this->H.setZero();
	// X Y Z
	for (int i=0;i<3;i++)
		this->H(i, i) = 1.0;

	// Quat
	this->H(3, 6) = 1.0;
	this->H(4, 7) = 1.0;
	this->H(5, 8) = 1.0;
	this->H(6, 9) = 1.0;

	firstMeasurement = true;
};

// Motion model predicts new pose after dt time
putslam::Mat34 DecayingVelocityModel::predict(float dt) {

	Eigen::Matrix<double, 13, 13> F;
	this->x_apriori = this->statePrediction(dt, F);
	this->P_apriori = F * this->P_aposteriori * F.transpose() + this->Q;

	// Normalize
	double norm = this->x_apriori.block<4, 1>(6, 0).norm();
	this->x_apriori.block<4, 1>(6, 0) = this->x_apriori.block<4, 1>(6, 0) / norm;

	// Return current estimate
	return getCurrentEstimate(this->x_apriori);
}

// Current pose is corrected from the SLAM system
putslam::Mat34 DecayingVelocityModel::correct(putslam::Mat34 poseIncrement) {

	Eigen::Matrix<double, 7, 1> measurementVector =
			Eigen::Matrix<double, 7, 1>::Identity();

	// X Y Z and Quaternion
	for (int i = 0; i < 3; i++)
		measurementVector(i) = poseIncrement(i, 3);
	Eigen::Quaternion<double> Quaternion(
			poseIncrement.matrix().block<3, 3>(0, 0));
	measurementVector(3) = Quaternion.coeffs().x();
	measurementVector(4) = Quaternion.coeffs().y();
	measurementVector(5) = Quaternion.coeffs().z();
	measurementVector(6) = Quaternion.coeffs().w();

	// First measurement
	if (firstMeasurement) {
		firstMeasurement = false;
		for (int i = 0; i < 3; i++)
			this->x_apriori(i) = this->x_aposteriori(i) = measurementVector(i);
		for (int i = 6; i < 10; i++)
			this->x_apriori(i) = this->x_aposteriori(i) = measurementVector(i-3);

		this->P_aposteriori = this->P_aposteriori * 0.0;
		this->P_apriori = this->P_apriori * 0.0;
	}
	// Next measurement
	else
	{

//		std::cout<<"ESTIMATE : " << std::endl << getCurrentEstimate(this->x_apriori).matrix() << std::endl<<std::endl;

//		Eigen::Matrix<double,3,3> x = getCurrentEstimate(this->x_apriori).matrix().block<3,3>(0,0);
//		Eigen::Matrix<double,4,4> y = Eigen::Matrix<double,4,4>::Identity();
//		y.block<3,3>(0,0) = x;
//
//
//		poseIncrement = putslam::Mat34(y) * poseIncrement;
//		Eigen::Matrix<double, 6, 1> measurementVector = Eigen::Matrix<double, 6, 1>::Identity();
//		// X Y Z and Quaternion
//		for (int i=0;i<3;i++)
//			measurementVector(i) = poseIncrement(i,3);
//
//		measurementVector(3) = poseIncrement(2,1);
//		measurementVector(4) = poseIncrement(0,2);
//		measurementVector(5) = poseIncrement(1,0);
//		std::cout<<"POSE INCREMENT: " << std::endl << poseIncrement.matrix() << std::endl<< std::endl;


//		std::cout<<"STATE VECTOR apriori: " << this->x_apriori.transpose() << std::endl;

		// Some additional variables
		Eigen::Matrix<double, 13, 13> I = Eigen::Matrix<double, 13, 13>::Identity();
		Eigen::Matrix<double, 13, 7> K = Eigen::Matrix<double, 13, 7>::Zero();

		// EKF equations
		K =
				(this->P_apriori * this->H.transpose())
						* (this->H * this->P_apriori * this->H.transpose()
								+ this->R).inverse();

//		std::cout<<" K " << K.matrix() << std::endl;

//		std::cout<< " WTF: " << (measurementVector - this->H * this->x_apriori).matrix() << std::endl;

		this->x_aposteriori = this->x_apriori
				+ K * (measurementVector - this->H * this->x_apriori);
		this->P_aposteriori = (I - K * this->H) * this->P_apriori;

//		std::cout<<"STATE VECTOR aposteriori: " << this->x_aposteriori.transpose() << std::endl;

//		int a; std::cin>>a;
	}

	// Normalize quaternion
	double norm = this->x_aposteriori.block<4, 1>(6, 0).norm();
	this->x_aposteriori.block<4, 1>(6, 0) = this->x_aposteriori.block<4, 1>(6, 0) / norm;

	// Return current estimate
	return getCurrentEstimate(this->x_aposteriori);

}

Eigen::Matrix<double, 13, 1> DecayingVelocityModel::statePrediction(float dt, Eigen::Matrix<double, 13, 13> &jacobian) {
	Eigen::Matrix<double, 13, 13> A = Eigen::Matrix<double, 13, 13>::Identity();

	// X Y Z
	A(0, 3) = dt;
	A(1, 4) = dt;
	A(2, 5) = dt;

	//
	// Quaternion
	//
	double Wx = this->x_aposteriori(10);
	double Wy = this->x_aposteriori(11);
	double Wz = this->x_aposteriori(12);

	// 1st row
	A(6, 6) = 1.0;
	A(6, 7) = -0.5 * Wx * dt;
	A(6, 8) = -0.5 * Wy * dt;
	A(6, 9) = -0.5 * Wz * dt;

	// A 2nd row
	A(7, 6) = 0.5 * Wx * dt;
	A(7, 7) = 1;
	A(7, 8) = 0.5 * Wz * dt;
	A(7, 9) = -0.5 * Wy * dt;

	// A 3rd row
	A(8, 6) = 0.5 * Wy * dt;
	A(8, 7) = -0.5 * Wz * dt;
	A(8, 8) = 1;
	A(8, 9) = 0.5 * Wx * dt;

	// A 4th row
	A(9, 6) = 0.5 * Wz * dt;
	A(9, 7) = 0.5 * Wy * dt;
	A(9, 8) = -0.5 * Wx * dt;
	A(9, 9) = 1;

	// Already did fill a lot of values, so save jacobian (but not ready yet)
	jacobian = A;

	// Vx, Vy, Vz - decaying model
	for (int i = 3; i < 6; i++)
		A(i, i) = decayingRate;

	// Wx, Wy, Wz - decaying model
	for (int i = 10; i < 13; i++)
		A(i, i) = decayingRate;

	//
	// State transition is ready - now time to fill jacobian
	//
	Eigen::Matrix<double, 4, 1> Quaternion = this->x_aposteriori.block<4,1>(6, 0);

	// 1st row
	jacobian(6, 10) = 0.5 * dt * Quaternion(1);
	jacobian(6, 11) = 0.5 * dt * Quaternion(2);
	jacobian(6, 12) = 0.5 * dt * Quaternion(3);

	// 2nd row
	jacobian(7, 10) = -0.5 * dt * Quaternion(0);
	jacobian(7, 11) = 0.5 * dt * Quaternion(3);
	jacobian(7, 12) = -0.5 * dt * Quaternion(2);

	// 3rd row
	jacobian(8, 10) = -0.5 * dt * Quaternion(3);
	jacobian(8, 11) = -0.5 * dt * Quaternion(0);
	jacobian(8, 12) = 0.5 * dt * Quaternion(1);

	// 4th row
	jacobian(9, 10) = 0.5 * dt * Quaternion(2);
	jacobian(9, 11) = -0.5 * dt * Quaternion(1);
	jacobian(9, 12) = -0.5 * dt * Quaternion(0);

	// Doing the prediction
	return A * this->x_aposteriori;
}

putslam::Mat34 DecayingVelocityModel::getCurrentEstimate(Eigen::Matrix<double, 13, 1> stateVector)
{
		Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
	Eigen::Quaternion<double> quat(stateVector(9), stateVector(6), stateVector(7), stateVector(8));
	tmp(0, 3) = stateVector(0);
	tmp(1, 3) = stateVector(1);
	tmp(2, 3) = stateVector(2);
	tmp.block<3, 3>(0, 0) = quat.toRotationMatrix();

	return putslam::Mat34(tmp);
}
