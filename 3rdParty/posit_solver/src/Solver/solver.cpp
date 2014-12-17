#include "../../include/Solver/solver.h"
#include "../../include/Solver/utils.h"
#include <iostream>
#include <cassert>
#include <stdexcept>
#include "../../include/Solver/unscented.h"

#include <omp.h>
#ifdef _GO_PARALLEL_
#endif

#define _FAST_MULT_
//#define NAN_CHECK

namespace PSolver {

  Solver::Solver(){
    _H.setZero();
    _b.setZero();
    _T.setIdentity();
    _error = 0;
    _reference = 0;
    _current = 0;
    _use_only_points= false;
    _omega_tainted = true;

    _max_error = .01;

    _damping = 100;

    _flat_omega << 
      1, 0, 0,
      0, 1e-3, 0,
      0, 0, 1e-3;

 
    _long_linear_omega << 
      1e-1, 0, 0,
      0, 1, 0,
      0, 0, 1;

    _information_matrix.setZero();
    _prior_information.setZero();
    _prior_transform.setIdentity();
    _has_prior = false;
  }
  
  Solver::~Solver() {}


  void Solver::setReferenceModel(const Cloud* m) {
    _reference = m;
    _omega_tainted = true;
    _has_prior = false;
    _information_matrix.setZero();
  }
  

  void Solver::setCurrentModel(const Cloud* m) {
    _current = m;
    _has_prior = false;
    if (m){
      _errors.resize(m->size());
      std::fill(_errors.begin(), _errors.end(), -1.0f);
    }
    _information_matrix.setZero();
  }

  void Solver::errorAndJacobian(Eigen::Vector3f&  pointError, 
				       Eigen::Vector3f&  normalError, 
				       Matrix6f&  J, 
				       const Eigen::Vector3f& referencePoint, 
				       const Eigen::Vector3f& referenceNormal, 
				       const Eigen::Vector3f& currentPoint,
				       const Eigen::Vector3f& currentNormal) const {
    J.setZero();
    // apply the transform to the point
    Eigen::Vector3f tp=_T*currentPoint;
    Eigen::Vector3f tn=_T.linear()*currentNormal;

    pointError=tp-referencePoint;
    normalError=tn-referenceNormal;
    J.block<3,3>(0,0).setIdentity();  J.block<3,3>(0,3)=-2*skew(tp);
                                      J.block<3,3>(3,3)=-2*skew(tn);
  }

  void Solver::errorAndJacobianPoint(Eigen::Vector3f&  pointError, 
						  Matrix3_6f&  J, 
						  const Eigen::Vector3f& referencePoint, 
						  const Eigen::Vector3f& currentPoint) const {
    J.setZero();
    // apply the transform to the point
    Eigen::Vector3f tp=_T*currentPoint;
    pointError=tp-referencePoint;
    // jacobian of the transform part = [ I 2*skew(T*referencePoint) ]
    J.block<3,3>(0,0).setIdentity();  J.block<3,3>(0,3)=-2*skew(tp);
  }


    void Solver::setReferencePointsHint(const std::vector<int>& referencePointsHint) {
      computeOmegas(referencePointsHint);
    }


  void Solver::computeOmegas(const std::vector<int> updateList) { 
    size_t numOmegas;

    _omega_points.resize(_reference->size());
    _omega_normals.resize(_reference->size());

    bool compactCompute = false;
    if (updateList.size()) {
      numOmegas = updateList.size();
      compactCompute = true;
    } else {
      numOmegas = _reference->size();
    }

#ifdef _GO_PARALLEL_
    #pragma omp parallel for
#endif

    for (size_t _i = 0; _i<numOmegas; _i++){
      int idx = compactCompute ? updateList[_i] : _i;
      const Eigen::Vector3f& referencePoint = (*_reference)[idx].point();
      const Eigen::Vector3f& referenceNormal = (*_reference)[idx].normal();
      
      Eigen::Matrix3f& omegap = _omega_points[idx];
      Eigen::Matrix3f& omegan = _omega_normals[idx];
      
      omegap.setZero();
      // if the point has a normal
      if (referenceNormal.squaredNorm()>0) {
	// compute the rotation matrix that aligns the z axis to the normal direction
	Eigen::Vector3f axis(0, -referenceNormal.z(), referenceNormal.y());
	float s = axis.norm();
	float c = referenceNormal.x();
	float angle = atan2(s,c);
	Eigen::AngleAxisf aa(angle, axis);
	Eigen::Matrix3f Rn=aa.toRotationMatrix();
	omegap+=Rn*_flat_omega*Rn.transpose();
	omegan=Rn*_long_linear_omega*Rn.transpose();
      } else {
	omegan.setZero();
      }
       
      float depthScaling = 1/(1+fabs(referencePoint.z()));
      omegap *= depthScaling;
      omegan *= depthScaling;

#ifdef NAN_CHECK
      if (isNan(omegap) || isNan(omegan)) {
	cerr << endl;
	cerr << " point index: " << idx << endl;
	cerr << "p: " << endl << referencePoint.transpose() << endl;
	cerr << "Omegap" << endl << omegap << endl;
	cerr << "n: " << endl << referenceNormal.transpose() << endl;
	cerr << "Omegan" << endl << omegan << endl;
	throw std::runtime_error("NAN detected");
      }
#endif

 
    }
    _omega_tainted = false;
  }


  void Solver::linearize(Matrix6f& H,Vector6f& b, 
				      float& error, 
                      const CorrespondenceVector& correspondences,
				      size_t imin, size_t imax){

    imax = imax > correspondences.size() ? correspondences.size() : imax;
    error = 0;
    Eigen::Vector3f ep;
    Eigen::Vector3f en;
    Eigen::Vector3f et;


    for (size_t _i = imin; _i<imax; _i++){
      int ridx=correspondences[_i].first;
      int cidx=correspondences[_i].second;
      if (ridx<0 || cidx <0)
    continue;
      _errors[cidx]=-1;

      const Eigen::Vector3f& currentPoint = (*_current)[cidx].point();
      const Eigen::Vector3f& referencePoint = (*_reference)[ridx].point();
      const Eigen::Matrix3f& omegap = _omega_points[ridx];

      if(_use_only_points) {
	Matrix3_6f J;
	Eigen::Matrix3f Omega = omegap;
	errorAndJacobianPoint(ep, J, referencePoint, currentPoint);
	error = ep.transpose()*Omega*ep;
	float scale = 1;
	_errors[cidx]=error;
	if (error>_max_error){
	  _errors[cidx] = -error;
	  scale  = _max_error/error;
	} 
	Omega*=scale;
	H.noalias()+=J.transpose()*Omega*J;
	b.noalias()+=J.transpose()*Omega*ep;

#ifdef NAN_CHECK
	Matrix6f Hi=J.transpose()*Omega*J;
	//Vector6f bi=J.transpose()*Omega*ep;
	if (isNan(Hi)) {
	  cerr << endl;
	  cerr << Hi;
	  cerr << "J" << endl << J << endl;
	  cerr << "Omega" << endl << Omega << endl;
	  cerr << "ep" << endl << ep << endl;
	  throw std::runtime_error("NAN detected");
	}
#endif


      }

      }

  }
  
  void Solver::linearize(const CorrespondenceVector& correspondences){
    if (_omega_tainted)
      computeOmegas();
    _H.setZero();
    _b.setZero();
    _error = 0;
    size_t numPoints = correspondences.size();
#ifdef _GO_PARALLEL1_
    int numThreads = omp_get_max_threads();
    int pointsPerThread = numPoints / numThreads;
    Matrix6f tH[numThreads]; 
    Vector6f tb[numThreads]; 
    float terror[numThreads];

#pragma omp parallel num_threads(numThreads)
    {

      int threadId = omp_get_thread_num();
      int imin = pointsPerThread * threadId;
      int imax = imin + pointsPerThread;

      terror[threadId] = 0;
      linearize(tH[threadId],
		tb[threadId],
        terror[threadId], correspondences, imin,imax);
    }
    for (int threadId=0; threadId<numThreads; threadId++){
      _H+=tH[threadId];
      _b+=tb[threadId];
      _error+=terror[threadId];
    }

#else // other branch of _GO_PARALLEL_

    linearize(_H,_b,_error,correspondences, 0,numPoints);
    /*std::cout << "H" << _H << "\n";
    std::cout << "b" << _b << "\n";
    std::cout << "error" << _error << "\n";
    std::cout << "numPoints" << numPoints << "\n";*/
#endif
  }


  void Solver::computeInformationMatrix() {
    typedef SigmaPoint<Vector6f> SigmaPoint;
    typedef std::vector<SigmaPoint, Eigen::aligned_allocator<SigmaPoint> > SigmaPointVector;

    // invert the hessian to get the covariance matrix of the increments
    Eigen::JacobiSVD<Matrix6f> svd(_H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Matrix6f localSigma = svd.solve(Matrix6f::Identity());
    SigmaPointVector sigmaPoints;
    Vector6f localMean = Vector6f::Zero();

    // sanmple from the localSigma a set of sigma points
    sampleUnscented(sigmaPoints, localMean, localSigma);
  
    // apply each sigma point to the current transform to propagate the perturbation
    for (size_t i = 0; i < sigmaPoints.size(); i++) {
      SigmaPoint &p = sigmaPoints[i];
      p._sample = t2v( v2t(p._sample) * _T);
    }

    Vector6f mean = Vector6f::Zero();
    // Reconstruct the gaussian 
    reconstructGaussian(mean, localSigma, sigmaPoints);

    // Compute the information matrix from the covariance
    _information_matrix = localSigma.inverse();

    if (isNan(_information_matrix))
      _information_matrix.setZero();

    _information_matrix = .5* (_information_matrix + _information_matrix.transpose());
  }


    
  void Solver::oneRound(const CorrespondenceVector& correspondences, bool computeStats){
    _information_matrix.setZero();
    linearize(correspondences);
    Matrix6f H = _H;
    Vector6f b = _b;

    if (_has_prior) {
      Matrix6f Jp = priorJacobian();
      Vector6f ep = priorError();
      H+=Jp.transpose()*_prior_information*Jp;
      b+=Jp.transpose()*_prior_information*ep;
    }

    H += _damping*Matrix6f::Identity();

    Vector6f dt = H.ldlt().solve(-b);
    _T = v2t(dt)*_T;
    Eigen::Matrix3f R = _T.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _T.linear() -= 0.5 * R * E;
    
    if (computeStats)
      computeInformationMatrix();
  }



  Vector6f Solver::priorError(const Vector6f& dt) {
    Eigen::Isometry3f dT=v2t(dt);
    return t2v(_inverse_prior_transform*dT*_T);
  }
  
  Matrix6f Solver::priorJacobian() {
    Matrix6f J;
    J.setZero();
    float epsilon = 1e-3;
    for (int i = 0; i<6; i++){
      Vector6f d_plus, d_minus;
      d_plus.setZero();
      d_minus.setZero();
      d_plus[i]=epsilon;
      d_minus[i]=-epsilon;
      J.col(i)=priorError(d_plus)-priorError(d_minus);
    }
    J*=(.5/epsilon);
    return J;
  }

}
