#include "../../include/Solver/posit_solver.h"
#include <omp.h>
#include <iostream>

namespace DenseRGBDAligner {
  using namespace std;

  inline Eigen::Isometry3f v2t(const Vector6f& t){
    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      T.linear().setIdentity();
    }
    return T;
  }

  inline Eigen::Matrix3f skew(const Eigen::Vector3f& p){
    Eigen::Matrix3f s;
    s << 
      0,  -p.z(), p.y(),
      p.z(), 0,  -p.x(), 
      -p.y(), p.x(), 0;
    return s;
  }

  PositSolver::PositSolver(){
    T.setIdentity();
    K <<  
      525, 0, 319.5, 
      0, 525, 239.5, 
      0, 0, 1;
    imageCols=640;
    imageRows=480;
    maxError = 1000;
    error = 0;
    inliers = 0;
  }


  void PositSolver::project(Vector2fVector& dest){
    Eigen::Matrix3f KT = K*T.linear();
    Eigen::Vector3f Kt = K*T.translation();
    dest.resize(modelPoints.size());

#ifdef _GO_PARALLEL_
    #pragma omp parallel for
#endif
    for (size_t i = 0; i<modelPoints.size(); i++){
      const Eigen::Vector3f& p=modelPoints[i];
      Eigen::Vector3f pp=KT*p + Kt;
      if (pp.z()<0)
	pp << 0,0,-1;
      pp *= (1./pp.z());
      if (pp.x()<0 || pp.x()>imageCols ||
	  pp.y()<0 || pp.y()>imageRows)
	pp << 0,0,-1;
      dest[i].x()=pp.x();
      dest[i].y()=pp.y();
    }
  }


  bool PositSolver::errorAndJacobian(Eigen::Vector2f&  error, Matrix2_6f&  J, 
				     const Eigen::Vector3f& modelPoint, const Eigen::Vector2f& imagePoint){
    J.setZero();
    // apply the transform to the point
    Eigen::Vector3f tp=T*modelPoint;

    // if the points is behind the camera, drop it;
    if (tp.z()<0)
      return false;

    float z = tp.z();
    float iz=1./z;
    float iz2 = iz*iz;

    // apply the projection to the transformed point
    Eigen::Vector3f pp1 = K*tp;
    float iw = 1./pp1.z();
    Eigen::Vector2f pp (pp1.x()*iw, pp1.y()*iw);

    // if the projected point is outside the image, drop it
    if (pp.x()<0 || pp.x()>imageCols ||
	pp.y()<0 || pp.y()>imageRows)
      return false;

    // compute the error of given the projection
    error = pp - imagePoint;

    // jacobian of the transform part = [ I 2*skew(T*modelPoint) ]
    Eigen::Matrix<float, 3, 6> Jt;
    Jt.setZero();
    Jt.block<3,3>(0,0).setIdentity();
    Jt.block<3,3>(0,3)=-2*skew(tp);

    // jacobian of the homogeneous division
    // 1/z   0    -x/z^2
    // 0     1/z  -y/z^2
    Eigen::Matrix<float, 2, 3> Jp;
    Jp << 
      iz, 0,   -pp1.x()*iz2,
      0,   iz, -pp1.y()*iz2;

    // apply the chain rule and get the damn jacobian
    J=Jp*K*Jt;
    return true;
  }


  void PositSolver::linearize(Matrix6f& H,Vector6f& b, 
			      float& error, int& inliers,
			      size_t imin, size_t imax){
    
    imax = imax > modelPoints.size() ? modelPoints.size() : imax;
    H.setZero();
    b.setZero();
    error = 0;
    inliers = 0;
    Eigen::Vector2f e;
    Matrix2_6f J;
    for (size_t i = imin; i<imax; i++){
      if (errorAndJacobian(e,J,modelPoints[i], imagePoints[i])) {
	float en = e.squaredNorm();
	float scale = 1;
	if (en>maxError){
	  scale  = maxError/en;
	}
	error += en;
	H.noalias() += J.transpose()*J*scale;
	b.noalias() += J.transpose()*e*scale;
	inliers ++;
      }
    }
  }
    
  void PositSolver::oneRound(){
    Matrix6f H = Matrix6f::Zero();
    Vector6f b = Vector6f::Zero();
    error = 0;
    inliers = 0;

    int numThreads = omp_get_max_threads();
    size_t numPoints = modelPoints.size();
    int iterationsPerThread = numPoints / numThreads;

#ifdef _GO_PARALLEL_
#pragma omp parallel num_threads(numThreads)
    {

      int threadId = omp_get_thread_num();
      int imin = iterationsPerThread * threadId;
      int imax = imin + iterationsPerThread;

      Matrix6f tH; 
      Vector6f tb; 
      float terror; terror = 0;
      int tinliers; tinliers = 0;

      linearize(tH,tb,terror,tinliers,imin,imax);
#pragma omp critical 
      {
	H+=tH;
	b+=tb;
	inliers += tinliers;
	error +=terror;
      }
    }
#else
    linearize(H,b,error,inliers,0,modelPoints.size());
#endif

    //H += sqrt(chi2)*Matrix6f::Identity();
    //cerr << H << endl;
    // add damping?
    Vector6f dt = H.ldlt().solve(-b);
    T = v2t(dt)*T;
    Eigen::Matrix3f R = T.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    T.linear() -= 0.5 * R * E;
  }

}
