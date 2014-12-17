#include <iostream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <vector>

namespace DenseRGBDAligner {

  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;


  struct PositSolver{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // initializes the solver
    PositSolver();

    // initializes projects the points of the model in the 2D points dest, give the transform
    void project(Vector2fVector& dest);

    // computes the reprojection error and the jacobian of a model point
    // returns false if the point is outside the image plane
    bool errorAndJacobian(Eigen::Vector2f&  error, Matrix2_6f&  J, 
			  const Eigen::Vector3f& modelPoint, const Eigen::Vector2f& imagePoint);

    // computes the hessian and the coefficient vector of the model points between imin and imax
    // it also computes the error and the number of inliers
    void linearize(Matrix6f& H,Vector6f& b, 
		   float& error, int& inliers,
		   size_t imin=0, size_t imax=std::numeric_limits<size_t>::max());    

    // does one round of optimization
    // updates the transform T, the error and the numver of inliers
    void oneRound();

    Vector3fVector modelPoints;
    Vector2fVector imagePoints;

    Eigen::Isometry3f T; // position of the world w.r.t the camera
    Eigen::Matrix3f K;
    float imageRows, imageCols;
    float maxError;
    int inliers; 
    float error;
  };
}
