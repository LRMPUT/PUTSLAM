#include "include/Solver/posit_solver.h"
#include "include/Solver/solver.h"
#include "include/Solver/cloud.h"
#include "include/Solver/correspondence_finder.h"
#include <chrono>
using namespace std;
using namespace DenseRGBDAligner;

int main(){
/*
  int numPoints = 30;//640*480;
  float cx=0, cy=0, cz=1;
  float xspread = 1, yspread = 1, zspread = 0.1;
  
  // sample randomly model points
  Vector3fVector modelPoints;
  modelPoints.resize(numPoints);
  for (size_t i = 0; i<numPoints; i++){
    float x = (drand48()-0.5)*xspread + cx;
    float y = (drand48()-0.5)*yspread + cy;
    float z = (drand48()-0.5)*zspread + cz;
    modelPoints[i] = Eigen::Vector3f(x,y,z);
  }
  
  // constructs a solver
  PositSolver solver;
  solver.maxError = 10;
  solver.modelPoints = modelPoints;

  // projects the model to image points
  Vector2fVector imagePoints;
  solver.project(imagePoints);
  solver.imagePoints = imagePoints;

  for (size_t i = 0; i<numPoints; i++){
    modelPoints[i](0)+=0.1;
    modelPoints[i](1)+=0.2;
    modelPoints[i](2)+=0.3;
  }
  solver.modelPoints = modelPoints;

  // sets a wrong initial guess to the solver and checks how it behaves
  solver.T.translation() << .3, .3, -.3;
  solver.T.linear()=Eigen::AngleAxisf(0.33*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  std::cout << " T: \n" << solver.T.matrix() << "\n";
//   for (size_t i = 0; i<imagePoints.size()
// ; i++){
//     Eigen::Vector2f ip = imagePoints[i];
//     cout << ip.x() << " " << ip.y() << endl;
//   }
  
  float e0, en;
  int   i0, in;
  Matrix6f Hessian;
  for (int i = 0; i< 1000; i++){
    solver.oneRound(Hessian);
    float e= solver.error;
    if (i==0){
      e0=solver.error;
      i0=solver.inliers;
    }
    cerr << ".";
  }
  cerr <<  endl;
  en=solver.error;
  in=solver.inliers;
  cerr << "Iteration 0.  Error: " << e0 << " inliers: " << i0 << " pixError: " << e0/i0 << endl;
  cerr << "Iteration n.  Error: " << en << " inliers: " << in << " pixError: " << en/in << endl;
  cerr << "Transform: " << endl << solver.T.matrix() << endl;
  cerr << "Hessian: " << endl << Hessian << endl;
  // put a certain number of points in the scene;
  // translate and project them
  
  // eliminate the points which are non visible
  
  
*/

    typedef std::chrono::high_resolution_clock myclock;
      myclock::time_point beginning = myclock::now();

  PSolver::Solver solver2;

  int pointsNo =1000;
  myclock::duration d = myclock::now() - beginning;
  unsigned seed2 = d.count();
  //std::mt19937 generator{1729};
  std::default_random_engine generator;
  generator.seed(seed2);
  std::uniform_real_distribution<double> distribution(-1.0,1.0);
  std::uniform_real_distribution<double> distributionNoise(-0.05,0.05);
  PSolver::Cloud reference;
  for (int i=0;i<pointsNo;i++){
      Eigen::Vector3f p;
      p(0) = distribution(generator);
      p(1) = distribution(generator);
      p(2) = 10.0;//distribution(generator);
      double length = sqrt(pow(p(0),2.0)+pow(p(1),2.0)+pow(p(2),2.0));
      Eigen::Vector3f norm(p(0)/length, p(1)/length, p(2)/length);
      PSolver::RichPoint point(p,norm);
      reference.push_back(point);
  }

  Eigen::Vector3f motion(0.0,-0.0,0.1);
  PSolver::Cloud current;
  Eigen::Matrix3f rot = Eigen::AngleAxisf(1*M_PI/180, Eigen::Vector3f::UnitY()).toRotationMatrix();
  std::cout << " rot: \n" << rot << "\n";
  for (int i=0;i<pointsNo;i++){
      Eigen::Vector3f p;
      p=rot*reference[i].point();
      p(0) = p(0)+motion(0);//+distributionNoise(generator);
      p(1) = p(1)+motion(1);
      p(2) = p(2)+motion(2);
      double length = sqrt(pow(p(0),2.0)+pow(p(1),2.0)+pow(p(2),2.0));
      Eigen::Vector3f norm(p(0)/length, p(1)/length, p(2)/length);
      PSolver::RichPoint point(p,norm);
      current.push_back(point);
  }

  solver2.setReferenceModel(&current);
  solver2.setCurrentModel(&reference);

  solver2.setMaxError(.001);
  solver2.setDamping(1);
  solver2.setGICP(true);

  Eigen::Isometry3f initGuess;
  initGuess.translation() << 0.01, -0.01, 0.11;
  initGuess.linear()=Eigen::AngleAxisf(1*M_PI/180, Eigen::Vector3f::UnitY()).toRotationMatrix();

    Matrix6f initial_guess_information = Matrix6f::Identity();
    solver2.setT(initGuess,initial_guess_information);

  Matrix6f info;
  info.setIdentity();
  info *=10;
  int iterations = 1000;
  PSolver::CorrespondenceFinder::CorrespondenceVector corres;

  for (int i=0;i<reference.size();i++){
      PSolver::CorrespondenceFinder::Correspondence pair;
      pair.first=i; pair.second=i;
      corres.push_back(pair);
  }

  const PSolver::CorrespondenceFinder::CorrespondenceVector& corr=corres;
  for (int i = 0; i< iterations; i++){
    solver2.oneRound(corr, true);
  }

  std::cout << "H: \n" << solver2.H() << "\n";
  std::cout << "info mat: \n" << solver2.informationMatrix() << "\n";
  std::cout << "info mat inv: \n" << solver2.informationMatrix().inverse() << "\n";
  std::cout << "T: \n" << solver2.T().matrix() << "\n";
  //std::cout << "T.inv: \n" << solver2.T().matrix().inverse() << "\n";
  std::cout << "e: \n" << solver2.error() << "\n";

  Eigen::Isometry3f trans;
  trans.matrix() = solver2.T().matrix().inverse();
    for (int i=0;i<pointsNo;i++){
         Eigen::Vector3f p;
         p(0) = reference[i].point()(0); p(1) = reference[i].point()(1); p(2) = reference[i].point()(2); p(3) = 1;
         Eigen::Vector3f pout = solver2.T().rotation()*p;
         pout(0)+= solver2.T()(0,3);
         pout(1)+= solver2.T()(1,3);
         pout(2)+= solver2.T()(2,3);
         std::cout << "(" << current[i].point()(0) << "," << current[i].point()(1) << "," << current[i].point()(2) << ") -> " << "(" << pout(0) << "," << pout(1) << "," << pout(2) << "), error: " << sqrt(pow(current[i].point()(0)-pout(0),2.0)+pow(current[i].point()(1)-pout(1),2.0)+pow(current[i].point()(2)-pout(2),2.0)) << "\n";
    }

}
