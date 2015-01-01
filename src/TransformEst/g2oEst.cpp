#include "../include/TransformEst/g2oEst.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of G2O Estimator
G2OEst::Ptr g2oEst;

G2OEst::G2OEst(void) : name("G2O Estimator") {

}

const std::string& G2OEst::getName() const {
    return name;
}

/// compute transformation using two set of keypoints (information matrix -- identity)
Mat34& G2OEst::computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB){
    graph.clear();
    //add vertices - robot poses
    Vec3 pos(0.0, 0.0, 0.0);  Quaternion rot(1, 0, 0, 0);
    VertexSE3 vertex(0, pos, rot);
    if (!graph.addVertexPose(vertex))
        std::cout << "error: vertex exists!\n";
    Vec3 pos1(transformation(0,3), transformation(1,3), transformation(2,3));  Quaternion rot1(transformation.rotation());
    VertexSE3 vertex1(1, pos1, rot1);
    if (!graph.addVertexPose(vertex1))
        std::cout << "error: vertex exists!\n";

    int featureIdx=10;
    Mat33 unc; unc.setIdentity();
    for (int i=0;i<setA.rows();i++){
        Vec3 posFeature(setB(i,0), setB(i,1), setB(i,2));
        Vertex3D vertexFeature(featureIdx, posFeature);
        if (!graph.addVertexFeature(vertexFeature))
            std::cout << "error: vertex exists!\n";
//std::cout << "feature: \n" << setA(i,0) << ", " << setA(i,1) << ", " <<  setA(i,2) << "\n";
//std::cout << "unc: \n" << setAUncertainty[i] << "\n";
//std::cout << "feature: \n" << setB(i,0) << ", " << setB(i,1) << ", " <<  setB(i,2) << "\n";
//std::cout << "unc: \n" << setBUncertainty[i] << "\n";
        Vec3 pos11(setB(i,0), setB(i,1), setB(i,2));
        Mat33 info; info.setIdentity();
        Edge3D edge1(pos11,info, 0,featureIdx);

        if (!graph.addEdge3D(edge1))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos12(setA(i,0), setA(i,1), setA(i,2));

        Edge3D edge2(pos12,info, 1,featureIdx);
        if (!graph.addEdge3D(edge2))
            std::cout << "error: vertex doesn't exist!\n";

        featureIdx++;
    }

    graph.save2file("singleTr.g2o");
    graph.optimize(70);

    return transformation;
}

/// compute transformation using two set of keypoints
Mat34& G2OEst::computeTransformation(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation){
    graph.clear();
    //add vertices - robot poses
    Vec3 pos(0.0, 0.0, 0.0);  Quaternion rot(1, 0, 0, 0);
    VertexSE3 vertex(0, pos, rot);
    if (!graph.addVertexPose(vertex))
        std::cout << "error: vertex exists!\n";
    Vec3 pos1(transformation(0,3), transformation(1,3), transformation(2,3));  Quaternion rot1(transformation.rotation());
    VertexSE3 vertex1(1, pos1, rot1);
    if (!graph.addVertexPose(vertex1))
        std::cout << "error: vertex exists!\n";

    int featureIdx=10;
    Mat33 unc; unc.setIdentity();
    for (int i=0;i<setA.rows();i++){
        Vec3 posFeature(setB(i,0), setB(i,1), setB(i,2));
        Vertex3D vertexFeature(featureIdx, posFeature);
        if (!graph.addVertexFeature(vertexFeature))
            std::cout << "error: vertex exists!\n";
//std::cout << "feature: \n" << setA(i,0) << ", " << setA(i,1) << ", " <<  setA(i,2) << "\n";
//std::cout << "unc: \n" << setAUncertainty[i] << "\n";
//std::cout << "feature: \n" << setB(i,0) << ", " << setB(i,1) << ", " <<  setB(i,2) << "\n";
//std::cout << "unc: \n" << setBUncertainty[i] << "\n";
        Vec3 pos11(setB(i,0), setB(i,1), setB(i,2));
        Mat33 info = setBUncertainty[i].inverse();
        Edge3D edge1(pos11,info, 0,featureIdx);

        if (!graph.addEdge3D(edge1))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos12(setA(i,0), setA(i,1), setA(i,2));
        info = setAUncertainty[i].inverse();
        Edge3D edge2(pos12,info, 1,featureIdx);
        if (!graph.addEdge3D(edge2))
            std::cout << "error: vertex doesn't exist!\n";

        featureIdx++;
    }

//    graph.save2file("singleTr.g2o");
    graph.optimize(70);
    transformation = graph.getTransform(1);

    return transformation;
}

/// Compute uncertainty matrix [6x6] (fi,psi,theta,x,y,z)
const Mat66& G2OEst::computeUncertainty(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation) {

    graph.clear();
    //add vertices - robot poses
    Vec3 pos(0.0, 0.0, 0.0);  Quaternion rot(1, 0, 0, 0);
    VertexSE3 vertex(0, pos, rot);
    if (!graph.addVertexPose(vertex))
        std::cout << "error: vertex exists!\n";
    Vec3 pos1(transformation(0,3), transformation(1,3), transformation(2,3));  Quaternion rot1(transformation.rotation());
    VertexSE3 vertex1(1, pos1, rot1);
    if (!graph.addVertexPose(vertex1))
        std::cout << "error: vertex exists!\n";

    int featureIdx=10;
    Mat33 unc; unc.setIdentity();
    for (int i=0;i<setA.rows();i++){
        Vec3 posFeature(setB(i,0), setB(i,1), setB(i,2));
        Vertex3D vertexFeature(featureIdx, posFeature);
        if (!graph.addVertexFeature(vertexFeature))
            std::cout << "error: vertex exists!\n";
//std::cout << "feature: \n" << setA(i,0) << ", " << setA(i,1) << ", " <<  setA(i,2) << "\n";
//std::cout << "unc: \n" << setAUncertainty[i] << "\n";
//std::cout << "feature: \n" << setB(i,0) << ", " << setB(i,1) << ", " <<  setB(i,2) << "\n";
//std::cout << "unc: \n" << setBUncertainty[i] << "\n";
        Vec3 pos11(setB(i,0), setB(i,1), setB(i,2));
        Mat33 info = setBUncertainty[i].inverse();
        Edge3D edge1(pos11,info, 0,featureIdx);

        if (!graph.addEdge3D(edge1))
            std::cout << "error: vertex doesn't exist!\n";
        Vec3 pos12(setA(i,0), setA(i,1), setA(i,2));
        info = setAUncertainty[i].inverse();
        Edge3D edge2(pos12,info, 1,featureIdx);
        if (!graph.addEdge3D(edge2))
            std::cout << "error: vertex doesn't exist!\n";

        featureIdx++;
    }

//    graph.save2file("singleTr.g2o");
    graph.optimize(70);

    uncertainty = graph.getHessian(1);
    //std::cout << "kabsch est: \n";
    //std::cout << transformation(0,3) << " " << transformation(1,3) << " "  << transformation(2,3);
    //Quaternion qq(transformation.rotation());
    //std::cout << " " << qq.x() << " " << qq.y() << " "  << qq.z() << " " << qq.w() << "\n";
    //getchar();
    uncertainty = computeInformationMatrix(uncertainty,transformation).inverse();

    uncertainty = uncertainty.inverse();

    return uncertainty;
}

///computes information matrix from hessian using unscented transform
Mat66 G2OEst::computeInformationMatrix(const Mat66& Hessian, const Mat34& transformation){

    using namespace PSolver;
  typedef SigmaPoint<Vector6f> SigmaPoint;
  typedef std::vector<SigmaPoint, Eigen::aligned_allocator<SigmaPoint> > SigmaPointVector;

    Matrix6f H(Hessian.cast<float>());
  // invert the hessian to get the covariance matrix of the increments
  Eigen::JacobiSVD<Matrix6f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Matrix6f localSigma = svd.solve(Matrix6f::Identity());
  SigmaPointVector sigmaPoints;
  Vector6f localMean = Vector6f::Zero();

  // sanmple from the localSigma a set of sigma points
  sampleUnscented(sigmaPoints, localMean, localSigma);

  // apply each sigma point to the current transform to propagate the perturbation
  Eigen::Isometry3f _T(transformation.matrix().cast<float>());
  for (size_t i = 0; i < sigmaPoints.size(); i++) {
    SigmaPoint &p = sigmaPoints[i];
    p._sample = t2v( v2t(p._sample) * _T);
  }

  Vector6f mean = Vector6f::Zero();
  // Reconstruct the gaussian
  reconstructGaussian(mean, localSigma, sigmaPoints);

  // Compute the information matrix from the covariance
  Matrix6f informationMatrix;
  informationMatrix = localSigma.inverse();

  if (isNan(informationMatrix))
    informationMatrix.setZero();

  informationMatrix = .5* (informationMatrix + informationMatrix.transpose());
  return informationMatrix.cast<float_type>();
}

putslam::TransformEst* putslam::createG2OEstimator(void) {
    g2oEst.reset(new G2OEst());
    return g2oEst.get();
}
