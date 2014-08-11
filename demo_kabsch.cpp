#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Tracker/trackerKLT.h"
#include "TransformEst/kabschEst.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include "Grabber/kinect_grabber.h"
#include "PoseGraph/graph_g2o.h"
#include <cmath>
#include <Eigen/Dense>

using namespace std;

auto startT = std::chrono::high_resolution_clock::now();
std::default_random_engine generator;

/// noise: x, y, z, qx, qy, qz
float_type noise[6] = {0.0, 0.0, 0.0, 0.0, 0.000, 0.000};
/// x, y, z, fi, psi, theta
float_type transformation[6] = {0.1, 0.2, -0.3, 0.1, 0.2, -0.3};

Graph * graph;

// optimization and pruning thread
void optimizeAndPrune(){
    // graph pruning and optimization
    graph->optimizeAndPrune(2, 70);
}

Point3D sampleFromMultivariateGaussian(Eigen::Vector3d mean, Eigen::MatrixXd cov){

    Eigen::MatrixXd normTransform(mean.rows(),mean.rows());
    Eigen::LLT<Eigen::MatrixXd> cholSolver(cov);
    // We can only use the cholesky decomposition if
    // the covariance matrix is symmetric, pos-definite.
    // But a covariance matrix might be pos-semi-definite.
    // In that case, we'll go to an EigenSolver
    if (cholSolver.info()==Eigen::Success) {
      // Use cholesky solver
      normTransform = cholSolver.matrixL();
    } else {
      // Use eigen solver
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(cov);
      normTransform = eigenSolver.eigenvectors()
                     * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }
    std::normal_distribution<double> normDistribution(0.0,0.5);
    Eigen::Vector3d sampleGauss;
    for (int i=0;i<3;i++)
        sampleGauss(i) = normDistribution(generator);
    Eigen::Vector3d sample = (normTransform*sampleGauss) + mean;

    Point3D point;
    point.x = sample(0); point.y = sample(1); point.z = sample(2);

    return point;
}

Eigen::Quaternion<double> quatFromEuler(float_type fi, float_type psi, float_type theta){
    Eigen::Quaternion<double> quat;
    Eigen::AngleAxis<double> rotX(fi, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxis<double> rotY(psi, Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> rotZ(theta, Eigen::Vector3d::UnitX());
    quat = rotX * rotY * rotZ;
    return quat;
}

void printUncertainty(Mat66& uncertainty, Mat34& trans, float_type x, float_type y, float_type z, float_type fi, float_type psi, float_type theta){
    std::cout << "uncertainty: \n" << uncertainty << std::endl;
    std::cout << "error x: " << fabs(trans.translation()(0)-x) << ", 3*sigma = " << 3*uncertainty(3,3);
    if (3*uncertainty(3,3) < fabs(trans.translation()(0)-x) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error y: " << fabs(trans.translation()(1)-y) << ", 3*sigma = " << 3*uncertainty(4,4);
    if (3*uncertainty(4,4) < fabs(trans.translation()(1)-y) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error z: " << fabs(trans.translation()(2)-z) << ", 3*sigma = " << 3*uncertainty(5,5);
    if (3*uncertainty(5,5) < fabs(trans.translation()(2)-z) ) std::cout << " alert!\n"; else std::cout << "\n";
    double fiRef = atan2(trans.matrix()(1,0), trans.matrix()(0,0));
    double psiRef = -asin(trans.matrix()(2,0));
    double thetaRef = atan2(trans.matrix()(2,1), trans.matrix()(2,2));
    std::cout << "error fi: " << fabs(fiRef-fi) << ", 3*sigma = " << 3*uncertainty(0,0);
    if (3*uncertainty(0,0) < fabs(fiRef-fi) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error psi: " << fabs(psiRef-psi) << ", 3*sigma = " << 3*uncertainty(1,1);
    if (3*uncertainty(1,1) < fabs(psiRef-psi) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error theta: " << fabs(thetaRef-theta) << ", 3*sigma = " << 3*uncertainty(2,2);
    if (3*uncertainty(2,2) < fabs(thetaRef-theta) ) std::cout << " alert!\n"; else std::cout << "\n";
}

void save2file(std::string filename, const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB, const Eigen::MatrixXd& setTransformed){
    std::ofstream file(filename);
    file << "close all; clear all;\n";
    file << "hold on;\n";
    for (int i=0;i<setA.rows();i++){
        file << "plot3(" << setA(i,0) << ", " << setA(i,1) << ", " << setA(i,2) << ",'r.' );\n";
    }
    for (int i=0;i<setB.rows();i++){
        file << "plot3(" << setB(i,0) << ", " << setB(i,1) << ", " << setB(i,2) << ",'go' );\n";
    }
    for (int i=0;i<setTransformed.rows();i++){
        file << "plot3(" << setTransformed(i,0) << ", " << setTransformed(i,1) << ", " << setTransformed(i,2) << ",'bx' );\n";
    }
    file.close();
}

/// Draw coordinate system
void plotCoordinates(std::ofstream& file, Mat34 pose) {
    file << "plot3([" << pose.matrix()(0,3) << ", " << pose.matrix()(0,3)+pose.matrix()(0,0)*0.1 << "], [" << pose.matrix()(1,3) << ", " << pose.matrix()(1,3)+pose.matrix()(1,0)*0.1 << "], [" << pose.matrix()(2,3) << ", " << pose.matrix()(2,3)+pose.matrix()(2,0)*0.1 << "], 'r', 'LineWidth',1); hold on\n";
    file << "plot3([" << pose.matrix()(0,3) << ", " << pose.matrix()(0,3)+pose.matrix()(0,1)*0.1 << "], [" << pose.matrix()(1,3) << ", " << pose.matrix()(1,3)+pose.matrix()(1,1)*0.1 << "], [" << pose.matrix()(2,3) << ", " << pose.matrix()(2,3)+pose.matrix()(2,1)*0.1 << "], 'g', 'LineWidth',1); hold on\n";
    file << "plot3([" << pose.matrix()(0,3) << ", " << pose.matrix()(0,3)+pose.matrix()(0,2)*0.1 << "], [" << pose.matrix()(1,3) << ", " << pose.matrix()(1,3)+pose.matrix()(1,2)*0.1 << "], [" << pose.matrix()(2,3) << ", " << pose.matrix()(2,3)+pose.matrix()(2,2)*0.1 << "], 'b', 'LineWidth',1); hold on\n";
}

void saveTrajectory(std::string filename, std::vector<Mat34> trajectory, std::string color){
    std::ofstream file(filename);
    //file << "close all;";
    file << "clear all;\n";
    file << "body_x=[";
    for (std::vector<Mat34>::iterator it = trajectory.begin(); it!=trajectory.end(); it++){
        file << (*it)(0,3) << ",";
    }
    file << "];\n";
    file << "body_y=[";
    for (std::vector<Mat34>::iterator it = trajectory.begin(); it!=trajectory.end(); it++){
        file << (*it)(1,3) << ",";
    }
    file << "];\n";
    file << "body_z=[";
    for (std::vector<Mat34>::iterator it = trajectory.begin(); it!=trajectory.end(); it++){
        file << (*it)(2,3) << ",";
    }
    file << "];\n";
    file << "plot3(body_x, body_y, body_z, '"<< color << "', 'LineWidth',3); hold on\n";
    /*for (size_t i = 0; i<trajectory.size();i++){
        plotCoordinates(file, trajectory[i]);
    }*/
    file << "xlabel('x');\n"; file << "ylabel('y');\n"; file << "zlabel('z');\n";
}

void generateSetpoint(Eigen::MatrixXd& setA, size_t numPoints){
    float_type center[3]={0.0,0.0,0.0};
    std::uniform_real_distribution<double> distribution(-1.5,1.5);
    for (size_t i = 0; i<numPoints; i++){
        setA(i,0) = center[0] + distribution(generator);
        setA(i,1) = center[1] + distribution(generator);
        setA(i,2) = center[2] + distribution(generator);
    }
}

void savePointCloud(std::string filename, PointCloud& cloud){
    std::ofstream file(filename);
    file << "close all; clear all;\n";
    file << "hold on;\n";
    for (int i=0;i<cloud.size();i++){
        file << "plot3(" << cloud[i].x << ", " << cloud[i].y << ", " << cloud[i].z << ",'r.' );\n";
    }
    file.close();
}

void savePointCloud(std::string filename, PointCloud& cloud, std::vector<Mat33>& uncertainty) {
    std::ofstream file(filename);
    file << "close all; clear all;\n";
    file << "hold on;\n";
    for (int i=0;i<cloud.size();i++){
        file << "plot3(" << cloud[i].x << ", " << cloud[i].y << ", " << cloud[i].z << ",'r.' );\n";
        //file << "plot3([" << cloud[i].x - 3*uncertainty[i](0,0)<< "," << cloud[i].x + 3*uncertainty[i](0,0)<< "],[" << cloud[i].y << ", " << cloud[i].y << "],[" << cloud[i].z << ", " << cloud[i].z << "],'-k');\n";
        //file << "plot3([" << cloud[i].x << "," << cloud[i].x << "],[" << cloud[i].y - 3*uncertainty[i](1,1)<< ", " << cloud[i].y + 3*uncertainty[i](1,1) << "],[" << cloud[i].z - 3*uncertainty[i](2,2) << ", " << cloud[i].z + 3*uncertainty[i](2,2) << "],'-k');\n";
        //file << "plot3([" << cloud[i].x << "," << cloud[i].x << "],[" << cloud[i].y << ", " << cloud[i].y << "],[" << cloud[i].z - 3*uncertainty[i](2,2) << ", " << cloud[i].z + 3*uncertainty[i](2,2) << "],'-k');\n";
        file << "C = [" << uncertainty[i](0,0) << ", " << uncertainty[i](0,1) << ", "<< uncertainty[i](0,2) << "; " << uncertainty[i](1,0) << ", " << uncertainty[i](1,1) << ", "<< uncertainty[i](1,2) << "; "<< uncertainty[i](2,0) << ", " << uncertainty[i](2,1) << ", "<< uncertainty[i](2,2) << "];\n";
        file << "M = [" << cloud[i].x << ";" << cloud[i].y << "; " << cloud[i].z << "];\n";
        file << "error_ellipse(C, M);\n";
    }
    file << "xlabel('x'); ylabel('y'); zlabel('z');\n";
    file.close();
}

std::vector<int> getCloud(const Mat34& sensorPose, KinectGrabber::UncertaintyModel& sensorModel, const PointCloud& room, PointCloud& setPoints, std::vector<Mat33>& setUncertainty){
    std::vector<int> pointIdentifiers;
    setPoints.clear();
    for (size_t i=0;i<room.size();i++){
        Eigen::Vector4d point(room[i].x, room[i].y, room[i].z, 1);
        Eigen::Vector4d pointCamera = sensorPose.matrix().inverse()*point;
        Eigen::Vector3d point2d = sensorModel.inverseModel(pointCamera(0), pointCamera(1), pointCamera(2));
        if (point2d(0)!=-1){
            Mat33 uncertainty;
            sensorModel.computeCov(point2d(0), point2d(1), point2d(2), uncertainty);
            Point3D point = sampleFromMultivariateGaussian(Eigen::Vector3d(pointCamera(0), pointCamera(1), pointCamera(2)),uncertainty);
            setPoints.push_back(point);
            setUncertainty.push_back(uncertainty);
            pointIdentifiers.push_back(i);
        }
    }
    return pointIdentifiers;
}

void matchClouds(const PointCloud& setAin, Eigen::MatrixXd& setAout, const std::vector<Mat33>& uncertaintyAin, std::vector<Mat33>& uncertaintyAout, const std::vector<int>& setAids, const PointCloud& setBin, Eigen::MatrixXd& setBout, const std::vector<Mat33>& uncertaintyBin, std::vector<Mat33>& uncertaintyBout, const std::vector<int>& setBids){
    int matchesNo=0;
    for (int i=0;i<setAin.size();i++){
        if (std::find(setBids.begin(),setBids.end(),setAids[i])!=setBids.end())
            matchesNo++;
    }
    setAout.resize(matchesNo,3); setBout.resize(matchesNo,3);
    uncertaintyAout.clear(); uncertaintyBout.clear();
    matchesNo=0;
    for (int i=0;i<setAin.size();i++){
        std:vector<int>::const_iterator itB = std::find(setBids.begin(),setBids.end(),setAids[i]);
        if (itB!=setBids.end()){
            setAout(matchesNo,0) = setAin[i].x; setAout(matchesNo,1) = setAin[i].y; setAout(matchesNo,2) = setAin[i].z;
            uncertaintyAout.push_back(uncertaintyAin[i]);
            setBout(matchesNo,0) = setBin[itB-setBids.begin()].x; setBout(matchesNo,1) = setBin[itB-setBids.begin()].y; setBout(matchesNo,2) = setBin[itB-setBids.begin()].z;
            uncertaintyBout.push_back(uncertaintyBin[itB-setBids.begin()]);
            matchesNo++;
        }
    }
}

PointCloud createRoom(size_t pointsNo, float_type width, float_type length, float_type height){
    PointCloud room;
    std::uniform_real_distribution<double> distributionWidth(-width/2.0, width/2.0);
    std::uniform_real_distribution<double> distributionLength(-length/2.0, length/2.0);
    std::uniform_real_distribution<double> distributionHeight(0, height);
    for (size_t i = 0; i<pointsNo/6;i++){
        Point3D point;
        //floor
        point.x = distributionWidth(generator); point.y = distributionLength(generator); point.z = 0;
        room.push_back(point);
        //ceiling
        point.x = distributionWidth(generator); point.y = distributionLength(generator); point.z = height;
        room.push_back(point);
        //wall 1
        point.x = -width/2.0; point.y = distributionLength(generator); point.z = distributionHeight(generator);
        room.push_back(point);
        //wall 2
        point.x = width/2.0; point.y = distributionLength(generator); point.z = distributionHeight(generator);
        room.push_back(point);
        //wall 3
        point.x = distributionWidth(generator); point.y = length/2.0; point.z = distributionHeight(generator);
        room.push_back(point);
        //wall 4
        point.x = distributionWidth(generator); point.y = -length/2.0; point.z = distributionHeight(generator);
        room.push_back(point);
    }
    return room;
}

int main(int argc, char * argv[])
{
    try {
        using namespace putslam;
        using namespace std::chrono;

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());

        // create kabsch transform estimator
        TransformEst* transEst = createKabschEstimator();

        size_t numPoints = 100;
        Eigen::MatrixXd setA(numPoints, 3);
        Eigen::MatrixXd setB(numPoints, 3);

        generator.seed((unsigned int)time(0));
        generateSetpoint(setA, numPoints);

        std::cout << "Mean transformation is: x=" << transformation[0] << ", y=" << transformation[1] << ", z=" << transformation[2];
        std::cout << ", fi=" << transformation[3] << ", psi=" << transformation[4] << ", theta=" << transformation[5];
        Eigen::Quaternion<double> quat = quatFromEuler(transformation[3],transformation[4], transformation[5]);
        std::cout << ", qw=" << quat.w() << ", qx=" << quat.x() << ", qy=" << quat.y() << ", qz=" << quat.z() << "\n";

        std::normal_distribution<double> normDistributionX(0.0,noise[0]);
        std::normal_distribution<double> normDistributionY(0.0,noise[1]);
        std::normal_distribution<double> normDistributionZ(0.0,noise[2]);
        std::normal_distribution<double> normDistributionQX(0.0,noise[3]);
        std::normal_distribution<double> normDistributionQY(0.0,noise[4]);
        std::normal_distribution<double> normDistributionQZ(0.0,noise[5]);

        std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
        for (size_t i = 0; i<numPoints; i++){
            Eigen::Quaternion<double> q = quatFromEuler(transformation[3], transformation[4], transformation[5]);
            /*q.x() += normDistributionQX(generator);
            q.y() += normDistributionQY(generator);
            q.y() += normDistributionQZ(generator);
            q.normalize();*/

            Eigen::Transform< double, 3, Eigen::Affine > transform = q * Eigen::Translation<double,3>(0,0,0);
            Vec3 point(setA(i,0), setA(i,1), setA(i,2));
            point.vector() = transform*point.vector();
            setB(i,0) = point.x()+normDistributionX(generator)+transformation[0]; setB(i,1) = point.y()+normDistributionY(generator)+transformation[1]; setB(i,2) = point.z()+normDistributionZ(generator)+transformation[2];

            Mat33 uncertainty; uncertainty.setIdentity();
            uncertainty(0,0) = noise[0]; uncertainty(1,1) = noise[1]; uncertainty(2,2) = noise[2];
            setAUncertainty.push_back(uncertainty);
            setBUncertainty.push_back(uncertainty);
        }
        Mat34 trans = transEst->computeTransformation(setA,setB);
        Quaternion quatRes(trans.rotation());
        std::cout << "Kabsch transformation: x = " << trans.translation()(0) << ", y = " << trans.translation()(1)  << ", z = " << trans.translation()(2) << ", qw=" << quatRes.w() << ", qx=" << quatRes.x() << ", qy=" << quatRes.y() << ", qz=" << quatRes.z() << "\n";;
        double fi = atan2(trans.matrix()(1,0), trans.matrix()(0,0));
        double psi = -asin(trans.matrix()(2,0));
        double theta = atan2(trans.matrix()(2,1), trans.matrix()(2,2));
        std::cout << "euler: " << "fi: " << fi << ", psi: " << psi << ",theta: " << theta <<"\n";

        Mat66 uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
        printUncertainty(uncertainty, trans, transformation[0], transformation[1], transformation[2], transformation[3], transformation[4], transformation[5]);

        Eigen::MatrixXd setTransformed(numPoints, 3);
        for (int i=0;i<setA.rows();i++){
            Vec3 point(setA(i,0), setA(i,1), setA(i,2));
            point.vector() = trans*point.vector();
            setTransformed(i,0) = point.x(); setTransformed(i,1) = point.y(); setTransformed(i,2) = point.z();
        }

        uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
        std::cout << "uncertainty x y z qx qy qz: \n" << uncertainty << std::endl;
        save2file("../../resources/kabsch.m",setA, setB, setTransformed);

        std::cout << "\n\n\nSingle transformation: room test\n";
        PointCloud room;
        size_t pointsNo = 1000;
        float_type roomDim[3] = {7, 7, 3};
        room = createRoom(pointsNo, roomDim[0], roomDim[1], roomDim[2]);
        savePointCloud("../../resources/room.m", room);

        KinectGrabber::UncertaintyModel sensorModel(configFile);
        Mat34 initPose = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>(0,0,roomDim[2]/2.0);
        initPose.matrix() *= sensorModel.config.pose.matrix();
        PointCloud cloudA; std::vector<Mat33> uncertaintyCloudA;
        std::vector<int> setAids = getCloud(initPose, sensorModel, room, cloudA, uncertaintyCloudA);
        savePointCloud("../../resources/cloudA.m", cloudA, uncertaintyCloudA);

        float_type moveTab[6] = {0.0,0.1,0.0, 0.0,0.0,0.0};//motion in global frame
        Eigen::Quaternion<double> quatMove = quatFromEuler(moveTab[3], moveTab[4], moveTab[5]);
        Mat34 move = quatMove*Eigen::Translation<double,3>(0,0,0);
        PointCloud cloudB; std::vector<Mat33> uncertaintyCloudB;
        std::cout <<"init pose: \n" << initPose.matrix() << "\n";
        Mat34 nextPose = initPose*move;
        nextPose(0,3)+=moveTab[0]; nextPose(1,3)+=moveTab[1]; nextPose(2,3)+=moveTab[2];
        std::cout <<"next pose: \n" << nextPose.matrix() << "\n";
        std::vector<int> setBids = getCloud(nextPose, sensorModel, room, cloudB, uncertaintyCloudB);
        savePointCloud("../../resources/cloudB.m", cloudB, uncertaintyCloudB);

        matchClouds(cloudA, setA, uncertaintyCloudA, setAUncertainty, setAids, cloudB, setB, uncertaintyCloudB, setBUncertainty, setBids);
        trans = transEst->computeTransformation(setB, setA);
        std::cout << "translation in sensor frame: \n" << trans.matrix() << "\n";
        std::cout << "setAuncertainy: " << setAUncertainty[0](0,0) << ", " << setAUncertainty[0](1,1) << ", " << setAUncertainty[0](2,2) << "\n";
        uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);

        Mat66 uncertaintySensor = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);

        Mat34 translation = sensorModel.config.pose*trans;
        trans(0,3) = translation(0,3); trans(1,3) = translation(1,3); trans(2,3) = translation(2,3);
        std::cout << "computed transformation: \n" << trans.matrix() << std::endl;
        fi = atan2(trans.matrix()(1,0), trans.matrix()(0,0));
        psi = -asin(trans.matrix()(2,0));
        theta = atan2(trans.matrix()(2,1), trans.matrix()(2,2));
        std::cout << "euler: " << "fi: " << fi << ", psi: " << psi << ",theta: " << theta <<"\n";
        printUncertainty(uncertainty, trans, moveTab[0], moveTab[1], moveTab[2], moveTab[3], moveTab[4], moveTab[5]);

        std::cout << "uncertainty in sensor frame!: x y z qx qy qz: \n" << uncertaintySensor << std::endl;

        ///sampling from ellipsoid
        int samplesNo=250;     // How many samples (columns) to draw
        // Define mean and covariance of the distribution
        Eigen::Vector3d mean(3);
        Eigen::MatrixXd covar(3,3);
        mean  <<  0,  0, 0;
        covar <<  4.6845, -1.8587, 1.6523,
                 -1.8587, 1.3192, -0.7436,
                  1.6523, -0.7436, 1.2799;
        PointCloud ellipsoid;
        for (int i=0;i<samplesNo;i++)
            ellipsoid.push_back(sampleFromMultivariateGaussian(mean, covar));
        savePointCloud("../../resources/ellipsoidCloud.m",ellipsoid);

        std::cout << "\n\n\nTrajectory test\n";
        //create reference trajectory
        Mat34 pose = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>(0.3*roomDim[0], -0.3*roomDim[1],roomDim[2]/2.0);
        int motions = 20;
        std::vector<Mat34> trajectory; trajectory.push_back(pose);
        Mat34 moveForward = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>(0, (0.6*roomDim[1])/float_type(motions), 0);
        Mat34 moveRot = quatFromEuler((M_PI/2.0)/float_type(motions),0,0)*Eigen::Translation<double,3>(0, 0, 0);
        for (int i=0;i<motions;i++){
            pose.matrix() *= move.matrix();
            trajectory.push_back(pose);
        }
        move = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>((roomDim[0]/2.0)/float_type(motions), 0, 0);
        for (int j=0;j<4;j++){
            //forward
            for (int i=0;i<motions;i++){
                pose.matrix() *= moveForward.matrix();
                trajectory.push_back(pose);
            }
            //rotate pi/2
            for (int i=0;i<motions;i++){
                pose.matrix() *= moveRot.matrix();
                trajectory.push_back(pose);
            }
        }
        saveTrajectory("../../resources/trajectory.m",trajectory, "k");

        graph = createPoseGraphG2O(sensorModel.config.pose);
        cout << "Current graph: " << graph->getName() << std::endl;

        //move camera along reference trajectory and estimate trajectory
        std::vector<Mat34> trajectorySensor; initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
        trajectorySensor.push_back(initPose);
        int vertexId = 0;
        Vec3 pos(initPose(0,3), initPose(1,3), initPose(2,3));  Quaternion rot(initPose.rotation());
        VertexSE3 vertex(vertexId, pos, rot);
        if (!graph->addVertexPose(vertex))
            std::cout << "error: vertex exists!\n";
        vertexId++;

        std::vector< std::vector<int> > setIds;
        std::vector<PointCloud> cloudSeq;
        std::vector< std::vector<Mat33> > uncertaintySet;
        Mat34 sensorPose; sensorPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
        setAids = getCloud(sensorPose, sensorModel, room, cloudA, uncertaintyCloudA);
        cloudSeq.push_back(cloudA);
        setIds.push_back(setAids);
        uncertaintySet.push_back( uncertaintyCloudA );
        for (int i=1;i<trajectory.size();i++){
            //get point clouds
            sensorPose.matrix() = trajectory[i].matrix()*sensorModel.config.pose.matrix();
            std::vector<int> setBids = getCloud(sensorPose, sensorModel, room, cloudB, uncertaintyCloudB);

            //match and estimate transformation
            matchClouds(cloudSeq.back(), setA, uncertaintySet.back(), setAUncertainty, setIds.back(), cloudB, setB, uncertaintyCloudB, setBUncertainty, setBids);
            if (setA.rows()>3){
                std::cout << "matched: " << setA.rows() << "\n";
                trans = transEst->computeTransformation(setB, setA);
                uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
                uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                sensorPose.matrix() = trajectorySensor.back().matrix()*trans.matrix();
                trajectorySensor.push_back(sensorPose);
                //add vertex to the graph
                pos.x() = sensorPose(0,3); pos.y() = sensorPose(1,3); pos.z() = sensorPose(2,3);
                Quaternion quatSensor(sensorPose.rotation());
                VertexSE3 vertexSensor(vertexId, pos, quatSensor);
                if (!graph->addVertexPose(vertexSensor))
                    std::cout << "error: vertex exists!\n";
                // add edge to the g2o graph
                pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                Quaternion quatMotion(trans.rotation());
                RobotPose measurement(pos, quatMotion);
                Mat66 infoMat(uncertainty); infoMat.inverse(); infoMat.setIdentity();
                EdgeSE3 edge(measurement,infoMat,vertexId-1,vertexId);
                if (!graph->addEdgeSE3(edge))
                    std::cout << "error: vertex doesn't exist!\n";
                vertexId++;
            }
            else{
                std::cout << "could not add edge\n";
            }
            cloudSeq.push_back(cloudB);
            setIds.push_back(setBids);
            uncertaintySet.push_back(uncertaintyCloudB);
        }
        //additional edges
        for (int i=7;i<trajectory.size();i++){

            //match and estimate transformation
            for (int j=2;j<8;j++){
                matchClouds(cloudSeq[i-j], setA, uncertaintySet[i-j], setAUncertainty, setIds[i-j], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
                if (setA.rows()>3){
                    trans = transEst->computeTransformation(setB, setA);
                    uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
                    uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                    sensorPose.matrix() = trajectorySensor.back().matrix()*trans.matrix();
                    // add edge to the g2o graph
                    pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                    Quaternion quatMotion(trans.rotation());
                    RobotPose measurement(pos, quatMotion);
                    Mat66 infoMat(uncertainty); infoMat.inverse(); infoMat.setIdentity();
                    EdgeSE3 edge(measurement,infoMat,i-j,i);
                    if (!graph->addEdgeSE3(edge))
                        std::cout << "error: vertex doesn't exist!\n";
                }
                else{
                    std::cout << "could not add\n";
                }
            }
        }

        saveTrajectory("../../resources/trajectorySensor.m",trajectorySensor, "r");
        graph->save2file("../../resources/graphKabsch.g2o");
        //optimize
        std::cout << "optimization\n";
        std::thread tOpt(optimizeAndPrune);

        std::cout << "end optimization\n";
        tOpt.join();
        std::vector<Mat34> trajectoryOpt = graph->getTrajectory();
        saveTrajectory("../../resources/trajectory_g2o_nouncertainty.m",trajectoryOpt, "g");

        graph->clear();
        //move camera along reference trajectory and estimate trajectory
        std::vector<Mat34> trajectorySensor2; initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
        trajectorySensor2.push_back(initPose);
        int vertexId2 = 0;
        Vec3 pos2(initPose(0,3), initPose(1,3), initPose(2,3));  Quaternion rot2(initPose.rotation());
        VertexSE3 vertex2(vertexId2, pos2, rot2);
        if (!graph->addVertexPose(vertex2))
            std::cout << "error: vertex exists!\n";
        vertexId2++;
        for (int i=1;i<trajectory.size();i++){
            //get point clouds
            sensorPose.matrix() = trajectory[i].matrix()*sensorModel.config.pose.matrix();

            //match and estimate transformation
            matchClouds(cloudSeq[i-1], setA, uncertaintySet[i-1], setAUncertainty, setIds[i-1], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
            if (setA.rows()>3){
                std::cout << "matched: " << setA.rows() << "\n";
                trans = transEst->computeTransformation(setB, setA);
                uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
                uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                sensorPose.matrix() = trajectorySensor.back().matrix()*trans.matrix();
                //add vertex to the graph
                pos.x() = sensorPose(0,3); pos.y() = sensorPose(1,3); pos.z() = sensorPose(2,3);
                Quaternion quatSensor(sensorPose.rotation());
                VertexSE3 vertexSensor(vertexId2, pos, quatSensor);
                if (!graph->addVertexPose(vertexSensor))
                    std::cout << "error: vertex exists!\n";
                // add edge to the g2o graph
                pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                Quaternion quatMotion(trans.rotation());
                RobotPose measurement(pos, quatMotion);
                Mat66 infoMat(uncertainty); infoMat = infoMat.inverse();
                EdgeSE3 edge(measurement,infoMat,vertexId2-1,vertexId2);
                if (!graph->addEdgeSE3(edge))
                    std::cout << "error: vertex doesn't exist!\n";
                vertexId2++;
            }
            else{
                std::cout << "could not add edge\n";
            }
        }
std::cout << "more edges\n";
        //additional edges
        for (int i=7;i<trajectory.size();i++){
            //match and estimate transformation
            for (int j=2;j<8;j++){
                matchClouds(cloudSeq[i-j], setA, uncertaintySet[i-j], setAUncertainty, setIds[i-j], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
                if (setA.rows()>3){
                    trans = transEst->computeTransformation(setB, setA);
                    uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
                    uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                    sensorPose.matrix() = trajectorySensor.back().matrix()*trans.matrix();
                    // add edge to the g2o graph
                    pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                    Quaternion quatMotion(trans.rotation());
                    RobotPose measurement(pos, quatMotion);
                    Mat66 infoMat(uncertainty); infoMat = infoMat.inverse();
                    EdgeSE3 edge(measurement,infoMat,i-j,i);
                    if (!graph->addEdgeSE3(edge))
                        std::cout << "error: vertex doesn't exist!\n";
                }
                else{
                    std::cout << "could not add\n";
                }
            }
        }

        graph->save2file("../../resources/graphKabsch_uncertainty.g2o");
        //optimize
        std::cout << "optimization\n";
        std::thread tOpt2(optimizeAndPrune);
        tOpt2.join();
        std::cout << "end optimization\n";

        std::vector<Mat34> trajectoryOpt2 = graph->getTrajectory();
        saveTrajectory("../../resources/trajectory_g2o_uncertainty.m",trajectoryOpt2, "b");
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
