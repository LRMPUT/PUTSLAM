#include "../include/Utilities/simulator.h"
#include <fstream>
#include "Defs/eigen3.h"
#include <vector>

Simulator::Simulator(void){
    generator.seed((unsigned int)time(0));
}

///create environment -- room(walls,floor, ceiling)
void Simulator::createRoom(size_t pointsNo, double width, double length, double height){
    environment.clear();
    std::uniform_real_distribution<double> distributionWidth(-width/2.0, width/2.0);
    std::uniform_real_distribution<double> distributionLength(-length/2.0, length/2.0);
    std::uniform_real_distribution<double> distributionHeight(0, height);
    for (size_t i = 0; i<pointsNo/6;i++){
        Point3D point;
        //floor
        point.x = distributionWidth(generator); point.y = distributionLength(generator); point.z = 0;
        environment.push_back(point);
        //ceiling
        point.x = distributionWidth(generator); point.y = distributionLength(generator); point.z = height;
        environment.push_back(point);
        //wall 1
        point.x = -width/2.0; point.y = distributionLength(generator); point.z = distributionHeight(generator);
        environment.push_back(point);
        //wall 2
        point.x = width/2.0; point.y = distributionLength(generator); point.z = distributionHeight(generator);
        environment.push_back(point);
        //wall 3
        point.x = distributionWidth(generator); point.y = -length/2.0; point.z = distributionHeight(generator);
        environment.push_back(point);
        //wall 4
        point.x = distributionWidth(generator); point.y = length/2.0; point.z = distributionHeight(generator);
        environment.push_back(point);
    }
}

///create environment -- random patches
void Simulator::createEnvironment(size_t pointsNo, double width, double length, double height){
    environment.clear();
    std::uniform_real_distribution<double> distributionWidth(-width/2.0, width/2.0);
    std::uniform_real_distribution<double> distributionLength(-length/2.0, length/2.0);
    std::uniform_real_distribution<double> distributionHeight(-height/2.0, height/2.0);

    double patchSize[3]={0.05,0,0.05};
    std::uniform_real_distribution<double> distWidthPatch(-patchSize[0], patchSize[0]);
    std::uniform_real_distribution<double> distLengthPatch(-patchSize[1], patchSize[1]);
    std::uniform_real_distribution<double> distHeightPatch(-patchSize[2], patchSize[2]);
    int patchNo=4;
    for (size_t i = 0; i<pointsNo;i++){
        Point3D point;
        point.x = distributionWidth(generator); point.y = distributionLength(generator); point.z = distributionHeight(generator);
        if (i%2){
            for (int j=0;j<patchNo;j++){
                Point3D pointTmp;
                pointTmp.x = point.x + distWidthPatch(generator); pointTmp.y = point.y + distLengthPatch(generator); pointTmp.z = point.z + distHeightPatch(generator);
                environment.push_back(pointTmp);
            }
        }
        else {
            for (int j=0;j<patchNo;j++){
                Point3D pointTmp;
                pointTmp.x = point.x + distLengthPatch(generator); pointTmp.y = point.y + distWidthPatch(generator); pointTmp.z = point.z + distHeightPatch(generator);
                environment.push_back(pointTmp);
            }
        }
    }
}

/// load environment from file
void Simulator::loadEnvironment(std::string filename){
    std::string line;    std::ifstream myfile(filename);
    Point3D point;
    if (myfile.is_open()) {
        Vec3 pos;
        while ( getline (myfile,line) ) {
            std::istringstream is(line);
            is >> pos.x() >> pos.y() >> pos.z();
            point.x = pos.x(); point.y = pos.y(); point.z = pos.z();
            environment.push_back(point);
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
}

/// samples from multivariate gaussian
Point3D Simulator::sampleFromMultivariateGaussian(Eigen::Vector3d mean, Eigen::MatrixXd cov){

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

/// get point cloud from current camera view
std::vector<int> Simulator::getCloud(const Mat34& sensorPose, DepthSensorModel& sensorModel, PointCloud& setPoints, std::vector<Mat33>& setUncertainty){
    std::vector<int> pointIdentifiers;
    setPoints.clear();
    setUncertainty.clear();
    for (size_t i=0;i<environment.size();i++){
        Eigen::Vector4d point(environment[i].x, environment[i].y, environment[i].z, 1);
        Eigen::Vector4d pointCamera = sensorPose.matrix().inverse()*point;
        Eigen::Vector3d point2d = sensorModel.inverseModel(pointCamera(0), pointCamera(1), pointCamera(2));
        if (point2d(0)!=-1){
            Mat33 uncertainty;
            sensorModel.computeCov((uint_fast16_t)point2d(0), (uint_fast16_t)point2d(1), point2d(2), uncertainty);
            Point3D point3D = sampleFromMultivariateGaussian(Eigen::Vector3d(pointCamera(0), pointCamera(1), pointCamera(2)),uncertainty);
            //point3D.x = pointCamera[0]; point3D.y = pointCamera[1]; point3D.z = pointCamera[2]; // no noise
            //point3D.x = room[i].x; point3D.y = room[i].y; point3D.z = room[i].z; // no noise global frame
            Eigen::Vector3d point2dTmp = sensorModel.inverseModel(point3D.x, point3D.y, point3D.z);
            if (point2dTmp(0)!=-1){
                setPoints.push_back(point3D);
                Eigen::Vector3d inv2d = sensorModel.inverseModel(point3D.x, point3D.y, point3D.z);
                sensorModel.computeCov((uint_fast16_t)inv2d(0), (uint_fast16_t)inv2d(1), inv2d(2), uncertainty);
                setUncertainty.push_back(uncertainty);
                pointIdentifiers.push_back((int)i);
            }
        }
    }
    return pointIdentifiers;
}

/// match point clouds
bool Simulator::matchClouds(const PointCloud& setAin, Eigen::MatrixXd& setAout, const std::vector<Mat33>& uncertaintyAin, std::vector<Mat33>& uncertaintyAout, const std::vector<int>& setAids, const PointCloud& setBin, Eigen::MatrixXd& setBout, const std::vector<Mat33>& uncertaintyBin, std::vector<Mat33>& uncertaintyBout, const std::vector<int>& setBids){
    int matchesNo=0;
    for (size_t i=0;i<setAin.size();i++){
        if (std::find(setBids.begin(),setBids.end(),setAids[i])!=setBids.end())
            matchesNo++;
    }
    setAout.resize(matchesNo,3); setBout.resize(matchesNo,3);
    uncertaintyAout.clear(); uncertaintyBout.clear();
    if (matchesNo==0){
        return false;
    }
    matchesNo=0;
    for (size_t i=0;i<setAin.size();i++){
        std::vector<int>::const_iterator itB = std::find(setBids.begin(),setBids.end(),setAids[i]);
        if (itB!=setBids.end()){
            setAout(matchesNo,0) = setAin[i].x; setAout(matchesNo,1) = setAin[i].y; setAout(matchesNo,2) = setAin[i].z;
            uncertaintyAout.push_back(uncertaintyAin[i]);
            setBout(matchesNo,0) = setBin[itB-setBids.begin()].x; setBout(matchesNo,1) = setBin[itB-setBids.begin()].y; setBout(matchesNo,2) = setBin[itB-setBids.begin()].z;
            uncertaintyBout.push_back(uncertaintyBin[itB-setBids.begin()]);
            matchesNo++;
        }
    }
    return true;
}

/// run experiment and return a sequence of point clouds
void Simulator::moveCamera(const std::vector<Mat34>& robotTrajectory, const Mat34 sensorPose, DepthSensorModel& sensorModel, std::vector<PointCloud>& cloudSeq, std::vector< std::vector<int> >& setIds, std::vector< std::vector<Mat33> >& uncertaintySet){
    cloudSeq.clear();    setIds.clear();    uncertaintySet.clear();
    Mat34 sensorPoseGlobal; sensorPoseGlobal.matrix() = robotTrajectory[0].matrix()*sensorPose.matrix();
    PointCloud cloud; std::vector<Mat33> uncertaintyCloud;
    //std::vector<int> setId = getCloud(sensorPoseGlobal, sensorModel, environment, cloud, uncertaintyCloud);
    //savePointCloud("../../resources/KabschUncertainty/cloud0.m", cloudA);
    //cloudA = cloud2local(cloudA, sensorPose);
    for (size_t i=0;i<robotTrajectory.size();i++){
        //get point clouds
        sensorPoseGlobal.matrix() = robotTrajectory[i].matrix()*sensorPose.matrix();
        std::vector<int> setId = getCloud(sensorPoseGlobal, sensorModel, cloud, uncertaintyCloud);
        cloudSeq.push_back(cloud);
        setIds.push_back(setId);
        //saveImageFeatures("../../resources/simulator/frame0.dat", sensorPose, cloudA, setAids, sensorModel, estimKabsch);
        uncertaintySet.push_back(uncertaintyCloud);
        //trajectorySensor.push_back(sensorPose);
    }
}

/// get environment
PointCloud& Simulator::getEnvironment(void){
    return environment;
}

void Simulator::saveImageFeatures(std::string filename, const Mat34& sensorPose, const PointCloud& cloud, const std::vector<int>& setIds, const DepthSensorModel& sensorModel, const Mat34& estimation){
    std::ofstream file(filename);
    file << "#sensor_x, sensor_y, sensor_z, sensor_qw, sensor_qx, sensor_qy, sensor_qz\n";
    file << "#Relative motion (estimation) Est_x, Est_y, Est_z, Est_qw, Est_qx, Est_qy, Est_qz\n";
    file << "#feature_id, feature_u, feature_v, depth\n";
    Quaternion q(sensorPose.rotation());
    Quaternion qKabsch(estimation.rotation());
    file << sensorPose(0,3) << ", " << sensorPose(1,3) << ", " << sensorPose(2,3) << ", " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "\n";
    file << estimation(0,3) << ", " << estimation(1,3) << ", " << estimation(2,3) << ", " << qKabsch.w() << ", " << qKabsch.x() << ", " << qKabsch.y() << ", " << qKabsch.z() << "\n";
    for (size_t i=0;i<cloud.size();i++){
        Eigen::Vector3d cameraPoint = sensorModel.inverseModel(cloud[i].x, cloud[i].y, cloud[i].z);
        file << setIds[i] << ", " << cameraPoint.x() << ", " << cameraPoint.y() << ", " << cameraPoint.z() << "\n";
    }
    file.close();
}

/// load trajectory from file (Freiburg notation)
void Simulator::loadTrajectory(std::string filename){
    std::string line;    std::ifstream myfile(filename);
    if (myfile.is_open()) {
        trajectory.clear();
        Quaternion q; Vec3 pos;
        double id;
        Mat34 pose;
        while ( getline (myfile,line) ) {
            std::istringstream is(line);
            is >> id >> pos.x() >> pos.y() >> pos.z() >> q.x() >> q.y() >> q.z() >> q.w();
            for (int i=0;i<3;i++)
                for (int j=0;j<3;j++)
                    pose(i,j) = q.matrix()(i,j);
            pose(0,3) = pos.x(); pose(1,3) = pos.y(); pose(2,3) = pos.z();
            trajectory.push_back(pose);
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
}

/// get trajectory
std::vector<Mat34>& Simulator::getTrajectory(void){
    return trajectory;
}
