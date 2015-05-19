//source: http://www.cplusplus.com/forum/beginner/26251/
//@author: m4ster r0shi

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <iostream>
#include <vector>
#include "../Defs/putslam_defs.h"
#include "../Grabber/depthSensorModel.h"

using namespace putslam;

class Simulator
{
public:

    Simulator(void);
    ~Simulator(){}

    /// create environment -- room(walls,floor, ceiling)
    void createRoom(size_t pointsNo, float_type width, float_type length, float_type height);
    /// create environment -- random patches
    void createEnvironment(size_t pointsNo, float_type width, float_type length, float_type height);
    /// load environment from file
    void loadEnvironment(std::string filename);
    /// run experiment and return a sequence of point clouds
    void moveCamera(const std::vector<Mat34>& robotTrajectory, const Mat34 sensorPose, DepthSensorModel& sensorModel, std::vector<PointCloud>& cloudSeq, std::vector< std::vector<int> >& setIds, std::vector< std::vector<Mat33> >& uncertaintySet);
    /// get environment
    PointCloud& getEnvironment(void);
    /// match point clouds
    bool matchClouds(const PointCloud& setAin, Eigen::MatrixXd& setAout, const std::vector<Mat33>& uncertaintyAin, std::vector<Mat33>& uncertaintyAout, const std::vector<int>& setAids, const PointCloud& setBin, Eigen::MatrixXd& setBout, const std::vector<Mat33>& uncertaintyBin, std::vector<Mat33>& uncertaintyBout, const std::vector<int>& setBids);
    /// get point cloud from current camera view
    std::vector<int> getCloud(const Mat34& sensorPose, DepthSensorModel& sensorModel, PointCloud& setPoints, std::vector<Mat33>& setUncertainty);
    /// save set of image features to file
    void saveImageFeatures(std::string filename, const Mat34& sensorPose, const PointCloud& cloud, const std::vector<int>& setIds, const DepthSensorModel& sensorModel, const Mat34& estimation);
    /// load trajectory from file (Freiburg notation)
    void loadTrajectory(std::string filename);
    /// get trajectory
    std::vector<Mat34>& getTrajectory(void);

private:
    /// samples from multivariate gaussian
    Point3D sampleFromMultivariateGaussian(Eigen::Vector3d mean, Eigen::MatrixXd cov);

    /// simulation environment
    PointCloud environment;
    ///random engine
    std::default_random_engine generator;
    /// trajectory
    std::vector<Mat34> trajectory;

};

#endif //_SIMULATOR_H_
