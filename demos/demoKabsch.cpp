#include <iostream>
#include <thread>
#include "../include/Defs/putslam_defs.h"
#include "Tracker/trackerKLT.h"
#include "Utilities/simulator.h"
#include "TransformEst/kabschEst.h"
#include "TransformEst/g2oEst.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include "PoseGraph/graph_g2o.h"
#include <cmath>
#include <Eigen/Dense>
#include "../include/Grabber/kinectGrabber.h"

using namespace std;

#define INIT_VERTEX_ID 10000

auto startT = std::chrono::high_resolution_clock::now();
std::default_random_engine generator;

/// noise: x, y, z, qx, qy, qz
float_type noise[6] = {0.01, 0.02, 0.03, 0.0, 0.000, 0.000};
/// x, y, z, fi, psi, theta
float_type transformation[6] = {0.1, 0.2, -0.3, 0.0, 0.0, -0.0};

Graph * graph;

// optimization and pruning thread
void optimizeAndPrune(){
    // graph pruning and optimization
    graph->optimizeAndPrune(2.0, 70);
}

// optimization and pruning thread
void optimize(int iterNo){
    // graph pruning and optimization
    graph->optimize(iterNo);
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
    for (size_t i = 0; i<trajectory.size();i=i+trajectory.size()/10){
        plotCoordinates(file, trajectory[i]);
    }
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

void saveGroundTruth(std::string filename, std::vector<Mat34>& trajectory) {
    std::ofstream file(filename);
    file << "# ground truth trajectory:\n";
    file << "# file: 'simulator_room.bag'\n";
    file << "# timestamp tx ty tz qx qy qz qw\n";
    for (int i=0;i<trajectory.size();i++){
        Quaternion q(trajectory[i].rotation());
        file << i << " " << trajectory[i](0,3) << " " << trajectory[i](1,3) << " " << trajectory[i](2,3) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    file.close();

    std::ofstream fileInds("../../resources/KabschUncertainty/g2oIndices");
    for (int i=0;i<trajectory.size();i++){
        fileInds << i << "\n";
    }
    fileInds.close();
}

//converts local position to global
Eigen::MatrixXd cloud2local(const Eigen::MatrixXd& cloud, const Mat34& sensorPose){
    Eigen::MatrixXd tmp; tmp.resize(cloud.rows(),cloud.cols());
    for (int i=0;i<cloud.rows();i++){
        Mat34 pose; pose.setIdentity();
        pose(0,3) = cloud(i,0); pose(1,3) = cloud(i,1); pose(2,3) = cloud(i,2);
        //std::cout << cloud(i,1) << ", " << cloud(i,1) << ", " << cloud(i,2) << "\n";
        pose.matrix() = sensorPose.matrix().inverse() * pose.matrix();
        tmp(i,0) = pose(0,3); tmp(i,1) = pose(1,3); tmp(i,2) = pose(2,3);
        //std::cout << tmp(i,0) << ", " << tmp(i,1) << ", " << tmp(i,2) << "\n";
        //getchar();
    }
    return tmp;
}

//converts local position to global
PointCloud cloud2global(const PointCloud& cloud, Mat34& sensorPose){
    PointCloud tmp;
    for (int i=0;i<cloud.size();i++){
        Point3D point;
        Mat34 pose; pose.setIdentity();
        pose(0,3) = cloud[i].x; pose(1,3) = cloud[i].y; pose(2,3) = cloud[i].z;
        pose.matrix() = sensorPose.matrix() * pose.matrix();
        point.x = pose(0,3); point.y = pose(1,3); point.z = pose(2,3);
        tmp.push_back(point);
    }
    return tmp;
}

//converts local position to global
PointCloud cloud2local(const PointCloud& cloud, Mat34& sensorPose){
    PointCloud tmp;
    for (int i=0;i<cloud.size();i++){
        Point3D point;
        Mat34 pose; pose.setIdentity();
        pose(0,3) = cloud[i].x; pose(1,3) = cloud[i].y; pose(2,3) = cloud[i].z;
        pose.matrix() = sensorPose.matrix().inverse() * pose.matrix();
        point.x = pose(0,3); point.y = pose(1,3); point.z = pose(2,3);
        tmp.push_back(point);
    }
    return tmp;
}

/// compute mean squared error for the map
float_type computeMapAccuracy(Graph* currentGraph, PointCloud& groundTruth){
    float_type accuracy = 0;
    for (int i=0;i<groundTruth.size();i++){
        Point3D point = ((PoseGraphG2O*) currentGraph)->getVertex(i+INIT_VERTEX_ID);
        if ((point.x!=-1)&&(point.y!=-1)) //if point exists in the map
            accuracy+=pow(point.x - groundTruth[i].x,2.0) + pow(point.y - groundTruth[i].y,2.0) + pow(point.z - groundTruth[i].z,2.0);
    }
    return accuracy/float_type(groundTruth.size());
}

void runExperiment(int expType, const std::vector<Mat34>& trajectory, const DepthSensorModel& sensorModel, const std::vector<PointCloud>& cloudSeq, const std::vector< std::vector<Mat33> >& uncertaintySet, const std::vector< std::vector<int> >& setIds, Simulator& simulator, TransformEst* transEst){
    Mat34 initPose;
    std::vector<Mat34> trajectorySensor2; initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
    trajectorySensor2.push_back(initPose);
    int vertexId2 = 0;
    Vec3 pos2(initPose(0,3), initPose(1,3), initPose(2,3));  Quaternion rot2(initPose.rotation());
    VertexSE3 vertex2(vertexId2, pos2, rot2);
    if (!graph->addVertexPose(vertex2))
        std::cout << "error: vertex exists!\n";
    vertexId2++;
    Mat34 sensorPose;
    Eigen::MatrixXd setA(1, 3);
    Eigen::MatrixXd setB(1, 3);
    Mat66 uncertainty;
    for (int i=1;i<trajectory.size();i++){
        //get point clouds
        //sensorPose.matrix() = trajectory[i].matrix()*sensorModel.config.pose.matrix();

        //match and estimate transformation
        std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
        std::cout << "View points: " << cloudSeq[i-1].size() << "\n";
        simulator.matchClouds(cloudSeq[i-1], setA, uncertaintySet[i-1], setAUncertainty, setIds[i-1], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
        if (setA.rows()>3){
            std::cout << "matched: " << setA.rows() << "\n";
            Mat34 trans;
            if (expType==2){
                trans = transEst->computeTransformation(setB, setA);
                TransformEst* g2oEst = createG2OEstimator();
                trans = ((G2OEst*)g2oEst)->computeTransformation(setB, setBUncertainty, setA, setAUncertainty, trans);
            }
            else {
                trans = transEst->computeTransformation(setB, setA);
                //TransformEst* g2oEst = createG2OEstimator();
                //trans = ((G2OEst*)g2oEst)->computeTransformation(setB, setBUncertainty, setA, setAUncertainty, trans);
            }
            //trans(0,3) = 0;
            //trans(0,1) = 0; trans(0,2) = 0;
            //trans(1,0) = 0; trans(2,0) = 0;
            //trans(0,0) = 1;
            //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
            sensorPose.matrix() = trajectorySensor2.back().matrix()*trans.matrix();

            //add vertex to the graph
            Vec3 pos;
            trajectorySensor2.push_back(sensorPose);
            pos.x() = sensorPose(0,3); pos.y() = sensorPose(1,3); pos.z() = sensorPose(2,3);
            Quaternion quatSensor(sensorPose.rotation());
            VertexSE3 vertexSensor(vertexId2, pos, quatSensor);
            if (!graph->addVertexPose(vertexSensor))
                std::cout << "error: vertex exists!\n";
            // add edge to the g2o graph
            pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
            Quaternion quatMotion(trans.rotation());
            RobotPose measurement(pos, quatMotion);
            Mat66 infoMat;
            if (expType==0||expType==1)
                infoMat.setIdentity();
            else if (expType==2){
                //uncertainty = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                //uncertainty = transEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                // create g2o transform estimator
                TransformEst* g2oEst = createG2OEstimator();
                uncertainty = g2oEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                //uncertainty = transEst->computeUncertaintyGrisetti(setB, setA, trans);
                //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                // [xx  xy  xz  xrx  xry  xrz ]    [yy  yz  yx  yry  yrz  yrx ]
                // [yx  yy  yz  yrx  yry  yrz ] -> [zy  zz  zx  zry  zrz  zrx ]
                // [zx  zy  zz  zrx  zry  zrz ]    [xy  xz  xx  xry  xrz  xrx ]
                // [rxx rxy rxz rxrx rxry rxrz]    [ryy ryz ryx ryry ryrz ryrx]
                // [ryx ryy ryz ryrx ryry ryrz] -> [rzy rzz rzx rzry rzrz rzrx]
                // [rzx rzy rzz rzrx rzry rzrz]    [rxy rxz rxx rxry rxrz rxrx]
                /*Mat66 unc;
                unc(0,0) = uncertainty(1,1); unc(0,1) = 0; unc(0,2) = 0; unc(0,3) = 0; unc(0,4) = 0; unc(0,5) = 0;
                unc(1,0) = 0; unc(1,1) = uncertainty(2,2); unc(1,2) = 0; unc(1,3) = 0; unc(1,4) = 0; unc(1,5) = 0;
                unc(2,0) = 0; unc(2,1) = 0; unc(2,2) = uncertainty(3,3); unc(2,3) = 0; unc(2,4) = 0; unc(2,5) = 0;
                unc(3,0) = 0; unc(3,1) = 0; unc(3,2) = 0; unc(3,3) = 1; unc(3,4) = 0; unc(3,5) = 0;
                unc(4,0) = 0; unc(4,1) = 0; unc(4,2) = 0; unc(4,3) = 0; unc(4,4) = 1; unc(4,5) = 0;
                unc(5,0) = 0; unc(5,1) = 0; unc(5,2) = 0; unc(5,3) = 0; unc(5,4) = 0; unc(5,5) = 1;
                uncertainty = unc;*/
                //std::cout << "uncertainty: \n" << uncertainty << "\n";
                /*uncertainty (0,0) = 0; uncertainty (0,1) = 0; uncertainty (0,2) = 0; uncertainty (0,3) = 0; uncertainty (0,4) = 0; uncertainty (0,5) = 0;
                uncertainty (3,0) = 0; uncertainty (3,1) = 0; uncertainty (3,2) = 0; uncertainty (3,3) = 0; uncertainty (3,4) = 0; uncertainty (3,5) = 0;
                uncertainty (4,0) = 0; uncertainty (4,1) = 0; uncertainty (4,2) = 0; uncertainty (4,3) = 0; uncertainty (4,4) = 0; uncertainty (4,5) = 0;
                uncertainty (0,0) = 1e-7; uncertainty (1,0) = 0; uncertainty (2,0) = 0; uncertainty (3,0) = 0; uncertainty (4,0) = 0; uncertainty (5,0) = 0;
                uncertainty (0,3) = 0; uncertainty (1,3) = 0; uncertainty (2,3) = 0; uncertainty (3,3) = 1e-7; uncertainty (4,3) = 0; uncertainty (5,3) = 0;
                uncertainty (0,4) = 0; uncertainty (1,4) = 0; uncertainty (2,4) = 0; uncertainty (3,4) = 0; uncertainty (4,4) = 1e-7; uncertainty (5,4) = 0;*/
                infoMat = uncertainty.inverse();
                /*Quaternion qqq(trans.rotation());
                infoMat(0,0) = infoMat(0,0); infoMat(0,1) = 0; infoMat(0,2) = 0; infoMat(0,3) = 0; infoMat(0,4) = 0; infoMat(0,5) = 0;
                infoMat(1,0) = 0; infoMat(1,1) = infoMat(1,1); infoMat(1,2) = 0; infoMat(1,3) = 0; infoMat(1,4) = 0; infoMat(1,5) = 0;
                infoMat(2,0) = 0; infoMat(2,1) = 0; infoMat(2,2) = infoMat(2,2); infoMat(2,3) = 0; infoMat(2,4) = 0; infoMat(2,5) = 0;
                infoMat(3,0) = 0; infoMat(3,1) = 0; infoMat(3,2) = 0; infoMat(3,3)=1; infoMat(3,4)=0; infoMat(3,5)=0;
                infoMat(4,0) = 0; infoMat(4,1) = 0; infoMat(4,2) = 0; infoMat(4,3)=0; infoMat(4,4)=1; infoMat(4,5)=0;
                infoMat(5,0) = 0; infoMat(5,1) = 0; infoMat(5,2) = 0; infoMat(5,3)=0; infoMat(5,4)=0; infoMat(5,5)=1;
                */
                /*std::cout << "trans: \n" <<  trans.matrix() << "\n";
                std::cout << "uncertainty: \n" <<  uncertainty << "\n";
                std::cout << "info: \n" << infoMat << "\n";
                getchar();*/
            }
            else if (expType==3){
                uncertainty = transEst->computeUncertaintyStrasdat(setA, setAUncertainty, setB, setBUncertainty, trans);
                infoMat = uncertainty;//.inverse();
            }
            EdgeSE3 edge(measurement,infoMat,vertexId2-1,vertexId2);
            if (!graph->addEdgeSE3(edge))
                std::cout << "error: vertex doesn't exist!\n";
            vertexId2++;
        }
        else{
            std::cout << "could not add edge1\n";
            getchar();
        }
    }
    if (expType>0){
        std::cout << "more edges\n";
        //additional edges
        for (int i=2;i<trajectory.size();i++){
            //match and estimate transformation
            for (int j=2;j<18;j++){
                if (i-j<0)
                    break;
                std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
                simulator.matchClouds(cloudSeq[i-j], setA, uncertaintySet[i-j], setAUncertainty, setIds[i-j], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
                int efficientFeatures = 15;
                if (setA.rows()>efficientFeatures){
                    Mat34 trans;
                    if (expType==2){
                        trans = transEst->computeTransformation(setB, setA);
                        TransformEst* g2oEst = createG2OEstimator();
                        trans = ((G2OEst*)g2oEst)->computeTransformation(setB, setBUncertainty, setA, setAUncertainty, trans);
                    }
                    else {
                        trans = transEst->computeTransformation(setB, setA);
                        //TransformEst* g2oEst = createG2OEstimator();
                        //trans = ((G2OEst*)g2oEst)->computeTransformation(setB, setBUncertainty, setA, setAUncertainty, trans);
                    }
                    //trans(0,3) = 0;
                    //trans(0,1) = 0; trans(0,2) = 0;
                    //trans(1,0) = 0; trans(2,0) = 0;
                    //trans(0,0) = 1;
                    //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                    // add edge to the g2o graph
                    Vec3 pos;
                    pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                    Quaternion quatMotion(trans.rotation());
                    RobotPose measurement(pos, quatMotion);
                    Mat66 infoMat;
                    if (expType==1)
                        infoMat.setIdentity();
                    else if (expType==2){
                        //uncertainty = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                        //uncertainty = transEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                        TransformEst* g2oEst = createG2OEstimator();
                        uncertainty = g2oEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                        //uncertainty = transEst->computeUncertaintyGrisetti(setB, setA, trans);
                        //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                        /*Mat66 unc;
                        unc(0,0) = uncertainty(1,1); unc(0,1) = 0; unc(0,2) = 0; unc(0,3) = 0; unc(0,4) = 0; unc(0,5) = 0;
                        unc(1,0) = 0; unc(1,1) = uncertainty(2,2); unc(1,2) = 0; unc(1,3) = 0; unc(1,4) = 0; unc(1,5) = 0;
                        unc(2,0) = 0; unc(2,1) = 0; unc(2,2) = uncertainty(3,3); unc(2,3) = 0; unc(2,4) = 0; unc(2,5) = 0;
                        unc(3,0) = 0; unc(3,1) = 0; unc(3,2) = 0; unc(3,3) = 1; unc(3,4) = 0; unc(3,5) = 0;
                        unc(4,0) = 0; unc(4,1) = 0; unc(4,2) = 0; unc(4,3) = 0; unc(4,4) = 1; unc(4,5) = 0;
                        unc(5,0) = 0; unc(5,1) = 0; unc(5,2) = 0; unc(5,3) = 0; unc(5,4) = 0; unc(5,5) = 1;
                        uncertainty = unc;*/
                        /*uncertainty (0,0) = 0; uncertainty (0,1) = 0; uncertainty (0,2) = 0; uncertainty (0,3) = 0; uncertainty (0,4) = 0; uncertainty (0,5) = 0;
                        uncertainty (3,0) = 0; uncertainty (3,1) = 0; uncertainty (3,2) = 0; uncertainty (3,3) = 0; uncertainty (3,4) = 0; uncertainty (3,5) = 0;
                        uncertainty (4,0) = 0; uncertainty (4,1) = 0; uncertainty (4,2) = 0; uncertainty (4,3) = 0; uncertainty (4,4) = 0; uncertainty (4,5) = 0;
                        uncertainty (0,0) = 1e-7; uncertainty (1,0) = 0; uncertainty (2,0) = 0; uncertainty (3,0) = 0; uncertainty (4,0) = 0; uncertainty (5,0) = 0;
                        uncertainty (0,3) = 0; uncertainty (1,3) = 0; uncertainty (2,3) = 0; uncertainty (3,3) = 1e-7; uncertainty (4,3) = 0; uncertainty (5,3) = 0;
                        uncertainty (0,4) = 0; uncertainty (1,4) = 0; uncertainty (2,4) = 0; uncertainty (3,4) = 0; uncertainty (4,4) = 1e-7; uncertainty (5,4) = 0;
                        */
                        infoMat = uncertainty.inverse();
                        /*Quaternion qqq(trans.rotation());
                        infoMat(0,0) = infoMat(0,0); infoMat(0,1) = 0; infoMat(0,2) = 0; infoMat(0,3) = 0; infoMat(0,4) = 0; infoMat(0,5) = 0;
                        infoMat(1,0) = 0; infoMat(1,1) = infoMat(1,1); infoMat(1,2) = 0; infoMat(1,3) = 0; infoMat(1,4) = 0; infoMat(1,5) = 0;
                        infoMat(2,0) = 0; infoMat(2,1) = 0; infoMat(2,2) = infoMat(2,2); infoMat(2,3) = 0; infoMat(2,4) = 0; infoMat(2,5) = 0;
                        infoMat(3,0) = 0; infoMat(3,1) = 0; infoMat(3,2) = 0; infoMat(3,3)=1; infoMat(3,4)=0; infoMat(3,5)=0;
                        infoMat(4,0) = 0; infoMat(4,1) = 0; infoMat(4,2) = 0; infoMat(4,3)=0; infoMat(4,4)=1; infoMat(4,5)=0;
                        infoMat(5,0) = 0; infoMat(5,1) = 0; infoMat(5,2) = 0; infoMat(5,3)=0; infoMat(5,4)=0; infoMat(5,5)=1;*/
                        /*Quaternion qqq(trans.rotation());
                        std::cout << "trans x y z qx qy qz qw: " << trans(0,3) << ", " << trans(1,3) << ", " << trans(2,3) << ", " << qqq.w() << ", " << qqq.x() << ", " << qqq.y() << ", " << qqq.z() << ", "<< "\n";
                        std::cout << "uncertainty: \n" << uncertainty << "\n";
                        std::cout << "infoMat: \n" << infoMat << "\n";
                        getchar();*/
                    }
                    else if (expType==3){
                        uncertainty = transEst->computeUncertaintyStrasdat(setA, setAUncertainty, setB, setBUncertainty, trans);
                        infoMat = uncertainty;//.inverse();
                    }
                    EdgeSE3 edge(measurement,infoMat,i-j,i);
                    if (!graph->addEdgeSE3(edge))
                        std::cout << "error: vertex doesn't exist!\n";
                }
                else{
                    std::cout << "could not add edge2\n";
                    //getchar();
                }
            }
        }
    }
}

void runExperiment2cameras(int expType, const std::vector<Mat34>& trajectory, const DepthSensorModel& sensorModel, const std::vector<PointCloud>& cloudSeq, const std::vector< std::vector<Mat33> >& uncertaintySet, const std::vector< std::vector<int> >& setIds, const std::vector<PointCloud>& cloudSeq2, const std::vector< std::vector<Mat33> >& uncertaintySet2, const std::vector< std::vector<int> >& setIds2, Simulator& simulator, TransformEst* transEst){
    Mat34 initPose;
    initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
    std::vector<Mat34> trajectorySensor2;
    trajectorySensor2.push_back(initPose);
    int vertexId2 = 0;
    Vec3 pos2(initPose(0,3), initPose(1,3), initPose(2,3));  Quaternion rot2(initPose.rotation());
    VertexSE3 vertex2(vertexId2, pos2, rot2);
    if (!graph->addVertexPose(vertex2))
        std::cout << "error: vertex exists!\n";
    vertexId2++;
    Mat34 sensorPose;
    Eigen::MatrixXd setA(1, 3);
    Eigen::MatrixXd setB(1, 3);
    Mat66 uncertainty;
    for (int i=1;i<trajectory.size();i++){
        //get point clouds
        //sensorPose.matrix() = trajectory[i].matrix()*sensorModel.config.pose.matrix();

        //match and estimate transformation
        std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
        std::cout << "View points: " << cloudSeq[i-1].size() << "\n";
        if (expType==2)
            simulator.matchClouds(cloudSeq2[i-1], setA, uncertaintySet2[i-1], setAUncertainty, setIds2[i-1], cloudSeq2[i], setB, uncertaintySet2[i], setBUncertainty, setIds2[i]);
        else
            simulator.matchClouds(cloudSeq[i-1], setA, uncertaintySet[i-1], setAUncertainty, setIds[i-1], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
        if (setA.rows()>3){
            std::cout << "matched: " << setA.rows() << "\n";
            Mat34 trans;
            trans = transEst->computeTransformation(setB, setA);
            if (expType==2){
                Vec3 pos;
                pos.x() = trans(2,3); pos.y() = -trans(1,3); pos.z() = trans(0,3);
                Mat33 rott;
                //  x  y  z        x->z y->-y z->x
                //x xx yx zx  x->z  zz  -yz    xz
                //y xy yy zy  y->-y -zy  yy   -xy
                //z xz yz zz  z->x  zx  -yx    xx
                rott(0,0) = trans.rotation()(2,2); rott(0,1) = -trans.rotation()(2,1); rott(0,2) = trans.rotation()(2,0);
                rott(1,0) = -trans.rotation()(1,2); rott(1,1) = trans.rotation()(1,1); rott(1,2) = -trans.rotation()(1,0);
                rott(2,0) = trans.rotation()(0,2); rott(2,1) = -trans.rotation()(0,1); rott(2,2) = trans.rotation()(0,0);
                trans.matrix()(0,3) = pos.x(); trans.matrix()(1,3) = pos.y(); trans.matrix()(2,3) = pos.z();
                trans.matrix()(0,0) = rott(0,0); trans.matrix()(0,1) = rott(0,1); trans.matrix()(0,2) = rott(0,2);
                trans.matrix()(1,0) = rott(1,0); trans.matrix()(1,1) = rott(1,1); trans.matrix()(1,2) = rott(1,2);
                trans.matrix()(2,0) = rott(2,0); trans.matrix()(2,1) = rott(2,1); trans.matrix()(2,2) = rott(2,2);
            }
            sensorPose.matrix() = trajectorySensor2.back().matrix()*trans.matrix();
            //add vertex to the graph
            Vec3 pos;
            trajectorySensor2.push_back(sensorPose);
            pos.x() = sensorPose(0,3); pos.y() = sensorPose(1,3); pos.z() = sensorPose(2,3);
            Quaternion quatSensor(sensorPose.rotation());
            VertexSE3 vertexSensor(vertexId2, pos, quatSensor);
            if (!graph->addVertexPose(vertexSensor))
                std::cout << "error: vertex exists!\n";
            // add edge to the g2o graph
            pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
            Quaternion quatMotion(trans.rotation());
            RobotPose measurement(pos, quatMotion);
            Mat66 infoMat;
            if (expType==4){
                uncertainty = transEst->computeUncertaintyG2O(setB, setBUncertainty, setA, setAUncertainty, trans);
                //TransformEst* g2oEst = createG2OEstimator();
                //uncertainty = g2oEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                //uncertainty = transEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                infoMat = uncertainty.inverse();
            }
            else
                infoMat.setIdentity();
            EdgeSE3 edge(measurement,infoMat,vertexId2-1,vertexId2);
            if (!graph->addEdgeSE3(edge))
                std::cout << "error: vertex doesn't exist!\n";
            vertexId2++;
        }
        else{
            std::cout << "could not add edge1\n";
            getchar();
        }
    }
    vertexId2 = 1;
    if (expType==3||expType==4){
        for (int i=1;i<trajectory.size();i++){
            //get point clouds
            //sensorPose.matrix() = trajectory[i].matrix()*sensorModel.config.pose.matrix();

            //match and estimate transformation
            std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
            std::cout << "View points: " << cloudSeq2[i-1].size() << "\n";
            simulator.matchClouds(cloudSeq2[i-1], setA, uncertaintySet2[i-1], setAUncertainty, setIds2[i-1], cloudSeq2[i], setB, uncertaintySet2[i], setBUncertainty, setIds2[i]);
            if (setA.rows()>3){
                std::cout << "matched: " << setA.rows() << "\n";
                Mat34 trans = transEst->computeTransformation(setB, setA);
                // add edge to the g2o graph
                Vec3 pos;
                pos.x() = trans(2,3); pos.y() = -trans(1,3); pos.z() = trans(0,3);
                Mat33 rott;
                //  x  y  z        x->z y->-y z->x
                //x xx yx zx  x->z  zz  -yz    xz
                //y xy yy zy  y->-y -zy  yy   -xy
                //z xz yz zz  z->x  zx  -yx    xx
                rott(0,0) = trans.rotation()(2,2); rott(0,1) = -trans.rotation()(2,1); rott(0,2) = trans.rotation()(2,0);
                rott(1,0) = -trans.rotation()(1,2); rott(1,1) = trans.rotation()(1,1); rott(1,2) = -trans.rotation()(1,0);
                rott(2,0) = trans.rotation()(0,2); rott(2,1) = -trans.rotation()(0,1); rott(2,2) = trans.rotation()(0,0);
                trans.matrix()(0,3) = pos.x(); trans.matrix()(1,3) = pos.y(); trans.matrix()(2,3) = pos.z();
                trans.matrix()(0,0) = rott(0,0); trans.matrix()(0,1) = rott(0,1); trans.matrix()(0,2) = rott(0,2);
                trans.matrix()(1,0) = rott(1,0); trans.matrix()(1,1) = rott(1,1); trans.matrix()(1,2) = rott(1,2);
                trans.matrix()(2,0) = rott(2,0); trans.matrix()(2,1) = rott(2,1); trans.matrix()(2,2) = rott(2,2);
                Quaternion quatMotion(rott);
                RobotPose measurement(pos, quatMotion);
                Mat66 infoMat;
                if (expType==4){
                    uncertainty = transEst->computeUncertaintyG2O(setB, setBUncertainty, setA, setAUncertainty, trans);
                    //TransformEst* g2oEst = createG2OEstimator();
                    //uncertainty = g2oEst->computeUncertainty(setB, setBUncertainty, setA, setAUncertainty, trans);
                    Mat66 unc;
                    unc(0,0) = uncertainty(2,2); unc(0,1) = -uncertainty(2,1); unc(0,2) = uncertainty(2,0); unc(0,3) = uncertainty(2,5); unc(0,4) = -uncertainty(2,4); unc(0,5) = uncertainty(2,3);
                    unc(1,0) = -uncertainty(1,2); unc(1,1) = uncertainty(1,1); unc(1,2) = -uncertainty(1,0); unc(1,3) = -uncertainty(1,5); unc(1,4) = uncertainty(1,4); unc(1,5) = -uncertainty(1,3);
                    unc(2,0) = uncertainty(0,2); unc(2,1) = -uncertainty(0,1); unc(2,2) = uncertainty(0,0); unc(2,3) = uncertainty(0,5); unc(2,4) = -uncertainty(0,4); unc(2,5) = uncertainty(0,3);
                    unc(3,0) = uncertainty(5,2); unc(3,1) = -uncertainty(5,1); unc(3,2) = uncertainty(5,0); unc(3,3) = uncertainty(5,5); unc(3,4) = -uncertainty(5,4); unc(3,5) = uncertainty(5,3);
                    unc(4,0) = -uncertainty(4,2); unc(4,1) = uncertainty(4,1); unc(4,2) = -uncertainty(4,0); unc(4,3) = -uncertainty(4,5); unc(4,4) = uncertainty(4,4); unc(4,5) = -uncertainty(4,3);
                    unc(5,0) = uncertainty(3,2); unc(5,1) = -uncertainty(3,1); unc(5,2) = uncertainty(3,0); unc(5,3) = uncertainty(3,5); unc(5,4) = -uncertainty(3,4); unc(5,5) = uncertainty(3,3);
                    uncertainty = unc;
                    infoMat = uncertainty.inverse();
                }
                else
                    infoMat.setIdentity();
                EdgeSE3 edge(measurement,infoMat,vertexId2-1,vertexId2);
                if (!graph->addEdgeSE3(edge))
                    std::cout << "error: vertex doesn't exist!\n";
                vertexId2++;
            }
            else{
                std::cout << "could not add edge2\n";
                getchar();
            }
        }
    }
}

void runExperiment2D(int expType, const std::vector<Mat34>& trajectory, const DepthSensorModel& sensorModel, const std::vector<PointCloud>& cloudSeq, const std::vector< std::vector<Mat33> >& uncertaintySet, const std::vector< std::vector<int> >& setIds, Simulator& simulator, TransformEst* transEst){
    Mat34 initPose;
    std::vector<Mat34> trajectorySensor2; initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
    trajectorySensor2.push_back(initPose);
    int vertexId2 = 0;
    Eigen::Vector2d pos2(initPose(0,3), initPose(1,3)); float_type theta = 0;
    VertexSE2 vertex2(vertexId2, pos2, theta);
    if (!graph->addVertexSE2(vertex2))
        std::cout << "error: vertex exists!\n";
    vertexId2++;
    Mat34 sensorPose(initPose);
    Eigen::MatrixXd setA(1, 3);
    Eigen::MatrixXd setB(1, 3);
    Mat33 uncertainty;
    for (int i=1;i<trajectory.size();i++){
        //get point clouds
        //sensorPose.matrix() = trajectory[i].matrix()*sensorModel.config.pose.matrix();

        //match and estimate transformation
        std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
        std::cout << "View points2: " << cloudSeq[i-1].size() << "\n";
        //PointCloud cloudA(cloudSeq[i-1]);
        //PointCloud cloudB(cloudSeq[i]);
        //for (int j=0;j<cloudA.size();j++)  cloudA[j].y= 0;
        //for (int j=0;j<cloudB.size();j++)  cloudB[j].y= 0;

        simulator.matchClouds(cloudSeq[i-1], setA, uncertaintySet[i-1], setAUncertainty, setIds[i-1], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);

        if (setA.rows()>3){
            std::cout << "matched: " << setA.rows() << "\n";
            Mat34 trans = transEst->computeTransformation(setB, setA);

            //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
            sensorPose.matrix() = trajectorySensor2.back().matrix()*trans.matrix();

            //add vertex to the graph
            Eigen::Vector2d pos;
            trajectorySensor2.push_back(sensorPose);
            pos.x() = sensorPose(0,3); pos.y() = sensorPose(1,3);
            Eigen::Vector3d euler = g2o::internal::toEuler(trans.rotation());

            Quaternion q(trans.rotation());
            const double& q0 = q.w();
            const double& q1 = q.x();
            const double& q2 = q.y();
            const double& q3 = q.z();
            double roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)); // r32/r33
            double pitch = asin(2*(q0*q2-q3*q1));
            double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));

            theta += roll;
            VertexSE2 vertexSensor(vertexId2, pos, theta);
            if (!graph->addVertexSE2(vertexSensor))
                std::cout << "error: vertex exists!\n";
            // add edge to the g2o graph
            pos.x() = trans(2,3); pos.y() = -trans(1,3);
            Mat33 infoMat;
            if (expType==0||expType==1)
                infoMat.setIdentity();
            else if (expType==2){
                //uncertainty = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                Mat66 unc = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
                //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                /*for (int k=0;k<setA.rows();k++){
                    std::cout << "matched point: (" << setA(k,0) << "," << setA(k,1) << ") -> (" << setB(k,0) << "," << setB(k,1) << "\n";
                }*/
                uncertainty(0,0) = unc(2,2); uncertainty(0,1) = -unc(2,1); uncertainty(0,2) = unc(2,3);
                uncertainty(1,0) = -unc(1,2); uncertainty(1,1) = unc(1,1); uncertainty(1,2) = -unc(1,3);
                uncertainty(2,0) = unc(3,2); uncertainty(2,1) = -unc(3,1); uncertainty(2,2) = unc(3,3);
                //getchar();
                //uncertainty = transEst->computeUncertainty2D(setA, setAUncertainty, setB, setBUncertainty, trans);
                // [xx  xy  xz  xrx  xry  xrz ]    [yy  yz  yx  yry  yrz  yrx ]
                // [yx  yy  yz  yrx  yry  yrz ] -> [zy  zz  zx  zry  zrz  zrx ]
                // [zx  zy  zz  zrx  zry  zrz ]    [xy  xz  xx  xry  xrz  xrx ]
                // [rxx rxy rxz rxrx rxry rxrz]    [ryy ryz ryx ryry ryrz ryrx]
                // [ryx ryy ryz ryrx ryry ryrz] -> [rzy rzz rzx rzry rzrz rzrx]
                // [rzx rzy rzz rzrx rzry rzrz]    [rxy rxz rxx rxry rxrz rxrx]
                /*Mat66 unc;
                unc(0,0) = uncertainty(1,1); unc(0,1) = 0; unc(0,2) = 0; unc(0,3) = 0; unc(0,4) = 0; unc(0,5) = 0;
                unc(1,0) = 0; unc(1,1) = uncertainty(2,2); unc(1,2) = 0; unc(1,3) = 0; unc(1,4) = 0; unc(1,5) = 0;
                unc(2,0) = 0; unc(2,1) = 0; unc(2,2) = uncertainty(3,3); unc(2,3) = 0; unc(2,4) = 0; unc(2,5) = 0;
                unc(3,0) = 0; unc(3,1) = 0; unc(3,2) = 0; unc(3,3) = 1; unc(3,4) = 0; unc(3,5) = 0;
                unc(4,0) = 0; unc(4,1) = 0; unc(4,2) = 0; unc(4,3) = 0; unc(4,4) = 1; unc(4,5) = 0;
                unc(5,0) = 0; unc(5,1) = 0; unc(5,2) = 0; unc(5,3) = 0; unc(5,4) = 0; unc(5,5) = 1;
                uncertainty = unc;*/
                //std::cout << "uncertainty: \n" << uncertainty << "\n";
                infoMat = uncertainty.inverse();

                /*Quaternion qqq(trans.rotation());
                infoMat(0,0) = infoMat(0,0); infoMat(0,1) = 0; infoMat(0,2) = 0; infoMat(0,3) = 0; infoMat(0,4) = 0; infoMat(0,5) = 0;
                infoMat(1,0) = 0; infoMat(1,1) = infoMat(1,1); infoMat(1,2) = 0; infoMat(1,3) = 0; infoMat(1,4) = 0; infoMat(1,5) = 0;
                infoMat(2,0) = 0; infoMat(2,1) = 0; infoMat(2,2) = infoMat(2,2); infoMat(2,3) = 0; infoMat(2,4) = 0; infoMat(2,5) = 0;
                infoMat(3,0) = 0; infoMat(3,1) = 0; infoMat(3,2) = 0; infoMat(3,3)=1; infoMat(3,4)=0; infoMat(3,5)=0;
                infoMat(4,0) = 0; infoMat(4,1) = 0; infoMat(4,2) = 0; infoMat(4,3)=0; infoMat(4,4)=1; infoMat(4,5)=0;
                infoMat(5,0) = 0; infoMat(5,1) = 0; infoMat(5,2) = 0; infoMat(5,3)=0; infoMat(5,4)=0; infoMat(5,5)=1;
                */
                /*std::cout << "trans: \n" <<  trans.matrix() << "\n";
                std::cout << "uncertainty: \n" <<  uncertainty << "\n";
                std::cout << "info: \n" << infoMat << "\n";
                getchar();*/
            }
            EdgeSE2 edge(pos,roll,infoMat,vertexId2-1,vertexId2);
            if (!graph->addEdgeSE2(edge))
                std::cout << "error: vertex doesn't exist!\n";
            vertexId2++;
        }
        else{
            std::cout << "could not add edge1\n";
            getchar();
        }
    }
    if (expType>0){
        std::cout << "more edges\n";
        //additional edges
        for (int i=2;i<trajectory.size();i++){
            //match and estimate transformation
            for (int j=2;j<18;j++){
                if (i-j<0)
                    break;
                std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
                simulator.matchClouds(cloudSeq[i-j], setA, uncertaintySet[i-j], setAUncertainty, setIds[i-j], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
                int efficientFeatures = 15;
                if (setA.rows()>efficientFeatures){
                    Mat34 trans = transEst->computeTransformation(setB, setA);
                    //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                    // add edge to the g2o graph
                    Eigen::Vector2d pos;
                    pos.x() = trans(2,3); pos.y() = -trans(1,3);
                    Eigen::Matrix<double,3,1> euler = trans.rotation().eulerAngles(2, 1, 0);

                    Mat33 infoMat;
                    if (expType==1)
                        infoMat.setIdentity();
                    else if (expType==2){
                        //uncertainty = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                        //uncertainty = transEst->computeUncertainty2D(setA, setAUncertainty, setB, setBUncertainty, trans);

                        Mat66 unc = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);
                        uncertainty(0,0) = unc(2,2); uncertainty(0,1) = -unc(2,1); uncertainty(0,2) = unc(2,3);
                        uncertainty(1,0) = -unc(1,2); uncertainty(1,1) = unc(1,1); uncertainty(1,2) = -unc(1,3);
                        uncertainty(2,0) = unc(3,2); uncertainty(2,1) = -unc(3,1); uncertainty(2,2) = unc(3,3);
                        //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);

                        infoMat = uncertainty.inverse();
                    }
                    Quaternion q(trans.rotation());
                    const double& q0 = q.w();
                    const double& q1 = q.x();
                    const double& q2 = q.y();
                    const double& q3 = q.z();
                    double roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)); // r32/r33
                    double pitch = asin(2*(q0*q2-q3*q1));
                    double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));

                    /*std::cout << "trans: \n" << trans.matrix() << "\n";
                    std::cout << "euler: \n" << euler << "\n";
                    std::cout << "euler2: \n" << roll << ", " << pitch << "," << yaw << "\n";

                    getchar();*/
                    EdgeSE2 edge(pos,roll,infoMat,i-j,i);
                    if (!graph->addEdgeSE2(edge))
                        std::cout << "error: vertex doesn't exist!\n";
                }
                else{
                    std::cout << "could not add edge2\n";
                    //getchar();
                }
            }
        }
    }
}

void createMap(Graph* graph, const std::vector<Mat34>& trajectory, const std::vector<PointCloud>& cloudSeq, const std::vector< std::vector<int> >& setIds){
    for (int i=0;i<trajectory.size();i++){
        for (int j=0; j<cloudSeq[i].size(); j++){
            Mat34 featurePose;
            Mat34 sensor2feature; sensor2feature.setIdentity();
            sensor2feature(0,3) = cloudSeq[i][j].x; sensor2feature(1,3) = cloudSeq[i][j].y; sensor2feature(2,3) = cloudSeq[i][j].z;
            featurePose.matrix() = trajectory[i].matrix() * sensor2feature.matrix();
            Vec3 posFeature(featurePose(0,3), featurePose(1,3), featurePose(2,3));
            Vertex3D vertexFeature(INIT_VERTEX_ID+setIds[i][j], posFeature);
            if (!graph->addVertexFeature(vertexFeature))
                std::cout << "error: vertex exists!\n";
        }
    }
}

void runExperimentBA(int expType, const std::vector<Mat34>& trajectory, const DepthSensorModel& sensorModel, const std::vector<PointCloud>& cloudSeq, const std::vector< std::vector<Mat33> >& uncertaintySet, const std::vector< std::vector<int> >& setIds, Simulator& simulator, TransformEst* transEst){
    Mat34 initPose;
    std::vector<Mat34> trajectorySensor2; initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
    trajectorySensor2.push_back(initPose);
    int vertexId2 = 0; int vertexId = INIT_VERTEX_ID;
    trajectorySensor2.push_back(initPose);

    Mat34 sensorPose;
    Mat33 uncertainty;
    for (int i=0;i<trajectory.size();i++){
        //get point clouds
        if (i!=0){
            Eigen::MatrixXd setA(1, 3);
            Eigen::MatrixXd setB(1, 3);
            std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
            simulator.matchClouds(cloudSeq[i-1], setA, uncertaintySet[i-1], setAUncertainty, setIds[i-1], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
            Mat34 trans;
            std::cout << i << ".";
            if (setA.rows()>3){
                //std::cout << "matched: " << setA.rows() << "\n";
                trans = transEst->computeTransformation(setB, setA);
                //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                sensorPose.matrix() = trajectorySensor2.back().matrix()*trans.matrix();
            }
            else {
                Mat34 ident; ident.setIdentity();
                trans = ident;
                sensorPose.matrix() = trajectorySensor2.back().matrix()*ident.matrix();
            }
            trajectorySensor2.push_back(sensorPose);
            Vec3 pos;
            pos.x() = sensorPose(0,3); pos.y() = sensorPose(1,3); pos.z() = sensorPose(2,3);
            Quaternion quatSensor(sensorPose.rotation());
            VertexSE3 vertexSensor(vertexId2, pos, quatSensor);
            if (!graph->addVertexPose(vertexSensor))
                std::cout << "error: vertex exists!\n";

            if (expType==2||expType==3||expType==4){
                // add edge to the g2o graph
                pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                Quaternion quatMotion(trans.rotation());
                RobotPose measurement(pos, quatMotion);
                Mat66 infoMat;
                Mat66 uncertaintyPose;
                if (expType==2)
                    infoMat.setIdentity();
                else if (expType==3||expType==4){
                    uncertaintyPose = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                    infoMat = uncertaintyPose.inverse();
                }
                std::cout << "add edge se3\n";
                //getchar();
                EdgeSE3 edge(measurement,infoMat,vertexId2-1,vertexId2);
                if (!graph->addEdgeSE3(edge))
                    std::cout << "error: vertex doesn't exist!\n";
            }
        }
        else {
            sensorPose.matrix() = initPose.matrix();
            Quaternion quatSensor(initPose.rotation());
            Vec3 pos(initPose(0,3), initPose(1,3), initPose(2,3));
            VertexSE3 vertexSensor(vertexId2, pos, quatSensor);
            if (!graph->addVertexPose(vertexSensor))
                std::cout << "error: vertex exists!\n";
        }

        //add features
        //std::cout << "View points1: " << cloudSeq[i].size() << "\n";
        for (int j=0; j<cloudSeq[i].size(); j++){
            Mat34 featurePose;
            Mat34 sensor2feature; sensor2feature.setIdentity();
            sensor2feature(0,3) = cloudSeq[i][j].x; sensor2feature(1,3) = cloudSeq[i][j].y; sensor2feature(2,3) = cloudSeq[i][j].z;
            featurePose.matrix() = sensorPose.matrix() * sensor2feature.matrix();
            Vec3 posFeature(featurePose(0,3), featurePose(1,3), featurePose(2,3));
            Vertex3D vertexFeature(vertexId+setIds[i][j], posFeature);
            if (!graph->addVertexFeature(vertexFeature))
                std::cout << "error: vertex exists!\n";

            //Vec3 pos13(-cloudSeq[i][j].y, -cloudSeq[i][j].z, cloudSeq[i][j].x);
            Vec3 pos13(cloudSeq[i][j].x, cloudSeq[i][j].y, cloudSeq[i][j].z);
            Mat33 infoMat13;
            if (expType==0)
                infoMat13.setIdentity();
            else if (expType==1||expType==2||expType==3||expType==4){
                uncertainty = uncertaintySet[i][j];
                //uncertainty(0,0) = uncertaintySet[i][j](1,1); uncertainty(0,1) = uncertaintySet[i][j](1,2); uncertainty(0,2) = uncertaintySet[i][j](0,1);
                //uncertainty(1,0) = uncertaintySet[i][j](2,1); uncertainty(1,1) = uncertaintySet[i][j](2,2); uncertainty(1,2) = uncertaintySet[i][j](0,2);
                //uncertainty(2,0) = uncertaintySet[i][j](0,1); uncertainty(2,1) = uncertaintySet[i][j](0,2); uncertainty(2,2) = uncertaintySet[i][j](0,0);
                // -y -z x
                // [xx xy xz]    [yy yz yx]
                // [yx yy yz] -> [zy zz zx]
                // [zx zy zz]    [xy xz xx]
                infoMat13 = uncertainty.inverse();
            }
            Edge3D edge(pos13,infoMat13, vertexId2, vertexId+setIds[i][j]);
            if (!graph->addEdge3D(edge))
                std::cout << "error: vertex doesn't exist!\n";
        }
        vertexId2++;
        if ((i%5)==0){
            std::thread tOpt4(optimize,5);
            tOpt4.join();
        }
    }

    if (expType==2||expType==4){
        std::cout << "more edges\n";
        //additional edges
        for (int i=2;i<trajectory.size();i++){
            //match and estimate transformation
            for (int j=2;j<18;j++){
                if (i-j<0)
                    break;
                Eigen::MatrixXd setA(1, 3);
                Eigen::MatrixXd setB(1, 3);
                std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
                simulator.matchClouds(cloudSeq[i-j], setA, uncertaintySet[i-j], setAUncertainty, setIds[i-j], cloudSeq[i], setB, uncertaintySet[i], setBUncertainty, setIds[i]);
                int efficientFeatures = 15;
                if (setA.rows()>efficientFeatures){
                    Mat34 trans = transEst->computeTransformation(setB, setA);
                    //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
                    // add edge to the g2o graph
                    Vec3 pos;
                    pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                    Quaternion quatMotion(trans.rotation());
                    RobotPose measurement(pos, quatMotion);
                    Mat66 infoMat;
                    Mat66 uncertaintyPose;
                    uncertaintyPose = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                    infoMat = uncertaintyPose.inverse();
                    std::cout << "add edge se3\n";
                    //getchar();
                    EdgeSE3 edge(measurement,infoMat,i-j,i);
                    if (!graph->addEdgeSE3(edge))
                        std::cout << "error: vertex doesn't exist!\n";
                    // add edge to the g2o graph
                    /*Vec3 pos;
                    pos.x() = trans(0,3); pos.y() = trans(1,3); pos.z() = trans(2,3);
                    Quaternion quatMotion(trans.rotation());
                    RobotPose measurement(pos, quatMotion);
                    Mat66 infoMat;
                    uncertainty = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
                    infoMat = uncertainty.inverse();
                    EdgeSE3 edge(measurement,infoMat,i-j,i);
                    if (!graph->addEdgeSE3(edge))
                        std::cout << "error: vertex doesn't exist!\n";*/
                }
                else{
                    std::cout << "could not add edge2\n";
                    //getchar();
                }
            }
        }
    }
}

std::vector<float_type> computeRPE(std::vector<Mat34>& trajectoryRef, std::vector<Mat34>& trajectory){
    std::vector<float_type> rpe;
    if (trajectoryRef.size()!=trajectory.size()){
        std::cout << "RPE computation error: trajectories sizes does no match\n";
        return rpe;
    }
    for (int i=1;i<trajectoryRef.size();i++){
        Mat34 error = (trajectoryRef[i-1].inverse()*trajectoryRef[i]).inverse()*(trajectory[i-1].inverse()*trajectory[i]);
        float_type errorVal = pow(error(0,3),2.0)+pow(error(1,3),2.0)+pow(error(2,3),2.0);
        rpe.push_back(errorVal);
    }
    return rpe;
}

float_type computeRMSE(std::vector<float_type>& error){
    float_type rmse=0;
    for (int i=0;i<error.size();i++){
        rmse+=error[i];
    }
    return sqrt(rmse/error.size());
}

float_type computeMean(std::vector<float_type>& error){
    float_type sum = std::accumulate(error.begin(), error.end(), 0.0);
    return sum / error.size();
}

float_type computeStd(std::vector<float_type>& error){
    float_type sum = std::accumulate(error.begin(), error.end(), 0.0);
    float_type mean = sum / error.size();

    float_type sq_sum = std::inner_product(error.begin(), error.end(), error.begin(), 0.0);
    return std::sqrt(sq_sum / error.size() - mean * mean);
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

        //uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
        //std::cout << "uncertainty x y z qx qy qz: \n" << uncertainty << std::endl;
        uncertainty = transEst->computeUncertaintyG2O(setA, setAUncertainty, setB, setBUncertainty, trans);
        std::cout << "uncertainty x y z qx qy qz (directly from quaternions): \n" << uncertainty << std::endl;
        save2file("../../resources/kabsch.m",setA, setB, setTransformed);
        //getchar();

        std::cout << "\n\n\nSingle transformation: room test\n";
        Simulator simulatorTest;
        size_t pointsNo = 50;
        float_type roomDim[3] = {15, 15, 15};
        //simulatorTest.createRoom(pointsNo, roomDim[0], roomDim[1], roomDim[2]);
        simulatorTest.createEnvironment(pointsNo, roomDim[0], roomDim[1], roomDim[2]);
        savePointCloud("../../resources/KabschUncertainty/room.m", simulatorTest.getEnvironment());

        DepthSensorModel sensorModel(configFile);
        Mat34 initPose = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>(0,0,0.0);
        initPose.setIdentity();
        initPose.matrix() *= sensorModel.config.pose.matrix();
        PointCloud cloudA; std::vector<Mat33> uncertaintyCloudA;
        std::vector<int> setAids = simulatorTest.getCloud(initPose, sensorModel, cloudA, uncertaintyCloudA);
        savePointCloud("../../resources/cloudA.m", cloudA, uncertaintyCloudA);

        float_type moveTab[6] = {0.0,0.1,0.0, 0.0,0.0,0.0};//motion in global frame
        Eigen::Quaternion<double> quatMove = quatFromEuler(moveTab[3], moveTab[4], moveTab[5]);
        Mat34 move = quatMove*Eigen::Translation<double,3>(0,0,0);
        PointCloud cloudB; std::vector<Mat33> uncertaintyCloudB;
        std::cout <<"init pose: \n" << initPose.matrix() << "\n";
        Mat34 nextPose = initPose*move;
        nextPose(0,3)+=moveTab[0]; nextPose(1,3)+=moveTab[1]; nextPose(2,3)+=moveTab[2];
        std::cout <<"next pose: \n" << nextPose.matrix() << "\n";
        std::vector<int> setBids = simulatorTest.getCloud(nextPose, sensorModel, cloudB, uncertaintyCloudB);
        savePointCloud("../../resources/cloudB.m", cloudB, uncertaintyCloudB);

        simulatorTest.matchClouds(cloudA, setA, uncertaintyCloudA, setAUncertainty, setAids, cloudB, setB, uncertaintyCloudB, setBUncertainty, setBids);
        trans = transEst->computeTransformation(setB, setA);
        std::cout << "translation in sensor frame: \n" << trans.matrix() << "\n";
        std::cout << "setAuncertainy: " << setAUncertainty[0](0,0) << ", " << setAUncertainty[0](1,1) << ", " << setAUncertainty[0](2,2) << "\n";
        uncertainty = transEst->computeUncertainty(setA, setAUncertainty, setB, setBUncertainty, trans);

        //Mat66 uncertaintySensor = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);

        Mat34 translation = sensorModel.config.pose*trans;
        trans(0,3) = translation(0,3); trans(1,3) = translation(1,3); trans(2,3) = translation(2,3);
        std::cout << "computed transformation: \n" << trans.matrix() << std::endl;
        fi = atan2(trans.matrix()(1,0), trans.matrix()(0,0));
        psi = -asin(trans.matrix()(2,0));
        theta = atan2(trans.matrix()(2,1), trans.matrix()(2,2));
        std::cout << "euler: " << "fi: " << fi << ", psi: " << psi << ",theta: " << theta <<"\n";
        printUncertainty(uncertainty, trans, moveTab[0], moveTab[1], moveTab[2], moveTab[3], moveTab[4], moveTab[5]);

        ///sampling from ellipsoid
        int samplesNo=250;     // How many samples (columns) to draw
        // Define mean and covariance of the distribution
        Eigen::Vector3d mean(3);
        Eigen::MatrixXd covar(3,3);
        mean  <<  0,  0, 0;
        covar <<  4.6845, -1.8587, 1.6523,
                 -1.8587, 1.3192, -0.7436,
                  1.6523, -0.7436, 1.2799;
        //PointCloud ellipsoid;
        //for (int i=0;i<samplesNo;i++)
        //    ellipsoid.push_back(simulatorTest.sampleFromMultivariateGaussian(mean, covar));
        //savePointCloud("../../resources/ellipsoidCloud.m",ellipsoid);


        graph = createPoseGraphG2O(sensorModel.config.pose);
        cout << "Current graph: " << graph->getName() << std::endl;
        //std::string filename1= "../../resources/KabschUncertainty/resultsHelix/graphKabsch_g2o_BAident0.g2o";
        //graph->load(filename1);
        //std::string filename2= "../../resources/KabschUncertainty/resultsHelix/graphKabsch_g2o_BAident0.m";
        //graph->plot2file(filename2);
        //std::cout << "fdf\n"; getchar();

        std::vector<float_type> MSEKabsch;
        std::vector<float_type> MSEBAnouncert;
        std::vector<float_type> MSEBAuncert;
        std::vector<float_type> MSEPerfectTraj;

        int trialsNo =100;
        for (int i=0;i<trialsNo;i++){

            Simulator simulator;
            size_t pointsNo = 5000;
            float_type roomDim[3] = {5.5, 5.5, 5.5};
            //simulator.createRoom(pointsNo, roomDim[0], roomDim[1], roomDim[2]);
            simulator.createEnvironment(1000, 15, 15, 15);
            std::string filenameCloud= "../../resources/KabschUncertainty/refCloud" + std::to_string(i) + ".m";
            savePointCloud(filenameCloud, simulator.getEnvironment());

            std::cout << "\n\n\nTrajectory test\n";
            //create reference trajectory
            std::vector<Mat34> trajectory;
            int motions = 20;
            int motions_rot = 20;
            Mat34 pose = quatFromEuler(0,0,0)*Eigen::Translation<double,3>(0,0,0);
            /*
            ///camera up
            pose(0,0) = 1; pose(0,1) = 0; pose(0,2) = 0;
            pose(1,0) = 0; pose(1,1) = 1; pose(1,2) = 0;
            pose(2,0) = 0; pose(2,1) = 0; pose(2,2) = 1;
            pose(0,3)=(-6.5/2.0)+1.5; pose(1,3)=(-6.5/2.0)+1.5; pose(2,3) = 0.0;
            //pose(0,3)=0; pose(1,3)=(-roomDim[1]/2.0)+1.5; pose(2,3) = -roomDim[2]/2.0;
            trajectory.push_back(pose);
            Mat34 moveRot = quatFromEuler((M_PI/2)/float_type(motions_rot),0,0)*Eigen::Translation<double,3>(0, 0, 0);
            Mat34 moveForward = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>((6.5-3.0)/float_type(motions),0.0, 0.0);
            */
            /// camera in direction of the motion
            pose(0,0) = 0; pose(0,1) = 0; pose(0,2) = 1;
            pose(1,0) = 0; pose(1,1) = -1; pose(1,2) = 0;
            pose(2,0) = 1; pose(2,1) = 0; pose(2,2) = 0;
            pose(0,3)=(-6.5/2.0)+1.5; pose(1,3)=(-6.5/2.0)+1.5; pose(2,3) = 0.0;
            //pose(0,3)=0; pose(1,3)=(-roomDim[1]/2.0)+1.5; pose(2,3) = -roomDim[2]/2.0;
            trajectory.push_back(pose);
            Mat34 moveRot = quatFromEuler(0,0,(M_PI/2)/float_type(motions_rot))*Eigen::Translation<double,3>(0, 0, 0);
            Mat34 moveForward = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>(0,0.0, (6.5-3.0)/float_type(motions));
            for (int j=0;j<4;j++){
                //forward
                for (int i=0;i<motions;i++){
                    pose.matrix() *= moveForward.matrix();
                    trajectory.push_back(pose);
                }
                //rotate pi/2
                for (int i=0;i<motions_rot;i++){
                    pose.matrix() *= moveRot.matrix();
                    trajectory.push_back(pose);
                }
            }
            simulator.loadTrajectory("../../resources/traj_living_room_kt1.txt");
            trajectory = simulator.getTrajectory();

            /*std::vector<Mat34> trajectorySec;
            //rectangular trajectory
            Mat34 poseSec = quatFromEuler(0,0,0)*Eigen::Translation<double,3>(0,0,0);
            poseSec(0,0) = 0; poseSec(0,1) = 0; poseSec(0,2) = 1;
            poseSec(1,0) = 0; poseSec(1,1) = -1; poseSec(1,2) = 0;
            poseSec(2,0) = 1; poseSec(2,1) = 0; poseSec(2,2) = 0;
            poseSec(0,3)=(-6.5/2.0)+1.5; poseSec(1,3)=(-6.5/2.0)+1.5; poseSec(2,3) = 0.0;
            //pose(0,3)=0; pose(1,3)=(-roomDim[1]/2.0)+1.5; pose(2,3) = -roomDim[2]/2.0;
            trajectorySec.push_back(poseSec);
            Mat34 moveRotSec = quatFromEuler(0,0,(M_PI/2)/float_type(motions_rot))*Eigen::Translation<double,3>(0, 0, 0);
            Mat34 moveForwardSec = Eigen::Quaternion<double>(1,0,0,0)*Eigen::Translation<double,3>(0,0.0, (6.5-3.0)/float_type(motions));
            for (int j=0;j<4;j++){
                //forward
                for (int i=0;i<motions;i++){
                    poseSec.matrix() *= moveForwardSec.matrix();
                    trajectorySec.push_back(poseSec);
                }
                //rotate pi/2
                for (int i=0;i<motions_rot;i++){
                    poseSec.matrix() *= moveRotSec.matrix();
                    trajectorySec.push_back(poseSec);
                }
            }*/
            // m 2mcos(2t) -2msin(2t)
            // 0 -sin(2t)  -cos(2t)
            // i    j         k
            // -msin(2t)*k-2mcos^2(2t)*i-2msin^2(2t)*i+mcos(2t)*j
            /*int trajectoryLength = 65;
            for (int j=0;j<trajectoryLength;j++){
                Mat34 pose;
                double t = -3.14+j*(6.28/double(trajectoryLength));
                pose(0,3) = cos(2*t); pose(1,3) = sin(2*t); pose(2,3) = t;
                pose(0,0) = -sin(2*t)/sqrt(5); pose(1,0) = cos(2*t)/sqrt(5); pose(2,0) = +2/sqrt(5);
                pose(0,1) = cos(2*t); pose(1,1) = sin(2*t); pose(2,1) = 0;
                pose(0,2) = (-2*sin(2*t))/sqrt(5); pose(1,2) = (2*cos(2*t))/sqrt(5); pose(2,2) = -1/sqrt(5);

                //pose(0,0) = (-2*sin(2*t))/sqrt(5); pose(1,0) = (2*cos(2*t))/sqrt(5); pose(2,0) = -1/sqrt(5);
                //pose(0,1) = -cos(2*t); pose(1,1) = -sin(2*t); pose(2,1) = 0;
                //pose(0,2) = -sin(2*t)/sqrt(5); pose(1,2) = cos(2*t)/sqrt(5); pose(2,2) = +2/sqrt(5);

                //pose(0,0) = cos(2*t); pose(1,0) = sin(2*t); pose(2,0) = 0;
                //pose(0,1) = (-2*sin(2*t))/sqrt(5); pose(1,1) = (2*cos(2*t))/sqrt(5); pose(2,1) = -1/sqrt(5);
                //pose(0,2) = -sin(2*t)/sqrt(5); pose(1,2) = cos(2*t)/sqrt(5); pose(2,2) = +2/sqrt(5);

                //pose(0,0) = 1; pose(1,0) = 0; pose(2,0) = 0;
                //pose(0,1) = 0; pose(1,1) = 1; pose(2,1) = 0;
                //pose(0,2) = 0; pose(1,2) = 0; pose(2,2) = 1;
                trajectory.push_back(pose);
                //if (j==0){
                //    std::cout << "orientation: \n" << pose.matrix() << "\n";
                //    std::cout << pose.rotation().determinant() << "\n";
                //    getchar();
                //}
            }*/
            saveTrajectory("../../resources/KabschUncertainty/trajectory.m",trajectory, "k");
            std::vector<Mat34> trajectorySens;
            for (int iter = 0; iter<trajectory.size();iter++){
                Mat34 tmppos;
                tmppos.matrix() = trajectory[iter].matrix()*sensorModel.config.pose.matrix();
                trajectorySens.push_back(tmppos);
            }
            saveGroundTruth("../../resources/KabschUncertainty/groundtruth.txt", trajectorySens);

            //move camera along reference trajectory and capture point clouds
            std::vector<Mat34> trajectorySensor; initPose.matrix() = trajectory[0].matrix()*sensorModel.config.pose.matrix();
            trajectorySensor.push_back(initPose);

            std::vector< std::vector<int> > setIds;
            std::vector< std::vector<Mat33> > uncertaintySet;
            std::vector<PointCloud> cloudSeq;

            /*std::vector< std::vector<int> > setIdsSec;
            std::vector<PointCloud> cloudSeqSec;
            std::vector< std::vector<Mat33> > uncertaintySetSec;*/
            simulator.moveCamera(trajectory,sensorModel.config.pose,sensorModel,cloudSeq, setIds, uncertaintySet);
            //simulator.moveCamera(trajectorySec,sensorModel.config.pose,sensorModel,cloudSeqSec, setIdsSec, uncertaintySetSec);
            /*std::cout << "koniec\n";
            getchar();*/
            //std::string filename= "../../resources/KabschUncertainty/trajectorySensor" + std::to_string(i) + ".m";
            //saveTrajectory(filename,trajectorySensor, "r");
            /*std::string filename2 = "../../resources/KabschUncertainty/trajectory2.m";
            saveTrajectory(filename2,trajectorySec, "r");

            graph->clear();
            runExperiment2cameras(1, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, cloudSeqSec, uncertaintySetSec, setIdsSec, simulator, transEst);
            std::vector<Mat34> trajectoryS = graph->getTrajectory();
            std::string filename= "../../resources/KabschUncertainty/trajectorySensor" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryS, "r");

            filename= "../../resources/KabschUncertainty/graphSensor" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            graph->clear();
            runExperiment2cameras(2, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, cloudSeqSec, uncertaintySetSec, setIdsSec, simulator, transEst);
            std::vector<Mat34> trajectory2cam = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectory2cam, "g");

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperiment2cameras(3, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, cloudSeqSec, uncertaintySetSec, setIdsSec, simulator, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_BAident" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt3(optimize,70);//tOpt2(optimizeAndPrune);
            tOpt3.join();
            std::cout << "end optimization Kabsch uncertainty " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_BAident" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryOpt3 = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_BAident" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryOpt3, "b");

            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperiment2cameras(4, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, cloudSeqSec, uncertaintySetSec, setIdsSec, simulator, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_uncertainty" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt4(optimize,70);//tOpt2(optimizeAndPrune);
            tOpt4.join();
            std::cout << "end optimization Kabsch uncertainty " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_uncertainty" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryOpt2 = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_uncertainty" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryOpt2, "k");*/
/*            graph->clear();
            runExperiment2D(0, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);
            std::vector<Mat34> trajectoryS = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectorySensorTest" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryS, "m");

            filename= "../../resources/KabschUncertainty/graphSensor" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            graph->clear();
            runExperiment2D(1, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt(optimize,70);//tOpt(optimizeAndPrune);

            std::cout << "end optimization Kabsch identity " << i << "\n";
            tOpt.join();
            filename= "../../resources/KabschUncertainty/graphKabsch_g2o" + std::to_string(i) + ".g2o";
            graph->save2file(filename);
            //getchar();

            std::vector<Mat34> trajectoryOpt = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryOpt, "g");

            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperiment2D(2, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_uncertainty" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt2(optimize,70);//tOpt2(optimizeAndPrune);
            tOpt2.join();
            std::cout << "end optimization Kabsch uncertainty " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_uncertainty" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryOpt2 = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_uncertainty" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryOpt2, "b");*/

            /// create a map from perfect trajectory
            graph->clear();
            createMap(graph, trajectorySens, cloudSeq, setIds);
            MSEPerfectTraj.push_back(computeMapAccuracy(graph, simulator.getEnvironment()));
            std::cout << "Perfect traj. accuracy: " << computeMapAccuracy(graph, simulator.getEnvironment()) << "\n";

            graph->clear();
            runExperiment(0, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, simulator, transEst);

            trajectorySensor = graph->getTrajectory();
            std::string filename= "../../resources/KabschUncertainty/trajectorySensor" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectorySensor, "r");

            filename= "../../resources/KabschUncertainty/graphSensor" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            createMap(graph, trajectorySensor, cloudSeq, setIds);
            MSEKabsch.push_back(computeMapAccuracy(graph, simulator.getEnvironment()));
            std::cout << "Kabsch accuracy: " << computeMapAccuracy(graph, simulator.getEnvironment()) << "\n";

            /*Mat34 prevPos = trajectorySensor[0];
            for (int tt=0;tt<trajectorySensor.size();tt++){
                Mat34 estimKabsch = prevPos.inverse() * trajectorySensor[tt];
                std::string fileFrame= "../../resources/simulator/frame" + std::to_string(tt) + ".dat";
                simulator.saveImageFeatures(fileFrame, trajectorySens[tt], cloudSeq[tt], setIds[tt], sensorModel, estimKabsch);
                prevPos = trajectorySensor[tt];
            }*/

/*            graph->clear();
            runExperiment(1, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, simulator, transEst);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt(optimize,70);//tOpt(optimizeAndPrune);

            std::cout << "end optimization Kabsch identity " << i << "\n";
            tOpt.join();
            filename= "../../resources/KabschUncertainty/graphKabsch_g2o" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryOpt = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryOpt, "g");
*/
  /*          graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperiment(2, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, simulator, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_uncertainty" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt2(optimize,70);//tOpt2(optimizeAndPrune);
            tOpt2.join();
            std::cout << "end optimization Kabsch uncertainty " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_uncertainty" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryOpt2 = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_uncertainty" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryOpt2, "b");
*/
/*
            //Strasdat
            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperiment(3, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_strasdat" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt3(optimize,70);//tOpt3(optimizeAndPrune);
            tOpt3.join();
            std::cout << "end optimization Strasdat " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_strasdat" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryStrasdat = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_strasdat" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryStrasdat, "c");
*/
            //Bundle Adjustment Identity
            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperimentBA(0, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, simulator, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_BAident" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt4(optimize,70);//tOpt4(optimize,70);
            tOpt4.join();
            std::cout << "end optimization BA identity " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_BAident" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryBAident = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_BAident" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryBAident, "g");

            std::cout << "BA nounc accuracy: " << computeMapAccuracy(graph, simulator.getEnvironment()) << "\n";
            MSEBAnouncert.push_back(computeMapAccuracy(graph, simulator.getEnvironment()));


            //Bundle Adjustment uncert
            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperimentBA(1, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, simulator, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_BAuncert" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt5(optimize,70);
            tOpt5.join();
            std::thread tOpt55(optimize,10);
            tOpt55.join();
            std::thread tOpt56(optimize,5);
            tOpt56.join();
            std::cout << "end optimization BA uncerainty " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_BAuncert" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryBAuncert = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_BAuncert" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryBAuncert, "b");

            std::cout << "BA unc accuracy: " << computeMapAccuracy(graph, simulator.getEnvironment()) << "\n";
            MSEBAuncert.push_back(computeMapAccuracy(graph, simulator.getEnvironment()));

//getchar();
/*
            //Bundle Adjustment + pose ident
            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperimentBA(2, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_BAposeident" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt6(optimize,70);
            tOpt6.join();
            std::cout << "end optimization BA + pose ident " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_BAposeident" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryBAposeident = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_BAposeident" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryBAposeident, "m");

            //Bundle Adjustment + pose uncert
            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperimentBA(3, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_BAposeuncert" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt7(optimize,70);
            tOpt7.join();
            std::cout << "end optimization BA + pose uncert " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_BAposeuncert" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryBAposeuncert = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_BAposeuncert" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryBAposeuncert, "y");

            //Bundle Adjustment + pose uncert + more edges
            graph->clear();
            //move camera along reference trajectory and estimate trajectory
            runExperimentBA(4, trajectory, sensorModel, cloudSeq, uncertaintySet, setIds, transEst);

            filename= "../../resources/KabschUncertainty/init_graph_g2o_BAposeuncertfull" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            //optimize
            std::cout << "optimization\n";
            std::thread tOpt8(optimize,70);
            tOpt8.join();
            std::cout << "end optimization BA + pose uncert full " << i << "\n";

            filename= "../../resources/KabschUncertainty/graphKabsch_g2o_BAposeuncertfull" + std::to_string(i) + ".g2o";
            graph->save2file(filename);

            std::vector<Mat34> trajectoryBAposeuncertfull = graph->getTrajectory();
            filename= "../../resources/KabschUncertainty/trajectory_g2o_BAposeuncertfull" + std::to_string(i) + ".m";
            saveTrajectory(filename,trajectoryBAposeuncertfull, "y");
*/
            /*std::array<float_type, 3> errors;
            std::vector<float_type> errorRPE = computeRPE(trajectory,trajectory);
            errors[0] = computeRMSE(errorRPE); errors[1] = computeMean(errorRPE); errors[2] = computeStd(errorRPE);
            std::cout << "original trajectory error: rpe.rmse " << errors[0] << ", mean " << errors[1] << ", std " << errors[2] << "\n";

            errorRPE = computeRPE(trajectory,trajectorySensor);
            std::cout << "traj size: " << trajectory.size() <<"\n";
            std::cout << "traj sensor size: " << trajectorySensor.size() <<"\n";
            errors[0] = computeRMSE(errorRPE); errors[1] = computeMean(errorRPE); errors[2] = computeStd(errorRPE);
            std::cout << "Sensor trajectory error: rpe.rmse " << errors[0] << ", mean " << errors[1] << ", std " << errors[2] << "\n";

            errorRPE = computeRPE(trajectory,trajectoryOpt);
            errors[0] = computeRMSE(errorRPE); errors[1] = computeMean(errorRPE); errors[2] = computeStd(errorRPE);
            std::cout << "g2o trajectory error: rpe.rmse " << errors[0] << ", mean " << errors[1] << ", std " << errors[2] << "\n";

            errorRPE = computeRPE(trajectory,trajectoryOpt2);
            errors[0] = computeRMSE(errorRPE); errors[1] = computeMean(errorRPE); errors[2] = computeStd(errorRPE);
            std::cout << "g2o with uncertainty trajectory error: rpe.rmse " << errors[0] << ", mean " << errors[1] << ", std " << errors[2] << "\n";
            getchar();*/
        }

        std::cout << "MSE perfect traj for the map: " << computeMean(MSEPerfectTraj) << ", std: " << computeStd(MSEPerfectTraj) << "\n";
        std::cout << "Kabsch MSE for the map: " << computeMean(MSEKabsch) << ", std: " << computeStd(MSEKabsch) << "\n";
        std::cout << "BA nouncert MSE for the map: " << computeMean(MSEBAnouncert) << ", std: " << computeStd(MSEBAnouncert) << "\n";
        std::cout << "BA uncert MSE for the map: " << computeMean(MSEBAuncert) << ", std: " << computeStd(MSEBAuncert) << "\n";

        ofstream myfile ("../../resources/KabschUncertainty/MSE.txt");
          if (myfile.is_open())
          {
            myfile << "MSE perfect traj for the map: " << computeMean(MSEPerfectTraj) << ", std: " << computeStd(MSEPerfectTraj) << "\n";
            myfile << "Kabsch MSE for the map: " << computeMean(MSEKabsch) << ", std: " << computeStd(MSEKabsch) << "\n";
            myfile << "BA nouncert MSE for the map: " << computeMean(MSEBAnouncert) << ", std: " << computeStd(MSEBAnouncert) << "\n";
            myfile << "BA uncert MSE for the map: " << computeMean(MSEBAuncert) << ", std: " << computeStd(MSEBAuncert) << "\n";
            myfile.close();
          }
          else cout << "Unable to open file";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
