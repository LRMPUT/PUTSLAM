#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Tracker/trackerKLT.h"
#include "TransformEst/kabschEst.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>

using namespace std;

auto startT = std::chrono::high_resolution_clock::now();

/// noise: x, y, z, qx, qy, qz
float_type noise[6] = {0.1, 0.01, 0.01, 0.0, 0.000, 0.000};
/// x, y, z, fi, psi, theta
float_type transformation[6] = {0.0, 0.0, -0.0, 0.0, 0.0, -0.0};

void printUncertainty(Mat66& uncertainty, Mat34& trans, float_type fi, float_type psi, float_type theta){
    std::cout << "uncertainty: \n" << uncertainty << std::endl;
    std::cout << "error x: " << fabs(trans.translation()(0)-transformation[0]) << ", 3*sigma = " << 3*uncertainty(3,3);
    if (3*uncertainty(3,3) < fabs(trans.translation()(0)-transformation[0]) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error y: " << fabs(trans.translation()(1)-transformation[1]) << ", 3*sigma = " << 3*uncertainty(4,4);
    if (3*uncertainty(4,4) < fabs(trans.translation()(1)-transformation[1]) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error z: " << fabs(trans.translation()(2)-transformation[2]) << ", 3*sigma = " << 3*uncertainty(5,5);
    if (3*uncertainty(5,5) < fabs(trans.translation()(2)-transformation[2]) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error fi: " << fabs(fi-transformation[3]) << ", 3*sigma = " << 3*uncertainty(0,0);
    if (3*uncertainty(0,0) < fabs(fi-transformation[3]) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error psi: " << fabs(psi-transformation[4]) << ", 3*sigma = " << 3*uncertainty(1,1);
    if (3*uncertainty(1,1) < fabs(psi-transformation[4]) ) std::cout << " alert!\n"; else std::cout << "\n";
    std::cout << "error theta: " << fabs(theta-transformation[5]) << ", 3*sigma = " << 3*uncertainty(2,2);
    if (3*uncertainty(2,2) < fabs(theta-transformation[5]) ) std::cout << " alert!\n"; else std::cout << "\n";
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

void generateSetpoint(Eigen::MatrixXd& setA, size_t numPoints, std::default_random_engine& generator){
    float_type center[3]={0.0,0.0,0.0};
    std::uniform_real_distribution<double> distribution(-1.5,1.5);
    for (size_t i = 0; i<numPoints; i++){
        setA(i,0) = center[0] + distribution(generator);
        setA(i,1) = center[1] + distribution(generator);
        setA(i,2) = center[2] + distribution(generator);
    }
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

        // create kabsch transform estimator
        TransformEst* transEst = createKabschEstimator();

        size_t numPoints = 100;
        Eigen::MatrixXd setA(numPoints, 3);
        Eigen::MatrixXd setB(numPoints, 3);

        std::default_random_engine generator((unsigned int)time(0));
        generateSetpoint(setA, numPoints, generator);

        std::cout << "Mean transformation is: x=" << transformation[0] << ", y=" << transformation[1] << ", z=" << transformation[2];
        std::cout << ", fi=" << transformation[3] << ", psi=" << transformation[4] << ", theta=" << transformation[5];
        Eigen::Quaternion<double> quat;
        Eigen::AngleAxis<double> rotX(transformation[3], Eigen::Vector3d::UnitZ());
        Eigen::AngleAxis<double> rotY(transformation[4], Eigen::Vector3d::UnitY());
        Eigen::AngleAxis<double> rotZ(transformation[5], Eigen::Vector3d::UnitX());
        quat = rotX * rotY * rotZ;
        std::cout << ", qw=" << quat.w() << ", qx=" << quat.x() << ", qy=" << quat.y() << ", qz=" << quat.z() << "\n";

        std::normal_distribution<double> normDistributionX(0.0,noise[0]);
        std::normal_distribution<double> normDistributionY(0.0,noise[1]);
        std::normal_distribution<double> normDistributionZ(0.0,noise[2]);
        std::normal_distribution<double> normDistributionQX(0.0,noise[3]);
        std::normal_distribution<double> normDistributionQY(0.0,noise[4]);
        std::normal_distribution<double> normDistributionQZ(0.0,noise[5]);

        std::vector<Mat33> setAUncertainty; std::vector<Mat33> setBUncertainty;
        for (size_t i = 0; i<numPoints; i++){
            Eigen::Quaternion<double> q;
            Eigen::AngleAxis<double> aaX(transformation[3], Eigen::Vector3d::UnitZ());
            Eigen::AngleAxis<double> aaY(transformation[4], Eigen::Vector3d::UnitY());
            Eigen::AngleAxis<double> aaZ(transformation[5], Eigen::Vector3d::UnitX());
            q = aaX * aaY * aaZ;
            /*q.x() += normDistributionQX(generator);
            q.y() += normDistributionQY(generator);
            q.y() += normDistributionQZ(generator);
            q.normalize();*/

            Eigen::Transform< double, 3, Eigen::Affine > transform = q * Eigen::Translation<double,3>(transformation[0],transformation[1],transformation[2]);
            Vec3 point(setA(i,0), setA(i,1), setA(i,2));
            point.vector() = transform*point.vector();
            setB(i,0) = point.x()+normDistributionX(generator); setB(i,1) = point.y()+normDistributionY(generator); setB(i,2) = point.z()+normDistributionZ(generator);

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
        printUncertainty(uncertainty, trans, fi, psi, theta);

        Eigen::MatrixXd setTransformed(numPoints, 3);
        for (int i=0;i<setA.rows();i++){
            Vec3 point(setA(i,0), setA(i,1), setA(i,2));
            point.vector() = trans*point.vector();
            setTransformed(i,0) = point.x(); setTransformed(i,1) = point.y(); setTransformed(i,2) = point.z();
        }

        uncertainty = transEst->ConvertUncertaintyEuler2quat(uncertainty, trans);
        std::cout << "uncertainty x y z qx qy qz: \n" << uncertainty << std::endl;
        save2file("../../resources/kabsch.m",setA, setB, setTransformed);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
