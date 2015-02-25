#include "../include/Grabber/depthSensorModel.h"

/// Construction
DepthSensorModel::DepthSensorModel(std::string configFile) : config(configFile){
    PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],
                  0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1],
                  0,0,1;
    Ruvd << config.varU, 0, 0,
            0, config.varV, 0,
            0, 0, 0;
}

void DepthSensorModel::getPoint(uint_fast16_t u, uint_fast16_t v, float_type depth, Eigen::Vector3d& point3D){
    Eigen::Vector3d point(u, v, 1);
    point3D = depth*PHCPModel*point;
}

Eigen::Vector3d DepthSensorModel::inverseModel(float_type x, float_type y,
		float_type z) const {
    Eigen::Vector3d point(((config.focalLength[0]*x)/z)+config.focalAxis[0], ((config.focalLength[1]*y)/z)+config.focalAxis[1], z);
    if (point(0)<0||point(0)>config.imageSize[0]||point(1)<0||point(1)>config.imageSize[1]||z<0.8||z>6.0){
        point(0) = -1; point(1) = -1; point(2) = -1;
    }
    return point;
}

/// u,v [px], depth [m]
void DepthSensorModel::computeCov(uint_fast16_t u, uint_fast16_t v, float_type depth, Mat33& cov) {
    //float_type dispDer = config.k3 * 1/(config.k2*pow(cos((disparity/config.k2) + config.k1),2.0));
    Mat33 J;
    J << depth/config.focalLength[0], 0, ((u/config.focalLength[0])-(config.focalAxis[0]/config.focalLength[0])),
         0, depth/config.focalLength[1], ((v/config.focalLength[1])-(config.focalAxis[1]/config.focalLength[1])),
         0, 0, 1;
    Ruvd(2,2) = (config.distVarCoefs[0]*pow(depth,3.0) + config.distVarCoefs[1]*pow(depth,2.0) + config.distVarCoefs[2]*depth + config.distVarCoefs[3])/3.0;
    cov=J*Ruvd*J.transpose();
}

/// compute information matrix
Mat33 DepthSensorModel::informationMatrix(float_type x, float_type y, float_type z){
    Mat33 info;
	Eigen::Vector3d cam3D = inverseModel(x, y, z);
	computeCov(cam3D(0), cam3D(1), cam3D(2), info);
    return info.inverse();
}

Mat33 DepthSensorModel::informationMatrixFromImageCoordinates(float_type u, float_type v, float_type z) {
    Mat33 info;
    computeCov(u, v, z, info);
    return info.inverse();
}
