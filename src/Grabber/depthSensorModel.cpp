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

void DepthSensorModel::getPoint(uint_fast16_t u, uint_fast16_t v, float_type depth, Eigen::Vector3d& point3D) const{
    Eigen::Vector3d point((double)u,(double)v, 1.0);
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
    J << depth/config.focalLength[0], 0.0, (((double)u/config.focalLength[0])-(config.focalAxis[0]/config.focalLength[0])),
         0.0, depth/config.focalLength[1], (((double)v/config.focalLength[1])-(config.focalAxis[1]/config.focalLength[1])),
         0.0, 0.0, 1.0;
    Ruvd(2,2) = config.distVarCoefs[0]*pow(depth,3.0) + config.distVarCoefs[1]*pow(depth,2.0) + config.distVarCoefs[2]*depth + config.distVarCoefs[3];
    cov=J*Ruvd*J.transpose();
}

/// point xyz in camera frame
void DepthSensorModel::computeCov(Eigen::Vector3f point, Mat33& cov){
    Eigen::Vector3d imageCoordinates = inverseModel(point.x(), point.y(), point.z());
    if (imageCoordinates.x()==-1)
        cov.setZero();
    else
        computeCov((unsigned long int)imageCoordinates.x(), (unsigned long int)imageCoordinates.y(), imageCoordinates.z(),cov);
}

/// compute information matrix
Mat33 DepthSensorModel::informationMatrix(float_type x, float_type y, float_type z){
    Mat33 info;
	Eigen::Vector3d cam3D = inverseModel(x, y, z);
	computeCov((unsigned long int)cam3D(0), (unsigned long int)cam3D(1), cam3D(2), info);
    return info.inverse();
}

Mat33 DepthSensorModel::informationMatrixFromImageCoordinates(float_type u, float_type v, float_type z) {
    Mat33 cov;
    computeCov((unsigned long int)u, (unsigned long int)v, z, cov);
    return cov.inverse();
}

/// compute uncertainty from normal vector
Mat33 DepthSensorModel::uncertinatyFromNormal(const Vec3& _normal){
    Mat33 cov; Vec3 x(1,0,0); Vec3 y;
    Vec3 normal(_normal);
    y.vector() = normal.vector().cross(x.vector());
    normalizeVector(normal);
    x.vector() = y.vector().cross(normal.vector());
    normalizeVector(x);
    Mat33 S(Mat33::Identity()); S(2,2)=config.scaleUncertaintyNormal;
    Mat33 R;
    R.block(0,0,3,1) = x.vector();
    R.block(0,1,3,1) = y.vector();
    R.block(0,2,3,1) = normal.vector();
    cov = ((R*S)*S)*R.inverse();
    return cov;
}

/// compute uncertainty from rgb gradient vector
Mat33 DepthSensorModel::uncertinatyFromRGBGradient(const Vec3& grad){
    //std::cout << "grad\n" << grad.vector() << "\n";
    Mat33 cov; Vec3 z(0,0,1); Vec3 y;
    y.vector() = z.vector().cross(grad.vector());
    normalizeVector(y);
    z.vector() = y.vector().cross(grad.vector());
    normalizeVector(z);
    Mat33 S(Mat33::Identity()); S(1,1)=config.scaleUncertaintyGradient; S(1,1)=config.scaleUncertaintyGradient;
    Mat33 R;
    R.block(0,0,3,1) = grad.vector();
    R.block(0,1,3,1) = y.vector();
    R.block(0,2,3,1) = z.vector();
    cov = ((R*S)*S)*R.inverse();
    //std::cout << "cov: \n" << cov <<"\n";
    //getchar();
    return cov;
}

/// normalize vector
void DepthSensorModel::normalizeVector(Vec3& normal) const {
    float_type norm = normal.vector().norm();
    normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
}

/// Create point cloud from current RGB and depth image
void DepthSensorModel::convert2cloud(const cv::Mat& color, const cv::Mat& depth, PointCloud& cloud) {
    cloud.clear();
    for (int i=0;i<color.rows;i++){
        for (int j=0;j<depth.cols;j++){
            const cv::Point3_ <uchar>* p = color.ptr<cv::Point3_<uchar> >(i,j);
            putslam::Point3D point;
            Eigen::Vector3d pointxyz;
            getPoint(j,i, (double)depth.at<uint16_t>(i, j)/config.depthImageScale, pointxyz);//5000 - depth scale
            point.x = pointxyz(0); point.y = pointxyz(1); point.z = pointxyz(2);
            point.r = p->z; point.g = p->y; point.b = p->x;
            cloud.push_back(point);
            //getchar();
        }
    }
}
