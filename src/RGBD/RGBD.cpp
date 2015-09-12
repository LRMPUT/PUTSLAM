/** @file RGBD.cpp
 *
 * \brief The methods, which might be useful when dealing with RGBD data
 * \author Michal Nowicki
 *
 */
#include "../include/RGBD/RGBD.h"

int RGBD::roundSize(double x, int size) {
	if (x < 0)
		x = 0;
	else if (x > size - 1)
		x = size;
	return round(x);
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
		std::vector<cv::Point2f> undistortedFeatures2D, cv::Mat depthImage, double depthImageScale) {

	// Assume standard distortion
	float cameraMatrix[3][3] = { { 517.3, 0, 318.6 }, { 0, 516.5, 255.3 }, { 0,
			0, 1 } };

	// Call method with additional parameters
	return keypoints2Dto3D(undistortedFeatures2D, depthImage,
			cv::Mat(3, 3, CV_32FC1, &cameraMatrix), depthImageScale);
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
		std::vector<cv::Point2f> undistortedFeatures2D, cv::Mat depthImage,
		cv::Mat cameraMatrix, double depthImageScale, int startingID) {

	// Lets create 3D points
	std::vector<Eigen::Vector3f> features3D(undistortedFeatures2D.size() - startingID);
	int i = 0;
	for (std::vector<cv::Point2f>::iterator it = undistortedFeatures2D.begin() + startingID;
			it != undistortedFeatures2D.end(); ++it) {

		features3D[i] = point2Dto3D(*it, depthImage, cameraMatrix, depthImageScale);
		i++;
	}

	return features3D;
}

Eigen::Vector3f RGBD::point2Dto3D(cv::Point2f feature2D,
		cv::Mat depthImage, cv::Mat cameraMatrix, double depthImageScale) {

	// Feature are extracted with subpixel precision, so find closest pixel
	int uRounded = roundSize(feature2D.x, depthImage.cols);
	int vRounded = roundSize(feature2D.y, depthImage.rows);

	// Convert it using the scaling of depth image
	float Z = depthImage.at<uint16_t>(vRounded, uRounded) / depthImageScale;

	// Compute the feature position in normalized image coordinates
	float u = (feature2D.x - cameraMatrix.at<float>(0, 2))
			/ cameraMatrix.at<float>(0, 0);
	float v = (feature2D.y - cameraMatrix.at<float>(1, 2))
			/ cameraMatrix.at<float>(1, 1);

	// Create 3D feature
	return Eigen::Vector3f(u * Z, v * Z, Z);
}

Eigen::Vector3f RGBD::point2Dto3D(cv::Point2f feature2D,
		float depth, cv::Mat cameraMatrix) {

	// Feature are extracted with subpixel precision, so find closest pixel
	int uRounded = round(feature2D.x);
	int vRounded = round(feature2D.y);

	// Compute the feature position in normalized image coordinates
	float u = (feature2D.x - cameraMatrix.at<float>(0, 2))
			/ cameraMatrix.at<float>(0, 0);
	float v = (feature2D.y - cameraMatrix.at<float>(1, 2))
			/ cameraMatrix.at<float>(1, 1);

	// Create 3D feature
	return Eigen::Vector3f(u * depth, v * depth, depth);
}

// Project 3D points onto images
std::vector<cv::Point2f> RGBD::points3Dto2D(std::vector<Eigen::Vector3f> features3D, cv::Mat cameraMatrix) {

	std::vector<cv::Point2f> features2D(features3D.size());
	int i=0;
	for (std::vector<Eigen::Vector3f>::iterator it = features3D.begin(); it!=features3D.end(); ++it, i++)
	{
		features2D[i] = RGBD::point3Dto2D(*it, cameraMatrix);
	}
	return features2D;
}

cv::Point2f RGBD::point3Dto2D(Eigen::Vector3f feature3D, cv::Mat cameraMatrix) {
	float u = feature3D.x() * cameraMatrix.at<float>(0, 0) / feature3D.z()
			+ cameraMatrix.at<float>(0, 2);
	float v = feature3D.y() * cameraMatrix.at<float>(1, 1) / feature3D.z()
			+ cameraMatrix.at<float>(1, 2);
	return cv::Point2f(u, v);
}

///compute normal
Vec3 RGBD::computeNormal(const cv::Mat& depthImage, int u, int v, const cv::Mat& cameraMatrix, double depthImageScale){
    cv::Point2f center2D(u,v);
    Eigen::Vector3f center = point2Dto3D(center2D,depthImage, cameraMatrix, depthImageScale);
    std::vector<Vec3> vecs;
    std::vector<int> uidx = {-1, -1, -1, 0, 1, 1, 1, 0};
    std::vector<int> vidx = {-1, 0, 1, 1, 1, 0, -1, -1};
    //std::cout << "center <<" << center(0) << ", " << center(1) << ", "<< center(2) << "\n";
    for(int i=0;i<uidx.size();i++){
            Eigen::Vector3f pointEnd = point2Dto3D(cv::Point2f(u+uidx[i], v+vidx[i]),depthImage, cameraMatrix, depthImageScale);
            if (pointEnd(2)>0){
                vecs.push_back(Vec3(pointEnd(0)-center(0),pointEnd(1)-center(1),pointEnd(2)-center(2)));
            }
            //std::cout << "plot3(" << pointEnd(0) << ", " << pointEnd(1) << ", "<< pointEnd(2) << ",'ro');\n";
            //std::cout << "plot3([" << pointEnd(0) << ", " << center(0) <<"], ["<< pointEnd(1) << ", " << center(1) <<"], ["<< pointEnd(2) << ", " << center(2) <<"],"<< "'-r');\n";
            //std::cout << "vec <<" << vecs.back().x() << ", " << vecs.back().y() << ", "<< vecs.back().z() << "\n";
    }
    //compute normals
    std::vector<Vec3> normals;
    for (int i = 0; i<vecs.size();i++){
        if (i==vecs.size()-1){
            Vec3 norm(vecs[i].vector().cross(vecs[0].vector()));
            //std::cout << "plot3([" << center(0) << ", " << center(0)+30*norm.x() <<"], ["<< center(1) << ", " << center(1)+30*norm.y() <<"], ["<< center(2) << ", " << center(2)+30*norm.z() <<"],"<< "'-g');\n";
            normals.push_back(norm);
        }
        else{
            Vec3 norm(vecs[i].vector().cross(vecs[i+1].vector()));
            normals.push_back(norm);
            //std::cout << "plot3([" << center(0) << ", " << center(0)+30*norm.x() <<"], ["<< center(1) << ", " << center(1)+30*norm.y() <<"], ["<< center(2) << ", " << center(2)+30*norm.z() <<"],"<< "'-g');\n";
        }
        //std::cout << "norm <<" << normals.back().x() << ", " << normals.back().y() << ", "<< normals.back().z() << "\n";
    }
    Vec3 normal;
    // compute average normal
    float_type sumX=0, sumY=0, sumZ=0;
    for (auto it=normals.begin();it!=normals.end();it++){
        sumX+=(*it).x(); sumY+=(*it).y(); sumZ+=(*it).z();
    }
    normal.x()=sumX/normals.size();
    normal.y()=sumY/normals.size();
    normal.z()=sumZ/normals.size();
    float_type norm = normal.vector().norm();
    normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
    return normal;
}

//compute rgb gradient (min)
Vec3 RGBD::computeRGBGradient(const cv::Mat& rgbImage,
		const cv::Mat& depthImage, int u, int v, const cv::Mat& cameraMatrix,
		double depthImageScale) {
	Vec3 grad;
	cv::Mat grayframe;
	cv::cvtColor(rgbImage, grayframe, CV_RGB2GRAY);
    float_type gradx; float_type grady;
    if ((u-1>0)&&(v-1>0)&&(u+1<rgbImage.cols)&&(v+1<rgbImage.rows)){
        cv::Mat patch = cv::Mat(rgbImage, cv::Rect(u-1,v-1,3,3));
        gradx = -3*patch.at<uint16_t>(0,0)-10*patch.at<uint16_t>(0,1)-3*patch.at<uint16_t>(0,2) +
                +3*patch.at<uint16_t>(2,0)+10*patch.at<uint16_t>(2,1)+3*patch.at<uint16_t>(2,2);
        grady = -3*patch.at<uint16_t>(0,0)-10*patch.at<uint16_t>(1,0)-3*patch.at<uint16_t>(2,0) +
                +3*patch.at<uint16_t>(0,2)+10*patch.at<uint16_t>(1,2)+3*patch.at<uint16_t>(2,2);
    }
    else {
        return Vec3(1, 1, 1);
    }
    float_type angle = atan2(grady, gradx) + (M_PI/2.0);
    int coord1[2]={int(sqrt(2)*sin(angle)), int(sqrt(2)*cos(angle))};
    int coord2[2]={int(sqrt(2)*sin(angle+M_PI)), int(sqrt(2)*cos(angle+M_PI))};
    Eigen::Vector3f pointCenter = point2Dto3D(cv::Point2f(u, v),depthImage, cameraMatrix, depthImageScale);
    Eigen::Vector3f pointEnd = point2Dto3D(cv::Point2f(u+coord1[0], v+coord1[1]),depthImage, cameraMatrix, depthImageScale);
    Eigen::Vector3f pointBeg = point2Dto3D(cv::Point2f(u+coord2[0], v+coord2[1]),depthImage, cameraMatrix, depthImageScale);
    if (pointEnd(2)>0&&pointBeg(2)){
        grad = Vec3(pointEnd(0)- pointBeg(0), pointEnd(1)- pointBeg(1), pointEnd(2)- pointBeg(2));
    }
    else {
        if (pointCenter(2)>0&&pointBeg(2)){
            grad = Vec3(pointCenter(0)- pointBeg(0), pointCenter(1)- pointBeg(1), pointCenter(2)- pointBeg(2));
        }
        else if (pointCenter(2)>0&&pointEnd(2)){
            grad = Vec3(pointEnd(0)- pointCenter(0), pointEnd(1)- pointCenter(1), pointEnd(2)- pointCenter(2));
        }
        else{
            grad = Vec3(coord1[0], coord1[1], 0);
        }
    }
    float_type norm = grad.vector().norm();
    grad.x() /= norm;    grad.y() /= norm;    grad.z() /= norm;
    return grad;
}

void RGBD::removeFeaturesWithoutDepth(std::vector<cv::KeyPoint> &features,
		cv::Mat depthImage) {
	// Lambda expression
	auto it =
			std::remove_if(features.begin(), features.end(),
					[depthImage](cv::KeyPoint kp) {
						if (depthImage.at<uint16_t>(kp.pt)  > 0.0) {
							return false;
						}
						return true;
					});
	features.erase(it, features.end());
}

void RGBD::removeMapFeaturesWithoutDepth(std::vector<MapFeature> &features,
		cv::Mat depthImage, float additionalDistance,
		std::vector<int> &frameIds, std::vector<float_type> &angles,
		double depthImageScale) {

	std::vector<MapFeature>::iterator featuresIter = features.begin();
	std::vector<int>::iterator frameIdsIter = frameIds.begin();
	std::vector<float_type>::iterator anglesIter = angles.begin();

	for (;featuresIter!=features.end();)
    {
		int uRounded = roundSize(featuresIter->u, depthImage.cols);
		int vRounded = roundSize(featuresIter->v, depthImage.rows);

		if (depthImage.at<uint16_t>(cv::Point2f(uRounded, vRounded))
				/ depthImageScale
				<= featuresIter->position.z() - additionalDistance) {
			featuresIter = features.erase(featuresIter);
			frameIdsIter = frameIds.erase(frameIdsIter);
			anglesIter = angles.erase(anglesIter);
		}
        else {
			++featuresIter;
			++frameIdsIter;
			++anglesIter;
		}
	}
}

std::vector<cv::Point2f> RGBD::removeImageDistortion(
		std::vector<cv::KeyPoint>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs) {

	// Check if the vector is not empty
	if (features.size() == 0)
		return std::vector<cv::Point2f>();

	// Convert to points2D and then to Mat
	std::vector<cv::Point2f> points2D;
	cv::KeyPoint::convert(features, points2D);
	cv::Mat pointsDisorted(points2D), pointsUndistorted;

	// Undistortion
	cv::undistortPoints(pointsDisorted, pointsUndistorted, cameraMatrix,
			distCoeffs);

	std::vector<cv::Point2f> returnVector;
	for (int i = 0; i < pointsUndistorted.rows; i++) {

		// u = (u_normalized*fx) + cx
		float u = pointsUndistorted.at<cv::Vec2f>(i)[0]
				* cameraMatrix.at<float>(0, 0) + cameraMatrix.at<float>(0, 2);

		// v = (v_normalized*fy) + cy
		float v = pointsUndistorted.at<cv::Vec2f>(i)[1]
				* cameraMatrix.at<float>(1, 1) + cameraMatrix.at<float>(1, 2);
		returnVector.push_back(cv::Point2f(u, v));
	}
	return returnVector;
}

std::vector<cv::Point2f> RGBD::removeImageDistortion(
		std::vector<cv::Point2f>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs) {

	// Check if the vector is not empty
	if (features.size() == 0)
		return std::vector<cv::Point2f>();

	// Initialize
	cv::Mat pointsDisorted(features), pointsUndistorted;

	// Undistortion
	cv::undistortPoints(pointsDisorted, pointsUndistorted, cameraMatrix,
			distCoeffs);

	std::vector<cv::Point2f> returnVector;
	for (int i = 0; i < pointsUndistorted.rows; i++) {

		// u = (u_normalized*fx) + cx
		float u = pointsUndistorted.at<cv::Vec2f>(i)[0]
				* cameraMatrix.at<float>(0, 0) + cameraMatrix.at<float>(0, 2);

		// v = (v_normalized*fy) + cy
		float v = pointsUndistorted.at<cv::Vec2f>(i)[1]
				* cameraMatrix.at<float>(1, 1) + cameraMatrix.at<float>(1, 2);
		returnVector.push_back(cv::Point2f(u, v));
	}
	return returnVector;
}

std::vector<Eigen::Vector3f> RGBD::imageToPointCloud(cv::Mat rgbImage,
		cv::Mat depthImage, cv::Mat cameraMatrix, Eigen::Matrix4f pose, double depthImageScale) {
	std::vector<Eigen::Vector3f> pointCloud;

	for(int j = 0;j < rgbImage.rows;j++){
	    for(int i = 0;i < rgbImage.cols;i++){

	    	float depth = depthImage.at<uint16_t>(cv::Point2f(i,j)) / depthImageScale;

	    	Eigen::Vector3f point3D = point2Dto3D(cv::Point2f(i,j),
	    			depth, cameraMatrix);

	    	point3D = pose.block<3,3>(0,0) * point3D + pose.block<3,1>(0,3);

	    	if ( point3D.z() > 0.0001) {
	    		pointCloud.push_back(point3D);
	    	}
	    }
	}
	return pointCloud;
}

void RGBD::saveToFile(std::vector<Eigen::Vector3f> pointCloud, std::string fileName, bool first, Eigen::Matrix4f tmpPose)
{
	std::ofstream fileToSave;

	if (first) {
		fileToSave.open(fileName);
		fileToSave << "NODE 0.0 0.0 0.0 0.0 0.0 0.0" << std::endl;
	}
	else {
		fileToSave.open(fileName,  std::ofstream::out | std::ofstream::app);
	}
	std::cout << "Writing NODE" << std::endl;

	// OCTOMAP:
	// he keyword NODE is followed by the 6D pose of the laser origin of the 3D scan
	//(coordinates are regarded as SI units: meter for translation & rad for angles. x points forward, y left, z up.
	//roll, pitch, and yaw angles are around the axes x, y, z respectively).
	Eigen::Vector3f eulerAnglesRPY = tmpPose.block<3,3>(0,0).eulerAngles(0, 1, 2);

//	fileToSave << "NODE " << tmpPose(0, 3) << " " << tmpPose(1, 3) << " "
//			<< tmpPose(2, 3) << " " << eulerAnglesRPY(0) << " "
//			<< eulerAnglesRPY(1) << " " << eulerAnglesRPY(2) << std::endl;

	for (int i=0;i<pointCloud.size();i++) {
		fileToSave << pointCloud[i].x() << " " << pointCloud[i].y() << " " << pointCloud[i].z() << std::endl;
	}
	fileToSave.close();
}
