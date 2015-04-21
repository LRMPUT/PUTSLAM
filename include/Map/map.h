/** @file map.h
 *
 * Environment Map interface
 *
 */

#ifndef _MAP_H_
#define _MAP_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>
#include "../include/Grabber/depthSensorModel.h"

namespace putslam {

/// Map interface
class Map {
public:

	/// Map type
	enum Type {
		/// Features Map + G2O
		MAP_FEATURES,
	};

	/// overloaded constructor
	Map(const std::string _name, Type _type) :
			name(_name), type(_type) {
	}
	;

	/// Name of the map
	virtual const std::string& getName() const = 0;

	/// Virtual descrutor
	virtual ~Map() {
	}

	/// get all visible features
	virtual std::vector<MapFeature> getVisibleFeatures(
			const Mat34& cameraPose) = 0;
    /// get all visible features and reduce results
    virtual std::vector<MapFeature> getVisibleFeatures(
            const Mat34& cameraPose, int graphDepthThreshold, float_type distanceThreshold) = 0;

    /// find nearest id of the image frame taking into acount the current angle of view and the view from the history
    virtual void findNearestFrame(const std::vector<MapFeature>& features, std::vector<int>& imageIds, std::vector<float_type>& angles, float_type maxAngle = 3.14) = 0;

    /// removes features which are too far from current camera pose (distant in graph)
    virtual void removeDistantFeatures(std::vector<MapFeature>& mapFeatures, int graphDepthThreshold = 0, float_type distanceThreshold = 0) = 0;

	/// Add NEW features and a NEW camera pose (initial guess) to the map
	/// Position of features in relation to camera pose
	virtual void addFeatures(const std::vector<RGBDFeature>& features,
			int poseId = -1) = 0;

	/// add measurements (features measured from the last camera pose)
	virtual void addMeasurements(const std::vector<MapFeature>& features,
			int poseId = -1) = 0;

	/// add measurement between two poses
	virtual void addMeasurement(int poseFrom, int poseTo,
			Mat34 transformation) = 0;

	/// add new pose of the camera, returns id of the new pose
    virtual int addNewPose(const Mat34& cameraPoseChange, float_type timestamp, cv::Mat image = cv::Mat(), cv::Mat depthImage = cv::Mat()) = 0;

	/// Get all features
	virtual std::vector<MapFeature> getAllFeatures(void) = 0;

	/// Get feature position
	virtual Vec3 getFeaturePosition(unsigned int id) = 0;

	/// get pose of the sensor (default: last pose)
	virtual Mat34 getSensorPose(int poseId = -1) = 0;

	// get number of poses stored in map
	virtual int getPoseCounter() = 0;

	/// getDepthSensorModel
	virtual DepthSensorModel getDepthSensorModel() = 0;

	/// start optimization thread
    virtual void startOptimizationThread(unsigned int iterNo, int verbose = 0, std::string RobustKernelName = "", float_type kernelDelta = 0) = 0;

    /// start map management thread
    virtual void startMapManagerThread(int verbose = 0) = 0;

	/// Wait for optimization thread to finish
	virtual void finishOptimization(std::string trajectoryFilename,
			std::string graphFilename) = 0;

    virtual /// Export graph and trajectory
    void exportOutput(std::string trajectoryFilename,
            std::string graphFilename) = 0;
    /// Wait for map management thread to finish
    virtual void finishManagementThr(void) = 0;

    /// Save map to file
    virtual void save2file(std::string mapFilename, std::string graphFilename) = 0;

    /// set Robust Kernel
    virtual void setRobustKernel(std::string name, float_type delta) = 0;

    /// disable Robust Kernel
    virtual void disableRobustKernel(void) = 0;

    /// get n-th image and depth image from the sequence
    virtual void getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage) = 0;

protected:
	/// Map type
	Type type;

	/// Map name
	const std::string name;
};
};

#endif // _MAP_H_
