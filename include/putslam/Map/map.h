/** @file map.h
 *
 * Environment Map interface
 * \author Dominik Belter
 */

#ifndef _MAP_H_
#define _MAP_H_

#include "../Defs/putslam_defs.h"
#include "Grabber/depthSensorModel.h"
#include "Matcher/matcherOpenCV.h"
#include "LoopClosure/loopClosure.h"
#include <string>
#include <vector>

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
			type(_type), name(_name) {
	}


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
            const Mat34& cameraPose, int graphDepthThreshold, double distanceThreshold) = 0;

    /// get all covisible features using covisibility graph
    virtual std::vector<MapFeature> getCovisibleFeatures(void) = 0;

    /// find nearest id of the image frame taking into acount the current angle of view and the view from the history
    virtual void findNearestFrame(const std::vector<MapFeature>& features, std::vector<int>& imageIds, std::vector<double>& angles, double maxAngle = 3.14) = 0;

    /// removes features which are too far from current camera pose (distant in graph)
    virtual void removeDistantFeatures(std::vector<MapFeature>& mapFeatures, int graphDepthThreshold = 0, double distanceThreshold = 0) = 0;

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
    virtual int addNewPose(const Mat34& cameraPoseChange, double timestamp, cv::Mat image = cv::Mat(), cv::Mat depthImage = cv::Mat()) = 0;

	/// Get all features
	virtual std::vector<MapFeature> getAllFeatures(void) = 0;

	/// Get feature position
    virtual Vec3 getFeaturePosition(unsigned int id) const = 0;

	/// get pose of the sensor (default: last pose)
    virtual Mat34 getSensorPose(int poseId = -1) const = 0;

	// get number of poses stored in map
	virtual int getPoseCounter() = 0;

	/// getDepthSensorModel
	virtual DepthSensorModel getDepthSensorModel() = 0;

	/// start optimization thread
    virtual void startOptimizationThread(unsigned int iterNo, int verbose = 0, std::string RobustKernelName = "", double kernelDelta = 0) = 0;

    /// start map management thread
    virtual void startMapManagerThread(int verbose = 0) = 0;

    /// start loop closure thread
    virtual void startLoopClosureThread(int verbose, Matcher* matcher) = 0;

	/// Wait for optimization thread to finish
	virtual void finishOptimization(std::string trajectoryFilename,
			std::string graphFilename) = 0;

    /// Export graph and trajectory
    virtual void exportOutput(std::string trajectoryFilename,
            std::string graphFilename) = 0;

    /// Wait for map management thread to finish
    virtual void finishManagementThr(void) = 0;

    /// Wait for loop closure thread to finish
    virtual void finishLoopClosureThr(void) = 0;

    /// Save map to file
    virtual void save2file(std::string mapFilename, std::string graphFilename) = 0;

    /// set Robust Kernel
    virtual void setRobustKernel(std::string name, double delta) = 0;

    /// disable Robust Kernel
    virtual void disableRobustKernel(void) = 0;

    /// get n-th image and depth image from the sequence
    virtual void getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage) = 0;

    /// get uncertainty of the pose
    //virtual Mat66 getPoseUncertainty(unsigned int id) const = 0;

    /// get uncertainty of the feature
    //virtual Mat33 getFeatureUncertainty(unsigned int id) const = 0;

    /// set drawing options
    virtual void setDrawOptions(bool _draw) = 0;

    /// use uncertainty
    virtual bool useUncertainty(void) = 0;

    /// store camera frames
    virtual void setStoreImages(bool storeImages) = 0;

    /// return the size of the map
    virtual int getNumberOfFeatures() = 0;

    virtual bool getAndResetLoopClosureSuccesful() = 0;

    /// get loopClosureMatchingRatiosLog
    virtual std::vector<double> getLoopClosureMatchingRatiosLog() = 0;

    virtual std::vector<LoopClosure::LCMatch> getLoopClosureAnalyzedPairsLog() = 0;

protected:
	/// Map type
	Type type;

	/// Map name
	const std::string name;
};
};

#endif // _MAP_H_
