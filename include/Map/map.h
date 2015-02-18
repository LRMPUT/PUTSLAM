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

	/// Add NEW features and a NEW camera pose (initial guess) to the map
	/// Position of features in relation to camera pose
	virtual void addFeatures(const std::vector<RGBDFeature>& features,
			int poseId = -1) = 0;

	/// add measurements (features measured from the last camera pose)
	virtual void addMeasurements(const std::vector<MapFeature>& features,
			int poseId = -1) = 0;

	/// add new pose of the camera, returns id of the new pose
	virtual int addNewPose(const Mat34& cameraPose, float_type timestamp) = 0;

	/// Get all features
	virtual std::vector<MapFeature> getAllFeatures(void) = 0;

	/// Get feature position
	virtual Vec3 getFeaturePosition(unsigned int id) = 0;

	/// get pose of the sensor (default: last pose)
	virtual Mat34 getSensorPose(int poseId = -1) = 0;

	/// start optimization thread
	virtual void startOptimizationThread(unsigned int iterNo, int verbose = 0) = 0;

	/// Wait for optimization thread to finish
	virtual void finishOptimization(std::string trajectoryFilename,
			std::string graphFilename) = 0;

    /// Save map to file
    virtual void save2file(std::string mapFilename, std::string graphFilename) = 0;

protected:
	/// Map type
	Type type;

	/// Map name
	const std::string name;
};
}
;

#endif // _MAP_H_
