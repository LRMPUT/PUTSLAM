#ifdef BUILD_WITH_ROS
#ifndef ROS_GRABBER_H
#define ROS_GRABBER_H

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "grabber.h"
#include "depthSensorModel.h"
#include <stddef.h>

//using namespace sensor_msgs;
using namespace message_filters;

using namespace putslam;

namespace putslam {
	/// create a single grabber (Kinect)
	Grabber* createGrabberROS(void);
	Grabber* createGrabberROS(ros::NodeHandle nh);
};

#define MAX_DEPTH 10000

class ROSGrabber : public Grabber {

	public:
	
	/// Pointer
  typedef std::unique_ptr<ROSGrabber> Ptr;

	//constructor	
	ROSGrabber(void);
	ROSGrabber(ros::NodeHandle nh);
	//destructor
	~ROSGrabber(void);

	/// Name of the grabber
  virtual const std::string& getName() const;

  /// Returns current point cloud
  virtual const PointCloud& getCloud(void) const;

  /// Grab image and/or point cloud
  virtual bool grab();

  /// Calibrate sensor
  virtual void calibrate(void);
	
  /// Closing a device
  int grabberClose();
	
  Eigen::Matrix4f getStartingSensorPose();
	
	/// Returns current frame
  const SensorFrame& getSensorFrame(void);

	//local functions

	void callback(const sensor_msgs::ImageConstPtr& imageRGB, const sensor_msgs::ImageConstPtr& imageDepth);

	//variables

	ros::NodeHandle nh;
  	message_filters::Subscriber<sensor_msgs::Image> imageRGB_sub;
  	message_filters::Subscriber<sensor_msgs::Image> imageDepth_sub;
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;
	
	int imageDepthScale;
	int iterate;
	int maxProcessFrames;
	int lastReadId;

};


#endif // ROS_GRABBER_H
#endif //BUILD_WITH_ROS
