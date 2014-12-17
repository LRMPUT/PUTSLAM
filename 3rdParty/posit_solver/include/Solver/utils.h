#pragma once
#include "defs.h"

namespace PSolver {
  
  //! returns the system time in seconds
  double getTime();

  bool  readDumpLine(bool& has_odom,
		     bool& has_imu,
		     RawDepthImage& depth, 
		     Eigen::Matrix3f& K, 
		     float& depth_scale, 
		     int& seq,
		     double& timestamp,
		     std::string& topic,
		     Eigen::Isometry3f& sensor_offset,
		     Eigen::Isometry3f& odom_guess,
		     Eigen::Isometry3f& imu,
		     std::istream& is);

  void  writeDumpLine(bool has_odom,
		      bool has_imu,
		      std::ostream& output_stream,
		      const RawDepthImage& depth, 
		      const Eigen::Matrix3f& K, 
		      float depth_scale, 
		      int seq,
		      double timestamp,
		      const std::string& topic,
		      const Eigen::Isometry3f& sensor_offset,
		      const Eigen::Isometry3f& odom_guess,
		      const Eigen::Isometry3f& imu,
		      const std::string& file_prefix="");

}

