#include "../../include/Solver/utils.h"
#include <sys/time.h>
#include <sstream>
#include <cstdio>
#include <opencv/highgui.h>

namespace PSolver {

  using namespace std;

  double getTime() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6*tv.tv_usec;
  }


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
		     Eigen::Isometry3f& imu_guess,
		     std::istream& is){
    char linebuf[1024];
    is.getline(linebuf, 1024);
    if (! is)
      return false;
    std::istringstream ls(linebuf);
    K.setIdentity();
    std::string image_filename;
    std::string tag;
    ls >> tag;
    if (tag != "DEPTH_IMAGE")
      return false;
    ls >> topic >> seq >> timestamp >> depth_scale;

    for (int i = 0 ; i<2; i++)
      for (int j = 0 ; j<3; j++)
	ls >> K(i,j);
    ls >> image_filename;
    Vector6f offset_v;
    int k=0;
    while(k<6 && ls){
      ls >> offset_v(k);
      k++;
    }

    Vector6f pose_v;
    k=0;
    while(k<6 && ls){
      ls >> pose_v(k);
      k++;
    }
    if (image_filename=="none")
      depth.create(0,0);
    else
      depth  = cv::imread(image_filename.c_str(), CV_LOAD_IMAGE_ANYDEPTH);
      
    if (! ls) {
      has_odom=false;
      return true;
    }

    sensor_offset=v2t(offset_v);
    odom_guess=v2t(pose_v);
    has_odom=true;
    has_imu = false;

    return true;

    Vector6f imu_v;
    k=0;
    while(k<6 && ls){
      ls >> imu_v(k);
      k++;
    }

    if (! ls) {
      has_imu=false;
      return true;
    }
    imu_guess = v2t(imu_v);
    return true;
  }



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
		      const Eigen::Isometry3f& imu_guess,
		      const std::string& file_prefix){

    std::string trimmed_topic = topic;
    std::replace(trimmed_topic.begin(), trimmed_topic.end(), '/', '.');
    if (trimmed_topic.at(0) == '.');
    trimmed_topic=trimmed_topic.substr(1);
    char buf[1024], tsbuf[64];
    buf[0] = 0;
    sprintf(buf, "%s%s-%07d.pgm", file_prefix.c_str(), trimmed_topic.c_str(), seq);
    output_stream << "DEPTH_IMAGE ";
    output_stream << topic << " ";
    output_stream << seq << " ";
    sprintf(tsbuf, "%.05lf", timestamp);
    output_stream << tsbuf << " ";
    output_stream << depth_scale << " ";
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++)
	output_stream << K(i,j) << " ";
    }
    output_stream << buf << " ";
    if (has_odom) {
      Vector6f offset_v=t2v(sensor_offset);
      output_stream << offset_v.transpose() << " ";
      Vector6f pose_v=t2v(odom_guess);
      output_stream << pose_v.transpose() << " ";
    } 
    if (has_imu) {
      Vector6f imu_v=t2v(imu_guess);
      output_stream << imu_v.transpose() << " ";

    } 
    cv::imwrite(buf, depth);
  }

}


