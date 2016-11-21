#include <iostream>

#include "../include/putslam/PUTSLAM/PUTSLAM.h"

int main(int argc, char** argv) { ////////////////////////////////////////////////////////////ROS
	ros::init(argc, argv, "demoROS"); ///////////////////////////////////////////////////////////////////////ROS
	std::unique_ptr<PUTSLAM> putslam;
	putslam.reset(new PUTSLAM);
	putslam->setWorkWithROS();
	putslam->initROSpublishers();
	putslam.get()->startProcessing();


	return 0;
}
