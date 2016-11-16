#include <iostream>

#include "../include/putslam/PUTSLAM/PUTSLAM.h"

int main() {

	std::unique_ptr<PUTSLAM> putslam;
	putslam.reset(new PUTSLAM);
	putslam.get()->startProcessing();


	return 0;
}
