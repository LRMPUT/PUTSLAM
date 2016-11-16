/*
 * USAC_utils.h
 *
 *  Created on: Jun 29, 2015
 *      Author: amin
 */

#ifndef USAC_UTILS_H_
#define USAC_UTILS_H_

#include <opencv2/opencv.hpp>
#include <vector>

std::vector<cv::DMatch> getNonRandomMatches(int iterationNumber);

#endif /* USAC_UTILS_H_ */
