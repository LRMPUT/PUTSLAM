#include "../include/USAC/USAC_utils.h"

std::vector<cv::DMatch> prepareNonRandomMatches(void)
{
	std::vector<cv::DMatch> nonRandomMatches;
	nonRandomMatches.push_back(cv::DMatch(104, 131, 0.0));
	nonRandomMatches.push_back(cv::DMatch(187, 196, 0.0));
	nonRandomMatches.push_back(cv::DMatch(33, 28, 0.0));
	nonRandomMatches.push_back(cv::DMatch(136, 143, 0.0));
	nonRandomMatches.push_back(cv::DMatch(111, 209, 0.0));
	nonRandomMatches.push_back(cv::DMatch(181, 191, 0.0));

	nonRandomMatches.push_back(cv::DMatch(186, 189, 0.0));
	nonRandomMatches.push_back(cv::DMatch(96, 95, 0.0));
	nonRandomMatches.push_back(cv::DMatch(125, 140, 0.0));
	nonRandomMatches.push_back(cv::DMatch(185, 201, 0.0));
	nonRandomMatches.push_back(cv::DMatch(75, 72, 0.0));
	nonRandomMatches.push_back(cv::DMatch(181, 191, 0.0));

	nonRandomMatches.push_back(cv::DMatch(86, 98, 0.0));
	nonRandomMatches.push_back(cv::DMatch(213, 217, 0.0));
	nonRandomMatches.push_back(cv::DMatch(232, 231, 0.0));
	nonRandomMatches.push_back(cv::DMatch(188, 186, 0.0));
	nonRandomMatches.push_back(cv::DMatch(46, 51, 0.0));
	nonRandomMatches.push_back(cv::DMatch(59, 66, 0.0));

	nonRandomMatches.push_back(cv::DMatch(181, 191, 0.0));
	nonRandomMatches.push_back(cv::DMatch(35, 39, 0.0));
	nonRandomMatches.push_back(cv::DMatch(32, 31, 0.0));
	nonRandomMatches.push_back(cv::DMatch(129, 119, 0.0));
	nonRandomMatches.push_back(cv::DMatch(138, 132, 0.0));
	nonRandomMatches.push_back(cv::DMatch(184, 151, 0.0));

	nonRandomMatches.push_back(cv::DMatch(222, 238, 0.0));
	nonRandomMatches.push_back(cv::DMatch(72, 75, 0.0));
	nonRandomMatches.push_back(cv::DMatch(238, 94, 0.0));
	nonRandomMatches.push_back(cv::DMatch(178, 225, 0.0));
	nonRandomMatches.push_back(cv::DMatch(10, 14, 0.0));
	nonRandomMatches.push_back(cv::DMatch(33, 28, 0.0));

	nonRandomMatches.push_back(cv::DMatch(83, 91, 0.0));
	nonRandomMatches.push_back(cv::DMatch(77, 78, 0.0));
	nonRandomMatches.push_back(cv::DMatch(113, 114, 0.0));
	nonRandomMatches.push_back(cv::DMatch(154, 148, 0.0));
	nonRandomMatches.push_back(cv::DMatch(63, 57, 0.0));
	nonRandomMatches.push_back(cv::DMatch(5, 5, 0.0));

	nonRandomMatches.push_back(cv::DMatch(206, 203, 0.0));
	nonRandomMatches.push_back(cv::DMatch(113, 114, 0.0));
	nonRandomMatches.push_back(cv::DMatch(215, 211, 0.0));
	nonRandomMatches.push_back(cv::DMatch(152, 156, 0.0));
	nonRandomMatches.push_back(cv::DMatch(235, 200, 0.0));
	nonRandomMatches.push_back(cv::DMatch(225, 235, 0.0));

	nonRandomMatches.push_back(cv::DMatch(3, 1, 0.0));
	nonRandomMatches.push_back(cv::DMatch(124, 117, 0.0));
	nonRandomMatches.push_back(cv::DMatch(111, 209, 0.0));
	nonRandomMatches.push_back(cv::DMatch(96, 95, 0.0));
	nonRandomMatches.push_back(cv::DMatch(10, 14, 0.0));
	nonRandomMatches.push_back(cv::DMatch(201, 193, 0.0));

	nonRandomMatches.push_back(cv::DMatch(185, 201, 0.0));
	nonRandomMatches.push_back(cv::DMatch(141, 145, 0.0));
	nonRandomMatches.push_back(cv::DMatch(77, 78, 0.0));
	nonRandomMatches.push_back(cv::DMatch(28, 33, 0.0));
	nonRandomMatches.push_back(cv::DMatch(184, 151, 0.0));
	nonRandomMatches.push_back(cv::DMatch(23, 25, 0.0));


	return nonRandomMatches;
}

std::vector<cv::DMatch> getNonRandomMatches(int iterationNumber)
{
	static std::vector<cv::DMatch> nonRandomMatches;
	if (iterationNumber == 0)
	{
		nonRandomMatches.clear();
		nonRandomMatches = prepareNonRandomMatches();
	}

	std::vector<cv::DMatch> currentMatches;
	currentMatches.push_back(nonRandomMatches[3*iterationNumber]);
	currentMatches.push_back(nonRandomMatches[3*iterationNumber+1]);
	currentMatches.push_back(nonRandomMatches[3*iterationNumber+2]);

	return currentMatches;
}
