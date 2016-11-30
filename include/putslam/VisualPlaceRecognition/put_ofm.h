#include "Defs/opencv.h"
//#include <opencv2/opencv.hpp>
#include <VisualPlaceRecognition/openfabmap.hpp>


of2::FabMap *generateFABMAPInstance(cv::FileStorage &settings);
int openFABMAP(std::string testPath, of2::FabMap *fabmap, std::string vocabPath, std::string resultsPath, bool addNewOnly);

cv::Ptr<cv::DescriptorExtractor> generateExtractor(cv::FileStorage &fs);
cv::Ptr<cv::FeatureDetector> generateDetector(cv::FileStorage &fs);
