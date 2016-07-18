//
// Created by mkr on 24.02.16.
//

#ifndef OPENFABMAP_VISUALPLACERECOGNITION_H
#define OPENFABMAP_VISUALPLACERECOGNITION_H

#include "Defs/opencv.h"
#include <../include/VisualPlaceRecognition/openfabmap.hpp>

class VisualPlaceRecognition
{
public:

    // minimum number of features that have to be found in an image to proceed
    uint32_t minFeatures = 100;
    // how many tail frames to skip
    uint32_t tailFramesToSkip = 10;
    // minimum probability to declare successful loop closure
    // if the probability value for the query frame is below this threshold, it is added as a new, unvisited place
    double_t minNewPlaceProb = 0.2;
    // constructor/initializer
    VisualPlaceRecognition();
    // destructor
    ~VisualPlaceRecognition();
    // find most similar place, return pairs of Ids and probabilities (if prob is above threshold), add new point (forced or based on criteria)
    std::vector<std::pair<int, double>> findAddPlace(cv::Mat frame, int32_t inputID, bool addFrame);

private:

    int32_t nextImageID;

    std::map<int32_t, int32_t> indexMap;

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::BOWImgDescriptorExtractor> bide;
    cv::Ptr<of2::FabMap> fabMap;


};

#endif //OPENFABMAP_VISUALPLACERECOGNITION_H
