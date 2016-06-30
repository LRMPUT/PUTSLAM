#include "Defs/opencv.h"
#include <../include/VisualPlaceRecognition/openfabmap.hpp>
#include <fstream>

#define OPENCV2P4

of2::FabMap *generateFABMAPInstance(cv::FileStorage &settings)
{

    cv::FileStorage fs;

    //load FabMap training data
    std::string fabmapTrainDataPath = settings["FilePaths"]["TrainImagDesc"];
    std::string chowliutreePath = settings["FilePaths"]["ChowLiuTree"];

    fs.open(fabmapTrainDataPath, cv::FileStorage::READ);
    cv::Mat fabmapTrainData;
    fs["BOWImageDescs"] >> fabmapTrainData;
    if (fabmapTrainData.empty()) {
        std::cerr << fabmapTrainDataPath << ": FabMap Training Data not found"
        << std::endl;
        return NULL;
    }
    fs.release();

    //load a chow-liu tree
    fs.open(chowliutreePath, cv::FileStorage::READ);
    cv::Mat clTree;
    fs["ChowLiuTree"] >> clTree;
    if (clTree.empty()) {
        std::cerr << chowliutreePath << ": Chow-Liu tree not found" <<
        std::endl;
        return NULL;
    }
    fs.release();

    //create options flags
    std::string newPlaceMethod =
            settings["openFabMapOptions"]["NewPlaceMethod"];
    std::string bayesMethod = settings["openFabMapOptions"]["BayesMethod"];
    int simpleMotionModel = settings["openFabMapOptions"]["SimpleMotion"];
    int options = 0;
    if(newPlaceMethod == "Sampled") {
        options |= of2::FabMap::SAMPLED;
    } else {
        options |= of2::FabMap::MEAN_FIELD;
    }
    if(bayesMethod == "ChowLiu") {
        options |= of2::FabMap::CHOW_LIU;
    } else {
        options |= of2::FabMap::NAIVE_BAYES;
    }
    if(simpleMotionModel) {
        options |= of2::FabMap::MOTION_MODEL;
    }

    of2::FabMap *fabmap;

    //create an instance of the desired type of FabMap
    std::string fabMapVersion = settings["openFabMapOptions"]["FabMapVersion"];
    if(fabMapVersion == "FABMAP1") {
        fabmap = new of2::FabMap1(clTree,
                                  settings["openFabMapOptions"]["PzGe"],
                                  settings["openFabMapOptions"]["PzGne"],
                                  options,
                                  settings["openFabMapOptions"]["NumSamples"]);
    } else if(fabMapVersion == "FABMAPLUT") {
        fabmap = new of2::FabMapLUT(clTree,
                                    settings["openFabMapOptions"]["PzGe"],
                                    settings["openFabMapOptions"]["PzGne"],
                                    options,
                                    settings["openFabMapOptions"]["NumSamples"],
                                    settings["openFabMapOptions"]["FabMapLUT"]["Precision"]);
    } else if(fabMapVersion == "FABMAPFBO") {
        fabmap = new of2::FabMapFBO(clTree,
                                    settings["openFabMapOptions"]["PzGe"],
                                    settings["openFabMapOptions"]["PzGne"],
                                    options,
                                    settings["openFabMapOptions"]["NumSamples"],
                                    settings["openFabMapOptions"]["FabMapFBO"]["RejectionThreshold"],
                                    settings["openFabMapOptions"]["FabMapFBO"]["PsGd"],
                                    settings["openFabMapOptions"]["FabMapFBO"]["BisectionStart"],
                                    settings["openFabMapOptions"]["FabMapFBO"]["BisectionIts"]);
    } else if(fabMapVersion == "FABMAP2") {
        fabmap = new of2::FabMap2(clTree,
                                  settings["openFabMapOptions"]["PzGe"],
                                  settings["openFabMapOptions"]["PzGne"],
                                  options);
    } else {
        std::cerr << "Could not identify openFABMAPVersion from settings"
                " file" << std::endl;
        return NULL;
    }

    //add the training data for use with the sampling method
    fabmap->addTraining(fabmapTrainData);

    return fabmap;

}




int openFABMAP(std::string testPath,
               of2::FabMap *fabmap,
               std::string vocabPath,
               std::string resultsPath,
               bool addNewOnly)
{

    cv::FileStorage fs;

    //ensure not overwriting results
    std::ifstream checker;
    checker.open(resultsPath.c_str());
    if(checker.is_open()) {
        std::cerr << resultsPath << ": Results already present" << std::endl;
        checker.close();
        return -1;
    }

    //load the vocabulary
    std::cout << "Loading Vocabulary" << std::endl;
    fs.open(vocabPath, cv::FileStorage::READ);
    cv::Mat vocab;
    fs["Vocabulary"] >> vocab;
    if (vocab.empty()) {
        std::cerr << vocabPath << ": Vocabulary not found" << std::endl;
        return -1;
    }
    fs.release();

    //load the test data
    fs.open(testPath, cv::FileStorage::READ);
    cv::Mat testImageDescs;
    fs["BOWImageDescs"] >> testImageDescs;
    if(testImageDescs.empty()) {
        std::cerr << testPath << ": Test data not found" << std::endl;
        return -1;
    }
    fs.release();

    //running openFABMAP
    std::cout << "Running openFABMAP" << std::endl;
    std::vector<of2::IMatch> matches;
    std::vector<of2::IMatch>::iterator l;



    cv::Mat confusion_mat(testImageDescs.rows, testImageDescs.rows, CV_64FC1);
    confusion_mat.setTo(0); // init to 0's


    if (!addNewOnly) {

        //automatically comparing a whole dataset
        fabmap->localize(testImageDescs, matches, true);

        for(l = matches.begin(); l != matches.end(); l++) {
            if(l->imgIdx < 0) {
                confusion_mat.at<double>(l->queryIdx, l->queryIdx) = l->match;

            } else {
                confusion_mat.at<double>(l->queryIdx, l->imgIdx) = l->match;
            }
        }

    } else {

        //criteria for adding locations used
        for(int i = 0; i < testImageDescs.rows; i++) {
            matches.clear();
            //compare images individually
            fabmap->localize(testImageDescs.row(i), matches);

            bool new_place_max = true;
            for(l = matches.begin(); l != matches.end(); l++) {

                if(l->imgIdx < 0) {
                    //add the new place to the confusion matrix 'diagonal'
                    confusion_mat.at<double>(i, (int)matches.size()-1) = l->match;

                } else {
                    //add the score to the confusion matrix
                    confusion_mat.at<double>(i, l->imgIdx) = l->match;
                }

                //test for new location maximum
                if(l->match > matches.front().match) {
                    new_place_max = false;
                }
            }

            if(new_place_max) {
                fabmap->add(testImageDescs.row(i));
            }
        }
    }

    //save the result as plain text for ease of import to Matlab
    std::ofstream writer(resultsPath.c_str());
    for(int i = 0; i < confusion_mat.rows; i++) {
        for(int j = 0; j < confusion_mat.cols; j++) {
            writer << confusion_mat.at<double>(i, j) << " ";
        }
        writer << std::endl;
    }
    writer.close();

    return 0;
}


/*
generates a feature detector based on options in the settings file
*/
cv::Ptr<cv::FeatureDetector> generateDetector(cv::FileStorage &fs) {

    //create common feature detector and descriptor extractor
    std::string detectorMode = fs["FeatureOptions"]["DetectorMode"];
    std::string detectorType = fs["FeatureOptions"]["DetectorType"];
    cv::Ptr<cv::FeatureDetector> detector;
//    if(detectorMode == "ADAPTIVE") {
//
//        if(detectorType != "STAR" && detectorType != "SURF" &&
//           detectorType != "FAST") {
//            std::cerr << "Adaptive Detectors only work with STAR, SURF "
//                    "and FAST" << std::endl;
//        }
//        else {
//
//            detector = new cv::DynamicAdaptedFeatureDetector(
//                    cv::AdjusterAdapter::create(detectorType),
//                    fs["FeatureOptions"]["Adaptive"]["MinFeatures"],
//                    fs["FeatureOptions"]["Adaptive"]["MaxFeatures"],
//                    fs["FeatureOptions"]["Adaptive"]["MaxIters"]);
//        }
//
//    } else if(detectorMode == "STATIC") {
        if(detectorType == "STAR") {

            detector = cv::xfeatures2d::StarDetector::create(
                    fs["FeatureOptions"]["StarDetector"]["MaxSize"],
                    fs["FeatureOptions"]["StarDetector"]["Response"],
                    fs["FeatureOptions"]["StarDetector"]["LineThreshold"],
                    fs["FeatureOptions"]["StarDetector"]["LineBinarized"],
                    fs["FeatureOptions"]["StarDetector"]["Suppression"]);

        } else if(detectorType == "FAST") {

            detector = cv::FastFeatureDetector::create(
                    fs["FeatureOptions"]["FastDetector"]["Threshold"],
                    (int)fs["FeatureOptions"]["FastDetector"]
                    ["NonMaxSuppression"] > 0);

        } else if(detectorType == "SURF") {

#ifdef OPENCV2P4
            detector = cv::xfeatures2d::SURF::create(
                    fs["FeatureOptions"]["SurfDetector"]["HessianThreshold"],
                    fs["FeatureOptions"]["SurfDetector"]["NumOctaves"],
                    fs["FeatureOptions"]["SurfDetector"]["NumOctaveLayers"],
                    (int)fs["FeatureOptions"]["SurfDetector"]["Extended"] > 0,
                    (int)fs["FeatureOptions"]["SurfDetector"]["Upright"] > 0);

#else
            detector = new cv::SurfFeatureDetector(
				fs["FeatureOptions"]["SurfDetector"]["HessianThreshold"],
				fs["FeatureOptions"]["SurfDetector"]["NumOctaves"],
				fs["FeatureOptions"]["SurfDetector"]["NumOctaveLayers"],
				(int)fs["FeatureOptions"]["SurfDetector"]["Upright"] > 0);
#endif
        } else if(detectorType == "SIFT") {
#ifdef OPENCV2P4
            detector = cv::xfeatures2d::SIFT::create(
                    fs["FeatureOptions"]["SiftDetector"]["NumFeatures"],
                    fs["FeatureOptions"]["SiftDetector"]["NumOctaveLayers"],
                    fs["FeatureOptions"]["SiftDetector"]["ContrastThreshold"],
                    fs["FeatureOptions"]["SiftDetector"]["EdgeThreshold"],
                    fs["FeatureOptions"]["SiftDetector"]["Sigma"]);
#else
            detector = new cv::SiftFeatureDetector(
				fs["FeatureOptions"]["SiftDetector"]["ContrastThreshold"],
				fs["FeatureOptions"]["SiftDetector"]["EdgeThreshold"]);
#endif
        } else if(detectorType == "MSER") {

            detector = cv::MSER::create(
                    fs["FeatureOptions"]["MSERDetector"]["Delta"],
                    fs["FeatureOptions"]["MSERDetector"]["MinArea"],
                    fs["FeatureOptions"]["MSERDetector"]["MaxArea"],
                    fs["FeatureOptions"]["MSERDetector"]["MaxVariation"],
                    fs["FeatureOptions"]["MSERDetector"]["MinDiversity"],
                    fs["FeatureOptions"]["MSERDetector"]["MaxEvolution"],
                    fs["FeatureOptions"]["MSERDetector"]["AreaThreshold"],
                    fs["FeatureOptions"]["MSERDetector"]["MinMargin"],
                    fs["FeatureOptions"]["MSERDetector"]["EdgeBlurSize"]);

        } else {
            std::cerr << "Could not create detector class. Specify detector "
                    "options in the settings file" << std::endl;
        }
//    } else {
//        std::cerr << "Could not create detector class. Specify detector "
//                "mode (static/adaptive) in the settings file" << std::endl;
//    }

    return detector;

}

/*
generates a feature detector based on options in the settings file
*/
cv::Ptr<cv::DescriptorExtractor> generateExtractor(cv::FileStorage &fs)
{
    std::string extractorType = fs["FeatureOptions"]["ExtractorType"];
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if(extractorType == "SIFT") {
#ifdef OPENCV2P4
        extractor = cv::xfeatures2d::SIFT::create(
                fs["FeatureOptions"]["SiftDetector"]["NumFeatures"],
                fs["FeatureOptions"]["SiftDetector"]["NumOctaveLayers"],
                fs["FeatureOptions"]["SiftDetector"]["ContrastThreshold"],
                fs["FeatureOptions"]["SiftDetector"]["EdgeThreshold"],
                fs["FeatureOptions"]["SiftDetector"]["Sigma"]);
#else
        extractor = new cv::SiftDescriptorExtractor();
#endif

    } else if(extractorType == "SURF") {

#ifdef OPENCV2P4
        extractor = cv::xfeatures2d::SURF::create(
                fs["FeatureOptions"]["SurfDetector"]["HessianThreshold"],
                fs["FeatureOptions"]["SurfDetector"]["NumOctaves"],
                fs["FeatureOptions"]["SurfDetector"]["NumOctaveLayers"],
                (int)fs["FeatureOptions"]["SurfDetector"]["Extended"] > 0,
                (int)fs["FeatureOptions"]["SurfDetector"]["Upright"] > 0);

#else
        extractor = new cv::SurfDescriptorExtractor(
			fs["FeatureOptions"]["SurfDetector"]["NumOctaves"],
			fs["FeatureOptions"]["SurfDetector"]["NumOctaveLayers"],
			(int)fs["FeatureOptions"]["SurfDetector"]["Extended"] > 0,
			(int)fs["FeatureOptions"]["SurfDetector"]["Upright"] > 0);
#endif

    } else {
        std::cerr << "Could not create Descriptor Extractor. Please specify "
                "extractor type in settings file" << std::endl;
    }

    return extractor;

}
