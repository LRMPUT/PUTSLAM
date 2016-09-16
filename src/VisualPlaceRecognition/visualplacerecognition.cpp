#include "../include/VisualPlaceRecognition/visualplacerecognition.h"
#include "../include/VisualPlaceRecognition/put_ofm.h"

VisualPlaceRecognition::VisualPlaceRecognition(uint32_t _minFeatures, uint32_t _tailFramesToSkip, double_t _minNewPlaceProb)
{
	minFeatures = _minFeatures;
	tailFramesToSkip = _tailFramesToSkip;
	minNewPlaceProb = _minNewPlaceProb;

    std::string settfilename = "../../resources/VisualPlaceRecognition/settings.yml";
    std::string vocabfilename = "../../resources/VisualPlaceRecognition/vocabulary.yml";

    cv::FileStorage fs;
    fs.open(settfilename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Could not open settings file: " << settfilename << std::endl;
        return;
    }

    detector = generateDetector(fs);
    if(!detector) {
        std::cerr << "Feature Detector error" << std::endl;
        return;
    }

    cv::Ptr<cv::DescriptorExtractor> extractor = generateExtractor(fs);
    if(!extractor) {
        std::cerr << "Feature Extractor error" << std::endl;
        return;
    }

    std::string placeAddOption = fs["FabMapPlaceAddition"];
//    bool addNewOnly = (placeAddOption == "NewMaximumOnly");

    fabMap = generateFABMAPInstance(fs);

    std::cout << "Loading Vocabulary" << std::endl;
    fs.open(vocabfilename, cv::FileStorage::READ);
    cv::Mat vocab;
    fs["Vocabulary"] >> vocab;
    if (vocab.empty()) {
        std::cerr << vocabfilename << ": Vocabulary not found" << std::endl;
        return;
    }
    fs.release();

    cv::Ptr<cv::DescriptorMatcher> matcher =
            cv::DescriptorMatcher::create("FlannBased");
    bide = new cv::BOWImgDescriptorExtractor(extractor, matcher);
    bide->setVocabulary(vocab);

    nextImageID = 0;
}

VisualPlaceRecognition::~VisualPlaceRecognition()
{

}

std::vector<std::pair<int, double>> VisualPlaceRecognition::findAddPlace(cv::Mat frame, int32_t inputID, bool addFrame)
{
	// return vector of found similar places
	std::vector<std::pair<int, double>> similarPlaces;

    cv::Mat kptsImg, bow;
    std::vector<cv::KeyPoint> kpts;

    // detect features
    detector->detect(frame, kpts);

    // cv::drawKeypoints(frame, kpts, kptsImg);
    // cv::imshow("Features", kptsImg);

    // bail out if insufficient number of keypoints detected
    if(kpts.size() < minFeatures)
    {
        //std::cout << "insufficient features found" << std::endl;
        return similarPlaces;
    }

    // extract BOW descriptor
    bide->compute(frame, kpts, bow);

    std::vector<of2::IMatch> matches;

    // compare with frame descriptors stored so far, unless it's the first frame
    if ( nextImageID > 0 )
    {
        fabMap->compare(bow, matches, false);
    }

//    double_t bestMatchProb = 0.0;
//    int32_t bestMatchTestIdx = -1;

    // returned value
    int32_t loopID = -1;


    // If we skip tailFramesToSkip frames, we need to rescale the probabilities
    double sumOfProb = 0;
    for(std::vector<of2::IMatch>::iterator it = matches.begin(); it != matches.end(); ++it)
    	if(std::distance(it, matches.end()) > tailFramesToSkip )
    		sumOfProb += it->match;


    for(std::vector<of2::IMatch>::iterator it = matches.begin(); it != matches.end(); ++it)
    {
        if(std::distance(it, matches.end()) <= tailFramesToSkip )
            break;

        if ( it->imgIdx > 0 && it->match/sumOfProb > minNewPlaceProb ) {

        	loopID = indexMap.find(it->imgIdx)->second;

        	similarPlaces.push_back(std::make_pair(loopID, it->match/sumOfProb));
        }

    }

    // if requested, add descriptor of current frame, store indexes, increment frame ID
    if(addFrame)
    {
        indexMap[nextImageID] = inputID;
        fabMap->add(bow);
        ++nextImageID;
    }

    return similarPlaces;
}
