#include <iostream>
#include <thread>
#include "../include/Defs/putslam_defs.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "Map/featuresMap.h"
#include <cmath>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char * argv[])
{
    try {
        using namespace putslam;
        using namespace std::chrono;

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFileGrabber(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
        std::string configFileMap(config.FirstChildElement( "Map" )->FirstChildElement( "parametersFile" )->GetText());

        // create kabsch transform estimator
        Map* map = createFeaturesMap(configFileMap, configFileGrabber);
        map->startOptimizationThread(1);

        for (int i=0;i<10;i++){
            //add some data to the map
            Mat34 cameraPose(Quaternion(1,0,0,0)*Vec3(0.01,0.02,0));
            std::vector<RGBDFeature> features;
            for (int j=0;j<10;j++){
                std::vector<ExtendedDescriptor> descriptors;
                ExtendedDescriptor desc;
                desc.cameraOrientation = Quaternion(1,0,0,0);
                descriptors.push_back(desc);
                RGBDFeature f(Vec3(0.2, 0.3, 2.2), descriptors);
                features.push_back(f);
            }
            map->addFeatures(features, cameraPose);
        }
        usleep(100000);
        map->finishOptimization();//Don't forget to finish optimization thread!!
        Mat34 cameraPose(Quaternion(1,0,0,0)*Vec3(0.01,0.02,0));
        std::vector<MapFeature> visibleFeatures = map->getVisibleFeatures(cameraPose);
        std::cout << visibleFeatures.size() << "\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
