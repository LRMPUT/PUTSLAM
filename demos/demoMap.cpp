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
        map->startOptimizationThread(1,1);

        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
        for (int i=0;i<2;i++){
            //add some data to the map
            Mat34 cameraPose(Quaternion(1,0,0,0)*Vec3(0.01,0.02,0));
            std::vector<RGBDFeature> features;
            for (int j=0;j<10;j++){
                RGBDFeature f(Vec3(0.2, 0.3, 2.2), 0, 0, std::vector<ExtendedDescriptor>());
                features.push_back(f);
            }
            unsigned int id  = map->addNewPose(cameraPose, (std::chrono::high_resolution_clock::now() - startTime).count());
            map->addFeatures(features, id);
            std::vector<MapFeature> mapFeatures = map->getAllFeatures();
            std::vector<MapFeature> measurements;
            for (std::vector<MapFeature>::iterator it = mapFeatures.begin(); it!=mapFeatures.end(); it++){
                MapFeature feat;
                feat.id = it->id;
                Mat34 poseGlob (it->position);
                Mat34 meas = cameraPose.inverse()*poseGlob;
                feat.position.x() = meas(0,3)-0.01; feat.position.y() = meas(1,3)+0.01; feat.position.z() = meas(2,3);
                if (feat.id<10000+i*10)
                    measurements.push_back(feat);
            }
            map->addMeasurements(measurements);
        }
        usleep(100000);
        map->finishOptimization("../../resources/trajectory.res", "../../resources/graph.g2o");//Don't forget to finish optimization thread!!
        Mat34 cameraPose(Quaternion(1,0,0,0)*Vec3(0.01,0.02,0));
        std::vector<MapFeature> visibleFeatures = map->getVisibleFeatures(cameraPose);
        std::cout << visibleFeatures.size() << "\n";
        std::cout << "cameraPose:\n" << map->getSensorPose().matrix() << "\n";
        map->save2file("../../resources/map.map", "../../resources/mapGraph.g2o");
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
