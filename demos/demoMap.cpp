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
        std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());

        // create kabsch transform estimator
        Map* map = createFeaturesMap();
        map->startOptimizationThread(3);
        usleep(1000);
        map->finishOptimization();//Don't forget to finish optimization thread!!
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
