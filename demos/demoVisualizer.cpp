#include <iostream>
#include "../include/Defs/putslam_defs.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "../include/Visualizer/Qvisualizer.h"
#include "../include/PUTSLAM/PUTSLAM.h"
#include <qapplication.h>

using namespace std;

std::unique_ptr<PUTSLAM> slam;

// run PUTSLAM
void runPUTSLAM(){
    slam.get()->startProcessing();
}

int main(int argc, char** argv)
{
    try {
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Visualizer" )->FirstChildElement( "parametersFile" )->GetText());

        slam.reset(new PUTSLAM);

        QApplication application(argc,argv);

        QGLVisualizer visu(configFile);

        visu.setWindowTitle("PUT SLAM map viewer");

        // Make the viewer window visible on screen.
        visu.show();
        slam.get()->attachVisualizer(&visu);

        // run PUTSLAM
        std::thread tSLAM(runPUTSLAM);

        // Run main loop.
        application.exec();
        tSLAM.join();

        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
