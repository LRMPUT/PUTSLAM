#include <iostream>
#include <thread>
#include "../include/Defs/putslam_defs.h"
#include "Visualizer/Qvisualizer.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <qapplication.h>

using namespace std;

int main(int argc, char** argv)
{
    try {
        using namespace putslam;
        using namespace std::chrono;
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Visualizer" )->FirstChildElement( "parametersFile" )->GetText());

        // Read command lines arguments.
        QApplication application(argc, argv);

        // create putslam visualizer
        Visualizer* visu = createVisualizerQGL(configFile);

        ((QGLVisualizer*) visu)->setWindowTitle("QGLViewer");

        // Make the viewer window visible on screen.
        ((QGLVisualizer*) visu)->show();

        // Run main loop.
        return application.exec();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
