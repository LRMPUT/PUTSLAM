#include "../include/Defs/putslam_defs.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "../include/Visualizer/Qvisualizer.h"
#include "../include/PUTSLAM/PUTSLAM.h"
#include <GL/glut.h>
#include <qapplication.h>
#include <iostream>
#include "Defs/opencv.h"

using namespace std;

std::unique_ptr<PUTSLAM> slam;

// run PUTSLAM
void runPUTSLAM(){
    std::cout << "Press Enter to start\n";
    getchar();
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

        QGLVisualizer::Config configVis(configFile);//something is wrong with QApplication when Qapplication
        //object is created. libTinyxml can read only ints from xml file

        slam.reset(new PUTSLAM);
        if (configVis.showFrames){
            cv::Mat im(480,680, CV_8UC3);
            cv::namedWindow( "PUTSLAM RGB frame", cv::WINDOW_AUTOSIZE );
            cv::imshow( "PUTSLAM RGB frame", im );
            cv::namedWindow( "PUTSLAM Depth frame", cv::WINDOW_AUTOSIZE );
            cv::imshow( "PUTSLAM Depth frame", im );
            slam->setDrawOptions(true, true);
        }
        else{
            slam->setDrawOptions(true, false);
        }

        QApplication application(argc,argv);

        glutInit(&argc, argv);

        QGLVisualizer visu(configVis);
        visu.setDepthSensorModel(slam.get()->getDepthSensorModel());

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
