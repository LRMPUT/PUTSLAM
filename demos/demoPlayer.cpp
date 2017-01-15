#include "Defs/putslam_defs.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#ifdef BUILD_PUTSLAM_VISUALIZER
    #include "Visualizer/Qvisualizer.h"
    #include <GL/glut.h>
    #include <qapplication.h>
#endif
#include "PUTSLAM/PUTSLAM.h"
#include <iostream>
#include "Defs/opencv.h"
#include "Utilities/CLParser.h"

using namespace std;

std::unique_ptr<PUTSLAM> slam;

// run PUTSLAM
void runPUTSLAM(std::string trajectoryFilename, int delayPlay){
    std::cout << "Press Enter to start\n";
    getchar();
    slam.get()->startPlaying(trajectoryFilename, delayPlay);
}

int main(int argc, char** argv)
{
    try {
        CLParser cmd_line(argc,argv,true);
        if (cmd_line.get_arg("-h").size())
            std::cout << "To run type: ./demoPlayer -i pathtotrajectoryfile.res -d 1500000\n";
        std::string trajname;
        int delay(1000);
        if (cmd_line.get_arg("-i").length()!=0){
            trajname = cmd_line.get_arg("-i");
        }
        else {
            std::cout << "No input file specified (-i fileneme)\n";
            return 0;
        }
        if (cmd_line.get_arg("-d").length()!=0){
            delay = std::stoi(cmd_line.get_arg("-d"));
        }
        else {
            std::cout << "Delay not specified (-d value)\n";
            return 0;
        }

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/putslamconfigGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string configFile(config.FirstChildElement( "Visualizer" )->FirstChildElement( "parametersFile" )->GetText());

        #ifdef BUILD_PUTSLAM_VISUALIZER
        QGLVisualizer::Config configVis(configFile);//something is wrong with QApplication when Qapplication
        //object is created. libTinyxml can read only ints from xml file
        #endif

        slam.reset(new PUTSLAM);
        #ifdef BUILD_PUTSLAM_VISUALIZER
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
        #endif
        // run PUTSLAM
        std::thread tSLAM(runPUTSLAM, trajname, delay);

        #ifdef BUILD_PUTSLAM_VISUALIZER
        // Run main loop.
        application.exec();
        #endif
        tSLAM.join();

        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
