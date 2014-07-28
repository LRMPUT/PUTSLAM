#include <iostream>
#include <thread>
#include "include/Defs/putslam_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Grabber/xtion_grabber.h"
#include "Grabber/ptgrey_grabber.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <cmath>
#include <ctime>
#include <ratio>
#include <chrono>

using namespace std;

unsigned const max_tracking_duration = 6;//seconds

int main()
{
    try {
        using namespace putslam;

        cv::Mat m, depth,color;
        cv::namedWindow( "Depth View", 1 );
        cv::namedWindow( "RGB View", 1 );

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";
        std::string grabberType(config.FirstChildElement( "Grabber" )->FirstChildElement( "name" )->GetText());

        Grabber* grabber;
        if (grabberType == "Kinect") {
            std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
            grabber = createGrabberKinect(configFile);
        }
        if (grabberType == "Xtion") {
            std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
            grabber = createGrabberXtion(configFile);
        }
        if (grabberType == "Ptgrey") {
            std::string configFile(config.FirstChildElement( "Grabber" )->FirstChildElement( "calibrationFile" )->GetText());
            grabber = createGrabberPtgrey(configFile);
        }
        else if (grabberType == "MesaImaging")
            grabber = createGrabberKinect();
        else // Default
            grabber = createGrabberKinect();

//        Mat33 cov;
//        ((XtionGrabber*)grabber)->model.computeCov(80, 360, 0.5837, cov);
//        Eigen::Vector3d vec;
//        ((XtionGrabber*)grabber)->model.getPoint(377.177, 112.906, 6.468, vec);

        // create objects and print configuration
        cout << "Current grabber: " << grabber->getName() << std::endl;

        auto start = chrono::system_clock::now();
        while (cv::waitKey(30) != 27){ //tracking
            SensorFrame sf;
            try{
            grabber->grab(); // grab frame
            }
            catch (int e)
              {
                cout << "An exception occurred. Exception Nr. " << e << '\n';
                break;
              }
            sf = grabber->getSensorFrame();
            //sf.depth.convertTo(depth, CV_8UC1, 255.0/1024.0); //conversion to 8-bit format
            //cv::imshow("Depth View",depth);
            cv::imshow( "RGB View", sf.image );

//            if (chrono::duration_cast<chrono::duration<unsigned> >(chrono::system_clock::now() - start).count()>max_tracking_duration){

//                break;
//            }
        }
        grabber->grabberClose();

    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
