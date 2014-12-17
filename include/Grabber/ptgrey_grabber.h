/** @file ptgrey_grabber.h
 *
 * implementation - Ptgrey Grabber
 *
 */

#ifndef PTGREY_GRABBER_H_INCLUDED
#define PTGREY_GRABBER_H_INCLUDED

#include "grabber.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>
#ifdef WITH_PTGREY
    #include "FlyCapture2.h"
#endif
#include <chrono>
#include <thread>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#ifdef WITH_PTGREY
    using namespace FlyCapture2;
#endif


namespace putslam {
    /// create a single grabber (Ptgrey)
    Grabber* createGrabberPtgrey(void);
    Grabber* createGrabberPtgrey(std::string configFile);
};

using namespace putslam;

/// Grabber implementation
class PtgreyGrabber : public Grabber {
    public:

    class UncertaintyModel {
      public:
        /// Construction
        UncertaintyModel(){
        }

        /// Construction
        UncertaintyModel(std::string configFile) : config(configFile){
            PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],
                          0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1],
                          0,0,1;
            Ruvd << config.varU, 0, 0,
                    0, config.varV, 0,
                    0, 0, 0;
        }

        void getPoint(uint_fast16_t u, uint_fast16_t v, float_type depth, Eigen::Vector3d& point3D){
            Eigen::Vector3d point(u, v, 1);
            point3D = depth*PHCPModel*point;
        }

        void computeCov(uint_fast16_t u, uint_fast16_t v, float_type depth, Mat33& cov){
            //float_type dispDer = config.k3 * 1/(config.k2*pow(cos((disparity/config.k2) + config.k1),2.0));
            Mat33 J;
            J << 0.0017*depth, 0, (0.0017*u-0.549),
                 0, 0.0017*depth, (0.0017*v-0.443),
                 0, 0, 1;
            Ruvd(2,2) = (config.distVarCoefs[0]*pow(depth,3.0) + config.distVarCoefs[1]*pow(depth,2.0) + config.distVarCoefs[2]*depth + config.distVarCoefs[3])/3.0;
            cov=J*Ruvd*J.transpose();
        }

        class Config{
          public:
            Config() :
                focalLength{582.64, 586.97},
                focalAxis{320.17, 260.0},
                varU(1.1046), varV(0.64160),
                distVarCoefs{-8.9997e-06, 3.069e-003, 3.6512e-006, -0.0017512e-3}{
            }
            Config(std::string configFilename){
                tinyxml2::XMLDocument config;
                std::string filename = "../../resources/" + configFilename;
                config.LoadFile(filename.c_str());
                if (config.ErrorID())
                    std::cout << "unable to load Ptgrey config file.\n";
                tinyxml2::XMLElement * model = config.FirstChildElement( "Model" );
                model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fx", &focalLength[0]);
                model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fy", &focalLength[1]);
                model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cx", &focalAxis[0]);
                model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cy", &focalAxis[1]);
                model->FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaU", &varU);
                model->FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaV", &varV);
                model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c3", &distVarCoefs[0]);
                model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c2", &distVarCoefs[1]);
                model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c1", &distVarCoefs[2]);
                model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c0", &distVarCoefs[3]);
                tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
                float_type query[4];
                posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
                Quaternion q(query[0], query[1], query[2], query[3]);
                posXML->QueryDoubleAttribute("x", &query[0]); posXML->QueryDoubleAttribute("y", &query[1]); posXML->QueryDoubleAttribute("z", &query[2]);
                Vec3 pos(query[0], query[1], query[2]);
                pose = q*pos;
            }
            public:
                float_type focalLength[2];
                float_type focalAxis[2];
                float_type varU, varV;// variance u,v
                float_type distVarCoefs[4];
                Mat34 pose; // Ptgrey pose in robot's coordination frame
        };

        Config config;

        private:
            Mat33 PHCPModel;//pin-hole camera projection model
            Mat33 Ruvd; //covariance matrix for [u,v,disp]
    };

        /// Pointer
        typedef std::unique_ptr<PtgreyGrabber> Ptr;

        /// Construction
        PtgreyGrabber(void);

        /// Construction
        PtgreyGrabber(std::string modelFilename);

        /// Name of the grabber
        virtual const std::string& getName() const;

        /// Returns current point cloud
        virtual const PointCloud& getCloud(void) const;

        /// Returns the current 2D image
        virtual const SensorFrame& getSensorFrame(void);

        /// Grab image and/or point cloud
        virtual bool grab();
    #ifdef WITH_PTGREY
        ///Sensor initialization
        virtual int initPtGrey ();
    #endif
        /// Calibrate sensor
        virtual void calibrate(void);

        ///Sensor uninitialize
        virtual int grabberClose(void);

        UncertaintyModel model;
    protected:
    #ifdef WITH_PTGREY
        Error error;
        BusManager busMgr;
        unsigned int numCameras;
        PGRGuid guid;
        Camera cam;

    private:
        ///Prints out error trace
        void PrintError( Error error );
        ///Prints out camera info
        void PrintCameraInfo( CameraInfo* pCamInfo );
    #endif

};

#endif // PTGREY_GRABBER_H_INCLUDED
