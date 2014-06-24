/** @file kinect_grabber.h
 *
 * implementation - Xtion Grabber
 *
 */
#ifndef XTION_GRABBER_H
#define XTION_GRABBER_H

#include <stddef.h>
#include <OpenNI.h>
#include "grabber.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>

#define MAX_DEPTH 10000

enum DisplayModes
{
    DISPLAY_MODE_OVERLAY,
    DISPLAY_MODE_DEPTH,
    DISPLAY_MODE_IMAGE
};

namespace putslam {
    /// create a single grabber (Xtion)
    Grabber* createGrabberXtion(void);
    Grabber* createGrabberXtion(std::string configFile);
};

using namespace putslam;

class XtionGrabber : public Grabber {

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
                    std::cout << "unable to load Kinect config file.\n";
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
                Mat34 pose; // kinect pose in robot's coordination frame
        };

        Config config;

        private:
            Mat33 PHCPModel;//pin-hole camera projection model
            Mat33 Ruvd; //covariance matrix for [u,v,disp]
    };


    /// Pointer
    typedef std::unique_ptr<XtionGrabber> Ptr;

    /// Construction
    XtionGrabber(void);

    /// Construction
    XtionGrabber(std::string modelFilename);

    /// Name of the grabber
    virtual const std::string& getName() const;

    /// Returns current point cloud
    virtual const PointCloud& getCloud(void) const;

    /// Returns the current 2D image
    virtual const SensorFrame& getSensorFrame(void) const;

    /// Grab image and/or point cloud
    virtual void grab();

    /// Calibrate sensor
    virtual void calibrate(void);

    virtual int initOpenNI ();

    UncertaintyModel model;

protected:
    openni::Status rc;
    openni::Device device;
    openni::VideoStream depth;
    openni::VideoStream color;
    openni::VideoFrameRef		m_depthFrame;
    openni::VideoFrameRef		m_colorFrame;
    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;
    const openni::SensorInfo *depthSensorInfo;
    const openni::SensorInfo *colorSensorInfo;


private:

};


#endif // XTION_GRABBER_H
