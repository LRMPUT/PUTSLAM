/** @file QVisualizer.h
 *
 * implementation - QGLVisualizer
 *
 */

#ifndef QVISUALIZER_H_INCLUDED
#define QVISUALIZER_H_INCLUDED

#include "../Defs/putslam_defs.h"
#include "../include/Utilities/observer.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <QGLViewer/qglviewer.h>
#include <iostream>

using namespace putslam;

/// Map implementation
class QGLVisualizer: public QGLViewer, public Observer{
public:
    /// Pointer
    typedef std::unique_ptr<QGLVisualizer> Ptr;

    class Config{
      public:
        Config() {
        }
        Config(std::string configFilename){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load Visualizer config file.\n";
            tinyxml2::XMLElement * model = config.FirstChildElement( "VisualizerConfig" );
            double rgba[4]={0,0,0,0};
            model->FirstChildElement( "background" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "background" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "background" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "background" )->QueryDoubleAttribute("alpha", &rgba[3]);
            backgroundColor.setRedF(rgba[0]); backgroundColor.setGreenF(rgba[1]);
            backgroundColor.setBlueF(rgba[2]); backgroundColor.setAlphaF(rgba[3]);
            model->FirstChildElement( "trajectory" )->QueryBoolAttribute("drawTrajectory", &drawTrajectory);
            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("width", &trajectoryWidth);
            trajectoryColor.setRedF(rgba[0]); trajectoryColor.setGreenF(rgba[1]);
            trajectoryColor.setBlueF(rgba[2]); trajectoryColor.setAlphaF(rgba[3]);
            model->FirstChildElement( "trajectoryPoints" )->QueryBoolAttribute("drawTrajectoryPoints", &drawTrajectoryPoints);
            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("size", &trajectoryPointsSize);
            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "trajectoryPoints" )->QueryIntAttribute("smoothness", &trajectoryPointSmoothness);
            trajectoryPointsColor.setRedF(rgba[0]); trajectoryPointsColor.setGreenF(rgba[1]);
            trajectoryPointsColor.setBlueF(rgba[2]); trajectoryPointsColor.setAlphaF(rgba[3]);
            model->FirstChildElement( "features" )->QueryBoolAttribute("drawFeatures", &drawFeatures);
            model->FirstChildElement( "features" )->QueryDoubleAttribute("size", &featuresSize);
            model->FirstChildElement( "features" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "features" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "features" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "features" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "features" )->QueryIntAttribute("smoothness", &featuresSmoothness);
            featuresColor.setRedF(rgba[0]); featuresColor.setGreenF(rgba[1]);
            featuresColor.setBlueF(rgba[2]); featuresColor.setAlphaF(rgba[3]);
            model->FirstChildElement( "pose2feature" )->QueryBoolAttribute("drawLinks", &drawPose2Feature);
            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("width", &pose2FeatureWidth);
            pose2FeatureColor.setRedF(rgba[0]); pose2FeatureColor.setGreenF(rgba[1]);
            pose2FeatureColor.setBlueF(rgba[2]); pose2FeatureColor.setAlphaF(rgba[3]);
        }
        public:
        /// Background color
        QColor backgroundColor;

        /// Draw trajectory
        bool drawTrajectory;

        /// Trajectory color
        QColor trajectoryColor;

        /// Trajectory line width
        double trajectoryWidth;

        /// Draw trajectory points
        bool drawTrajectoryPoints;

        /// Trajectory points color
        QColor trajectoryPointsColor;

        /// Trajectory points size
        double trajectoryPointsSize;

        /// Trajectory point smoothness
        int trajectoryPointSmoothness;

        /// Draw features
        bool drawFeatures;

        /// Features color
        QColor featuresColor;

        /// Features size
        double featuresSize;

        /// Features smoothness
        int featuresSmoothness;

        /// Draw pose 2 feature link
        bool drawPose2Feature;

        /// link color color
        QColor pose2FeatureColor;

        /// pose2feature link width
        double pose2FeatureWidth;
    };

    /// Construction
    QGLVisualizer(void);

    /// Construction
    QGLVisualizer(std::string configFile);

    /// Construction
    QGLVisualizer(Config _config);

    /// Destruction
    ~QGLVisualizer(void);

    /// Observer update
    void update(MapModifier& mapModifier);

private:
    Config config;

    ///Set of features (map for the front-end thread)
    std::map<int,MapFeature> featuresMap;

    /// mutex for critical section - map frontend
    std::recursive_mutex mtxFeaturesMap;

    ///camera trajectory
    std::vector<VertexSE3> camTrajectory;

    /// mutex for critical section - cam trajectory
    std::recursive_mutex mtxCamTrajectory;

    /// Map visualization -- buffer
    MapModifier bufferMapVisualization;

    /// draw objects
    void draw();

    /// draw objects
    void animate();

    /// initialize visualizer
    void init();

    /// generate help string
    std::string help() const;

    ///update map
    void updateMap();

    /// Update feature
    void updateFeature(std::map<int,MapFeature>& featuresMap,
            MapFeature& newFeature);
};

#endif // QVISUALIZER_H_INCLUDED
