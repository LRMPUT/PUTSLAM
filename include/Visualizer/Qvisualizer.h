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
#include "../include/Grabber/depthSensorModel.h"
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
            model->FirstChildElement( "camera" )->QueryBoolAttribute("flyingCamera", &flyingCamera);
            model->FirstChildElement( "pointCloud" )->QueryBoolAttribute("drawPointClouds", &drawPointClouds);
            model->FirstChildElement( "pointCloud" )->QueryIntAttribute("cloudPointSize", &cloudPointSize);
            // measurements
            model->FirstChildElement( "measurements" )->QueryBoolAttribute("drawMeasurements", &drawMeasurements);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("size", &measurementSize);
            model->FirstChildElement( "measurements" )->QueryIntAttribute("featureIDMin", &measurementFeaturesIds.first);
            model->FirstChildElement( "measurements" )->QueryIntAttribute("featureIDMax", &measurementFeaturesIds.second);
            measurementsColor.setRedF(rgba[0]); measurementsColor.setGreenF(rgba[1]);
            measurementsColor.setBlueF(rgba[2]); measurementsColor.setAlphaF(rgba[3]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidRed", &rgba[0]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidGreen", &rgba[1]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidBlue", &rgba[2]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidAlpha", &rgba[3]);
            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidScale", &ellipsoidScale);
            model->FirstChildElement( "measurements" )->QueryBoolAttribute("drawEllipsoids", &drawEllipsoids);
            ellipsoidColor.setRedF(rgba[0]); ellipsoidColor.setGreenF(rgba[1]);
            ellipsoidColor.setBlueF(rgba[2]); ellipsoidColor.setAlphaF(rgba[3]);

            model->FirstChildElement( "opencv" )->QueryBoolAttribute("showFrames", &showFrames);
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

        /// flying camera effect
        bool flyingCamera;

        /// draw point clouds
        bool drawPointClouds;

        /// point size
        int cloudPointSize;

        /// Draw measured feature positions
        bool drawMeasurements;

        /// Measurement color
        QColor measurementsColor;

        /// measured feature position size
        double measurementSize;

        /// id range of measured features (min, max)
        std::pair<int,int> measurementFeaturesIds;

        /// Draw ellipsoids
        bool drawEllipsoids;

        /// Measurement color
        QColor ellipsoidColor;

        /// measured feature position size
        double ellipsoidScale;

        /// show RGB frames
        bool showFrames;
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

    /// Observer update
    void update(const cv::Mat& color, const cv::Mat& depth, int frameNo);

    /// Observer update
    void update(std::vector<Edge>& features);

    /// Set depth sensor model
    inline void setDepthSensorModel(const DepthSensorModel& model){ sensorModel = model;};

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

    //pair -- pose id and point cloud
    std::vector<std::pair<int,PointCloud>> pointClouds;

    std::vector<GLuint> cloudsList;

    /// mutex for critical section - point clouds
    std::mutex mtxPointClouds;

    /// buffer color images
    std::vector<cv::Mat> colorImagesBuff;

    /// buffer depth images
    std::vector<cv::Mat> depthImagesBuff;

    /// buffer depth images
    std::vector<int> imagesIds;

    /// mutex for critical section - images
    std::mutex mtxImages;

    /// Sensor model
    DepthSensorModel sensorModel;

    /// Measurements (SE3 to 3D feature)
    std::vector<Edge3D> measurements;

    /// mutex for critical section - pose graph
    std::mutex mtxMeasurements;

    /// Measurements (SE3 to 3D feature)
    std::vector<Edge3D> measurementsBuff;

    /// mutex for critical section - pose graph
    std::mutex mtxMeasurementsBuff;

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

    /// Draw point clouds
    void drawPointClouds(void);

    /// Create point cloud List
    GLuint createCloudList(const std::pair<int,PointCloud>& pointCloud);

    /// Draw ellipsoid
    void drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, double fA, double fB, double fC) const;

    /// Draw ellipsoid
    void drawEllipsoid(const Vec3& pos, const Mat33& covariance) const;
};

#endif // QVISUALIZER_H_INCLUDED
