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

    /// Construction
    QGLVisualizer(void);

    /// Construction
    QGLVisualizer(std::string configFile);

    /// Destruction
    ~QGLVisualizer(void);

    /// Observer update
    void update(MapModifier& mapModifier);

    class Config{
      public:
        Config() {
        }
        Config(std::string configFilename){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load Map config file.\n";
            tinyxml2::XMLElement * model = config.FirstChildElement( "VisualizerConfig" );
            //model->FirstChildElement( "parameters" )->QueryBoolAttribute("useUncertainty", &useUncertainty);
            //model->FirstChildElement( "parameters" )->QueryBoolAttribute("fixVertices", &fixVertices);
            //model->FirstChildElement( "parameters" )->QueryIntAttribute("weakFeatureThr", &weakFeatureThr);
            //model->FirstChildElement( "parameters" )->QueryFloatAttribute("edges3DPrunningThreshold", &edges3DPrunningThreshold);

        }
        public:
    };

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
