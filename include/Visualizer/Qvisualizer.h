/** @file QVisualizer.h
 *
 * implementation - QGLVisualizer
 *
 */

#ifndef QVISUALIZER_H_INCLUDED
#define QVISUALIZER_H_INCLUDED

#include "visualizer.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include <QGLViewer/qglviewer.h>
#include <iostream>

namespace putslam {
/// create a single visualizer
Visualizer* createVisualizerQGL(void);
/// create a single visualizer - overloaded
Visualizer* createVisualizerQGL(std::string configFileGrabber);
}

using namespace putslam;

/// Map implementation
class QGLVisualizer: public Visualizer, public QGLViewer{
public:
    /// Pointer
    typedef std::unique_ptr<QGLVisualizer> Ptr;

    /// Construction
    QGLVisualizer(void);

    /// Construction
    QGLVisualizer(std::string configFile);

    /// Destruction
    ~QGLVisualizer(void);

    /// Name of the map
    const std::string& getName() const;

    /// visualize
    void visualize(void);

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

    /// draw objects
    void draw();

    /// initialize visualizer
    void init();

    /// generate help string
    std::string help() const;
};

#endif // QVISUALIZER_H_INCLUDED
