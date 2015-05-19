#include "../include/Visualizer/Qvisualizer.h"
#include <memory>
#include <stdexcept>
#include <chrono>

using namespace putslam;

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

QGLVisualizer::QGLVisualizer(void) {
}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFile) :
        config(configFile) {
    tinyxml2::XMLDocument configXML;
    std::string filename = "../../resources/" + configFile;
    configXML.LoadFile(filename.c_str());
    if (configXML.ErrorID())
        std::cout << "unable to load visualizer config file.\n";
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {
}

/// Observer update
void QGLVisualizer::update(MapModifier& mapModifier) {
    std::cout << "update visualizer\n";
    bufferMapVisualization.mtxBuffer.lock();
    mapModifier.mtxBuffer.lock();
    if (mapModifier.addFeatures()) {
        bufferMapVisualization.features2add.insert(mapModifier.features2add.begin(),
                mapModifier.features2add.end());
        mapModifier.features2add.clear();
    }
    if (mapModifier.updateFeatures()) {
        bufferMapVisualization.features2update.insert(mapModifier.features2update.begin(),
                mapModifier.features2update.end());
        mapModifier.features2update.clear();
    }
    if (mapModifier.addPoses()) {
        if (camTrajectory.size()==0){//set initial camera pose -- looks at the initial point of the trajectory
            qglviewer::Vec camPos(mapModifier.poses2add[0].pose(0,3), mapModifier.poses2add[0].pose(1,3), mapModifier.poses2add[0].pose(2,3));
            camera()->setPosition(camPos+qglviewer::Vec(1,1,1));
            camera()->lookAt(camPos);
        }
        bufferMapVisualization.poses2add.insert(bufferMapVisualization.poses2add.end(), mapModifier.poses2add.begin(),
                mapModifier.poses2add.end());
        mapModifier.poses2add.clear();
    }
    if (mapModifier.updatePoses()) {
        bufferMapVisualization.poses2update.insert(bufferMapVisualization.poses2update.end(), mapModifier.poses2update.begin(),
                mapModifier.poses2update.end());
        mapModifier.poses2update.clear();
    }
    mapModifier.mtxBuffer.unlock();
    bufferMapVisualization.mtxBuffer.unlock();
}

/// draw objects
void QGLVisualizer::draw(){
    // Here we are in the world coordinate system. Draw unit size axis.
    drawAxis();

    glLineWidth(2.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINE_STRIP);
    for (auto it=camTrajectory.begin();it!=camTrajectory.end();it++){
        glVertex3f(it->pose(0,3), it->pose(1,3), it->pose(2,3));
    }
    glEnd();
    updateMap();
}

/// draw objects
void QGLVisualizer::animate(){
}

/// Update feature
void QGLVisualizer::updateFeature(std::map<int,MapFeature>& featuresMap,
        MapFeature& newFeature) {
    featuresMap[newFeature.id].position = newFeature.position;
}

///update map
void QGLVisualizer::updateMap(){
    bufferMapVisualization.mtxBuffer.lock();
    if (bufferMapVisualization.addFeatures()) {
        featuresMap.insert(bufferMapVisualization.features2add.begin(),
                bufferMapVisualization.features2add.end());
        bufferMapVisualization.features2add.clear();
    }
    if (bufferMapVisualization.updateFeatures()) {
        for (auto it =
                bufferMapVisualization.features2update.begin();
                it != bufferMapVisualization.features2update.end(); it++) {
            updateFeature(featuresMap, it->second);
        }
        bufferMapVisualization.features2update.clear();
    }
    mtxCamTrajectory.lock();
    if (bufferMapVisualization.addPoses()) {
        camTrajectory.insert(camTrajectory.end(), bufferMapVisualization.poses2add.begin(),
                bufferMapVisualization.poses2add.end());
        bufferMapVisualization.poses2add.clear();
    }
    if (bufferMapVisualization.updatePoses()) {
        for (auto it =
                bufferMapVisualization.poses2update.begin();
                it != bufferMapVisualization.poses2update.end(); it++) {
            camTrajectory[it->vertexId].pose = it->pose;
        }
        bufferMapVisualization.features2update.clear();
    }
    mtxCamTrajectory.unlock();
    bufferMapVisualization.mtxBuffer.unlock();
}

/// initialize visualizer
void QGLVisualizer::init(){
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001);
    camera()->setZClippingCoefficient(100.0);

    // Opens help window
    help();

    startAnimation();
}

/// generate help string
std::string QGLVisualizer::help() const{
    std::string text("S i m p l e V i e w e r");
    text += "Use the mouse to move the camera around the object. ";
    text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
    text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
    text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
    text += "Simply press the function key again to restore it. Several keyFrames define a ";
    text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
    text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
    text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
    text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
    text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
    text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
    text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
    text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
    text += "Press <b>Escape</b> to exit the viewer.";
    return text;
}

