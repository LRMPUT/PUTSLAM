#include "../include/Visualizer/Qvisualizer.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>

using namespace putslam;

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

class SolidSphere
{
protected:
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> normals;
    std::vector<GLfloat> texcoords;
    std::vector<GLushort> indices;

public:
    SolidSphere(float radius, unsigned int rings, unsigned int sectors)
    {
        float const R = 1./(float)(rings-1);
        float const S = 1./(float)(sectors-1);
        int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
        texcoords.resize(rings * sectors * 2);
        std::vector<GLfloat>::iterator v = vertices.begin();
        std::vector<GLfloat>::iterator n = normals.begin();
        std::vector<GLfloat>::iterator t = texcoords.begin();
        for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
                float const y = sin( -M_PI_2 + M_PI * r * R );
                float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
                float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

                *t++ = s*S;
                *t++ = r*R;

                *v++ = x * radius;
                *v++ = y * radius;
                *v++ = z * radius;

                *n++ = x;
                *n++ = y;
                *n++ = z;
        }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
        for(r = 0; r < rings-1; r++) for(s = 0; s < sectors-1; s++) {
                *i++ = r * sectors + s;
                *i++ = r * sectors + (s+1);
                *i++ = (r+1) * sectors + (s+1);
                *i++ = (r+1) * sectors + s;
        }
    }

    void draw(GLfloat x, GLfloat y, GLfloat z)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslatef(x,y,z);
        glEnable(GL_NORMALIZE);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);

        glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);
        glDrawElements(GL_QUADS, indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
        glPopMatrix();
    }
};

QGLVisualizer::QGLVisualizer(void) {
}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config){

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
        else if (config.flyingCamera){
            qglviewer::Vec camPos(-mapModifier.poses2add.back().pose(0,3), mapModifier.poses2add.back().pose(1,3), mapModifier.poses2add.back().pose(2,3));
            camera()->setPosition(camPos+qglviewer::Vec(0,0,0));
            Quaternion quat(mapModifier.poses2add.back().pose.rotation());
            qglviewer::Quaternion q(quat.w(), quat.x(), quat.y(), quat.z());
            camera()->setOrientation(q*qglviewer::Quaternion(0, 0, 1, 0));
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

/// Observer update
void QGLVisualizer::update(const putslam::PointCloud& cloud, int frameNo){

}

/// draw objects
void QGLVisualizer::draw(){
    // Here we are in the world coordinate system. Draw unit size axis.
    drawAxis();

    if (config.drawTrajectory){
        glLineWidth(config.trajectoryWidth);
        glColor4f(config.trajectoryColor.red(), config.trajectoryColor.green(), config.trajectoryColor.blue(), config.trajectoryColor.alpha());
        glBegin(GL_LINE_STRIP);
        for (auto it=camTrajectory.begin();it!=camTrajectory.end();it++){
            glVertex3f(-it->pose(0,3), it->pose(1,3), it->pose(2,3));
        }
        glEnd();
    }
    SolidSphere sphereTraj(config.trajectoryPointsSize, config.featuresSmoothness, config.featuresSmoothness);
    if (config.drawTrajectoryPoints){
        glColor4f(config.trajectoryPointsColor.red(), config.trajectoryPointsColor.green(), config.trajectoryPointsColor.blue(), config.trajectoryPointsColor.alpha());
        for (auto it=camTrajectory.begin();it!=camTrajectory.end();it++){
            //glPushMatrix();
                sphereTraj.draw(-it->pose(0,3), it->pose(1,3), it->pose(2,3));
              //  glTranslated(it->pose(0,3), it->pose(1,3), it->pose(2,3));
                //solidSphere(config.trajectoryPointsSize, 10, 10);
                //glutSolidSphere(config.trajectoryPointsSize,6,6);
            //glPopMatrix();
        }
        glEnd();
    }
    SolidSphere sphereFeature(config.featuresSize, config.featuresSmoothness, config.featuresSmoothness);
    if (config.drawFeatures){
        glColor4f(config.featuresColor.red(), config.featuresColor.green(), config.featuresColor.blue(), config.featuresColor.alpha());
        for (auto it=featuresMap.begin();it!=featuresMap.end();it++){
            //glPushMatrix();
                sphereFeature.draw(-it->second.position.x(), it->second.position.y(), it->second.position.z());
                //glTranslated(it->second.position.x(), it->second.position.y(), it->second.position.z());
                //solidSphere(config.featuresSize, 10, 10);
                //glutSolidSphere(config.featuresSize,6,6);
            //glPopMatrix();
        }
        glEnd();
    }
    if (config.drawPose2Feature){
        glLineWidth(config.pose2FeatureWidth);
        glColor4f(config.pose2FeatureColor.red(), config.pose2FeatureColor.green(), config.pose2FeatureColor.blue(), config.pose2FeatureColor.alpha());
        glBegin(GL_LINES);
        for (auto it=featuresMap.begin();it!=featuresMap.end();it++){
            for (auto itPose=it->second.posesIds.begin();itPose!=it->second.posesIds.end();itPose++){
                glVertex3f(-camTrajectory[*itPose].pose(0,3), camTrajectory[*itPose].pose(1,3), camTrajectory[*itPose].pose(2,3));
                glVertex3f(-it->second.position.x(), it->second.position.y(), it->second.position.z());
            }
        }
        glEnd();
    }
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
        bufferMapVisualization.poses2update.clear();
    }
    mtxCamTrajectory.unlock();
    bufferMapVisualization.mtxBuffer.unlock();
}

/// initialize visualizer
void QGLVisualizer::init(){
    // Light setup
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
    GLfloat specular_color[4] = { 0.8f, 0.8f, 0.8f, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001);
    camera()->setZClippingCoefficient(100.0);

    setBackgroundColor(config.backgroundColor);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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

