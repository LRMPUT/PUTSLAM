#include "Visualizer/Qvisualizer.h"
#include "Defs/eigen3.h"
#include <memory>
#include <cmath>
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
        float const R = 1.0f/(float)(rings-1);
        float const S = 1.0f/(float)(sectors-1);
        unsigned int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
        texcoords.resize(rings * sectors * 2);
        std::vector<GLfloat>::iterator v = vertices.begin();
        std::vector<GLfloat>::iterator n = normals.begin();
        std::vector<GLfloat>::iterator t = texcoords.begin();
        for(r = 0; r < rings; r++)
        	for(s = 0; s < sectors; s++) {
                float const y = (float) sin( -M_PI_2 + M_PI * r * R );
                float const x = (float) cos(2*M_PI * s * S) * (float) sin( M_PI * r * R );
                float const z = (float) sin(2*M_PI * s * S) * (float) sin( M_PI * r * R );

                *t++ = (float) s*S;
                *t++ = (float) r*R;

                *v++ = (float) x * radius;
                *v++ = (float) y * radius;
                *v++ = (float) z * radius;

                *n++ = (float) x;
                *n++ = (float) y;
                *n++ = (float) z;
        }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
        for(r = 0; r < rings-1; r++)
        	for(s = 0; s < sectors-1; s++) {
                *i++ = (GLushort) ((float) r * (float)sectors + (float) s);
                *i++ = (GLushort) ((float) r * (float)sectors + (float) (s+1));
                *i++ = (GLushort) ((float) (r+1) * (float)sectors + (float) (s+1));
                *i++ = (GLushort) ((float) (r+1) * (float)sectors + (float) s);
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
        glDrawElements(GL_QUADS, (int) indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
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

/// Draw ellipsoid
void QGLVisualizer::drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, double fA, double fB, double fC) const {
    float tStep = (float)(M_PI) / (float)uiSlices;
    float sStep = (float)(M_PI) / (float)uiStacks;
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK,GL_COLOR);
    for(double t = -M_PI/2.0; t <= (M_PI/2.0f)+.0001f; t += tStep) {
        glBegin(GL_TRIANGLE_STRIP);
        for(double s = -M_PI; s <= M_PI+.0001f; s += sStep) {
            glVertex3d((fA * cos(t) * cos(s)), (fB * cos(t) * sin(s)), (fC * sin(t)));
            double norm = sqrt(pow(fA * cos(t) * cos(s),2.0)+pow(fB * cos(t) * sin(s),2.0) + pow(fC * sin(t),2.0));
            glNormal3d((fA * cos(t) * cos(s))/norm, (fB * cos(t) * sin(s))/norm, (fC * sin(t))/norm);
            glVertex3d(fA * cos(t+tStep) * cos(s), fB * cos(t+tStep) * sin(s), fC * sin(t+tStep));
            norm = sqrt(pow(fA * cos(t+tStep) * cos(s),2.0)+pow(fB * cos(t+tStep) * sin(s),2.0) + pow(fC * sin(t+tStep),2.0));
            glNormal3d((fA * cos(t+tStep) * cos(s))/norm, (fB * cos(t+tStep) * sin(s))/norm, (fC * sin(t+tStep))/norm);
        }
        glEnd();
    }
}

/// Draw ellipsoid
void QGLVisualizer::drawEllipsoid(const Vec3& pos, const Mat33& covariance) const{
    Eigen::SelfAdjointEigenSolver<Mat33> es;
    es.compute(covariance);
    Mat33 V(es.eigenvectors());
    double GLmat[16]={V(0,0), V(1,0), V(2,0), 0, V(0,1), V(1,1), V(2,1), 0, V(0,2), V(1,2), V(2,2), 0, pos.x(), pos.y(), pos.z(), 1};
    glPushMatrix();
        glMultMatrixd(GLmat);
        drawEllipsoid(10,10,sqrt(es.eigenvalues()(0))*config.ellipsoidScale, sqrt(es.eigenvalues()(1))*config.ellipsoidScale, sqrt(es.eigenvalues()(2))*config.ellipsoidScale);
    glPopMatrix();
}

/// Observer update
void QGLVisualizer::update(MapModifier& mapModifier) {
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
            qglviewer::Vec camPos((float)mapModifier.poses2add[0].pose(0,3), (float)mapModifier.poses2add[0].pose(1,3), (float)mapModifier.poses2add[0].pose(2,3));
            camera()->setPosition(camPos+qglviewer::Vec(1,1,1));
            camera()->lookAt(camPos);
        }
        else if (config.flyingCamera){
            Mat34 camPose = mapModifier.poses2add.back().pose;
            Mat34 camPose1((Quaternion(0, 0, 0, 1)*camPose.rotation()*Quaternion(0, 1, 0, 0))*Vec3(-camPose(0,3),-camPose(1,3),-camPose(2,3)));
            /*qglviewer::Vec camPos(mapModifier.poses2add.back().pose(0,3), mapModifier.poses2add.back().pose(1,3), mapModifier.poses2add.back().pose(2,3));
            camera()->setPosition(camPos+qglviewer::Vec(0,0,0));
            Quaternion quat(mapModifier.poses2add.back().pose.rotation());
            qglviewer::Quaternion q(quat.w(), quat.x(), quat.y(), quat.z());
            camera()->setOrientation(q*qglviewer::Quaternion(0, 0, 1, 0));*/
            //glScalef(1,-1,1);
            GLdouble GLmat[16]={camPose1(0,0), camPose1(1,0), camPose1(2,0), camPose1(3,0), camPose1(0,1), camPose1(1,1), camPose1(2,1), camPose1(3,1), camPose1(0,2), camPose1(1,2), camPose1(2,2), camPose1(3,2), camPose1(0,3), camPose1(1,3), camPose1(2,3), camPose1(3,3)};
            camera()->setFromModelViewMatrix(GLmat);
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
void QGLVisualizer::update(const cv::Mat& color, const cv::Mat& depth, int frameNo){
    //pointClouds.push_back(std::make_pair(frameNo, cloud));
    mtxImages.lock();
    colorImagesBuff.push_back(color);
    depthImagesBuff.push_back(depth);
    imagesIds.push_back(frameNo);
    mtxImages.unlock();
}

/// Observer update
void QGLVisualizer::update(std::vector<Edge>& features){
    mtxMeasurementsBuff.lock();
    mtxMeasurements.lock();

    for (Edge &e : features) {
    	Edge *pE = &e;
    	Edge3D* e3D = dynamic_cast<Edge3D*>(pE);
    	if (e3D)
    		measurementsBuff.push_back(*e3D);
    }

    //measurementsBuff.insert(measurementsBuff.end(), features.begin(), features.end());
    mtxMeasurements.unlock();
    mtxMeasurementsBuff.unlock();
}

/// Draw point clouds
void QGLVisualizer::drawPointClouds(void){
    mtxPointClouds.lock();
    for (size_t i = 0;i<pointClouds.size();i++){
        mtxCamTrajectory.lock();
        if (camTrajectory.size()>(size_t)imagesIds[i]){
            Mat34 camPose = camTrajectory[imagesIds[i]].pose;
            mtxCamTrajectory.unlock();
            double GLmat[16]={camPose(0,0), camPose(1,0), camPose(2,0), camPose(3,0), camPose(0,1), camPose(1,1), camPose(2,1), camPose(3,1), camPose(0,2), camPose(1,2), camPose(2,2), camPose(3,2), camPose(0,3), camPose(1,3), camPose(2,3), camPose(3,3)};
            glPushMatrix();
                glMultMatrixd(GLmat);
                glPointSize((float)config.cloudPointSize);
                glCallList(cloudsList[i]);
            glPopMatrix();
        }
    }
    mtxPointClouds.unlock();
}

/// Create point cloud List
GLuint QGLVisualizer::createCloudList(const std::pair<int,PointCloud>& pointCloud){
    // create one display list
    GLuint index = glGenLists(1);

    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glBegin(GL_POINTS);
        for (size_t n = 0; n < pointCloud.second.size(); n++){
            glColor3ub((unsigned char)pointCloud.second[n].r, (unsigned char)pointCloud.second[n].g, (unsigned char)pointCloud.second[n].b);
            glVertex3d(pointCloud.second[n].x, pointCloud.second[n].y, pointCloud.second[n].z);
        }
        glEnd();
    glEndList();
    return index;
}

/// draw objects
void QGLVisualizer::draw(){
    // Here we are in the world coordinate system. Draw unit size axis.
    drawAxis();
    if (config.drawPointClouds){
        drawPointClouds();
    }
    if (config.drawTrajectory){
        glLineWidth((float)config.trajectoryWidth);
        glColor4f((float)config.trajectoryColor.red(), (float)config.trajectoryColor.green(), (float)config.trajectoryColor.blue(), (float)config.trajectoryColor.alpha());
        glBegin(GL_LINE_STRIP);
        mtxCamTrajectory.lock();
        for (auto it=camTrajectory.begin();it!=camTrajectory.end();it++){
            glVertex3d(it->pose(0,3), it->pose(1,3), it->pose(2,3));
        }
        mtxCamTrajectory.unlock();
        glEnd();
    }
    SolidSphere sphereTraj((float)config.trajectoryPointsSize, config.featuresSmoothness, config.featuresSmoothness);
    if (config.drawTrajectoryPoints){
        glColor4f((float)config.trajectoryPointsColor.red(), (float)config.trajectoryPointsColor.green(), (float)config.trajectoryPointsColor.blue(), (float)config.trajectoryPointsColor.alpha());
        mtxCamTrajectory.lock();
        for (auto it=camTrajectory.begin();it!=camTrajectory.end();it++){
            //glPushMatrix();
                sphereTraj.draw((float)it->pose(0,3), (float)it->pose(1,3), (float)it->pose(2,3));
              //  glTranslated(it->pose(0,3), it->pose(1,3), it->pose(2,3));
                //solidSphere(config.trajectoryPointsSize, 10, 10);
                //glutSolidSphere(config.trajectoryPointsSize,6,6);
            //glPopMatrix();
        }
        mtxCamTrajectory.unlock();
        glEnd();
    }
    SolidSphere sphereFeature((float)config.featuresSize, config.featuresSmoothness, config.featuresSmoothness);
    if (config.drawFeatures){
        glColor4f((float)config.featuresColor.red(), (float)config.featuresColor.green(), (float)config.featuresColor.blue(), (float)config.featuresColor.alpha());
        mtxFeaturesMap.lock();
        for (auto it=featuresMap.begin();it!=featuresMap.end();it++){
            //glPushMatrix();
                sphereFeature.draw((float)it->second.position.x(), (float)it->second.position.y(), (float)it->second.position.z());
                //glTranslated(it->second.position.x(), it->second.position.y(), it->second.position.z());
                //solidSphere(config.featuresSize, 10, 10);
                //glutSolidSphere(config.featuresSize,6,6);
            //glPopMatrix();
        }
        mtxFeaturesMap.unlock();
        glEnd();
    }
    if (config.drawPose2Feature){
        glLineWidth((float)config.pose2FeatureWidth);
        glColor4f((float)config.pose2FeatureColor.red(), (float)config.pose2FeatureColor.green(), (float)config.pose2FeatureColor.blue(), (float)config.pose2FeatureColor.alpha());
        glBegin(GL_LINES);
        mtxCamTrajectory.lock();
        mtxFeaturesMap.lock();
        for (auto it=featuresMap.begin();it!=featuresMap.end();it++){
            for (auto itPose=it->second.posesIds.begin();itPose!=it->second.posesIds.end();itPose++){
                glVertex3d(camTrajectory[*itPose].pose(0,3), camTrajectory[*itPose].pose(1,3), camTrajectory[*itPose].pose(2,3));
                glVertex3d(it->second.position.x(), it->second.position.y(), it->second.position.z());
            }
        }
        mtxCamTrajectory.unlock();
        mtxFeaturesMap.unlock();
        glEnd();
    }
    if (config.drawMeasurements){
        mtxMeasurements.lock();
        for (size_t i = 0;i<measurements.size();i++){
            mtxCamTrajectory.lock();
            Mat34 camPose = camTrajectory[measurements[i].fromVertexId].pose;
            mtxCamTrajectory.unlock();
            double GLmat[16]={camPose(0,0), camPose(1,0), camPose(2,0), camPose(3,0), camPose(0,1), camPose(1,1), camPose(2,1), camPose(3,1), camPose(0,2), camPose(1,2), camPose(2,2), camPose(3,2), camPose(0,3), camPose(1,3), camPose(2,3), camPose(3,3)};
            glPushMatrix();
                glMultMatrixd(GLmat);
                glPointSize((float)config.measurementSize);
                glBegin(GL_POINTS);
                glColor4f((float)config.measurementsColor.red(), (float)config.measurementsColor.green(), (float)config.measurementsColor.blue(), (float)config.measurementsColor.alpha());
                glVertex3d(measurements[i].trans.x(),measurements[i].trans.y(),measurements[i].trans.z());
                glEnd();
                if (config.drawEllipsoids){
                    glColor4f((float)config.ellipsoidColor.red(), (float)config.ellipsoidColor.green(), (float)config.ellipsoidColor.blue(), (float)config.ellipsoidColor.alpha());
                    drawEllipsoid(measurements[i].trans, measurements[i].info.inverse());
                }
            glPopMatrix();
        }
        mtxMeasurements.unlock();
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
    if (config.drawPointClouds){
        if (colorImagesBuff.size()>0){
            mtxImages.lock();
            size_t imagesSize = colorImagesBuff.size();
            mtxImages.unlock();
            while (imagesSize){
                mtxImages.lock();
                cv::Mat color(colorImagesBuff.back());
                colorImagesBuff.pop_back();
                cv::Mat depth(depthImagesBuff.back());
                depthImagesBuff.pop_back();
                int frameNo(imagesIds.back());
                imagesSize = colorImagesBuff.size();
                mtxImages.unlock();
                PointCloud cloud;
                sensorModel.convert2cloud(color, depth, cloud);
                mtxPointClouds.lock();
                pointClouds.push_back(std::make_pair(frameNo, cloud));
                cloudsList.push_back(createCloudList(pointClouds.back()));
                mtxPointClouds.unlock();
            }
        }
    }
    mtxMeasurementsBuff.lock();
    if (measurementsBuff.size()>0){
        mtxMeasurements.lock();
        for (auto it = measurementsBuff.begin(); it!=measurementsBuff.end();it++){
            if (((int)(*it).toVertexId>config.measurementFeaturesIds.first)&&
                ((int)(*it).toVertexId<config.measurementFeaturesIds.second)){
                    measurements.push_back(*it);
            }
        }
        //measurements.insert(measurements.end(), measurementsBuff.begin(), measurementsBuff.end());
        mtxMeasurements.unlock();
    }
    measurementsBuff.clear();
    mtxMeasurementsBuff.unlock();
}

/// initialize visualizer
void QGLVisualizer::init(){
    // Light setup
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
    GLfloat specular_color[4] = { 0.99f, 0.99f, 0.99f, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    //Set global ambient light
    GLfloat black[] = {0.99f, 0.99f, 0.99f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);

    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001f);
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

