/** @file depthSensorModel.h
 *
 * implementation - Kinect/Xtion model
 *
 */

#ifndef DEPTH_SENSOR_MODEL_H_INCLUDED
#define DEPTH_SENSOR_MODEL_H_INCLUDED

#include "../Defs/putslam_defs.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>

using namespace putslam;

class DepthSensorModel {
  public:
    /// Construction
    DepthSensorModel() {};

    /// Construction
    DepthSensorModel(std::string configFile);

    void getPoint(uint_fast16_t u, uint_fast16_t v, float_type depth, Eigen::Vector3d& point3D) const;

    /// inverse model of the sensor
    Eigen::Vector3d inverseModel(float_type x, float_type y, float_type z) const;

    /// u,v [px], depth [m]
    void computeCov(uint_fast16_t u, uint_fast16_t v, float_type depth, Mat33& cov);

    /// point xyz in camera frame
    void computeCov(Eigen::Vector3f point, Mat33& cov);

    /// compute information matrix
    Mat33 informationMatrix(float_type x, float_type y, float_type z);

    /// compute information matrix using image coordinates
    Mat33 informationMatrixFromImageCoordinates(float_type u, float_type v, float_type z);

    /// Create point cloud from current RGB and depth image
    void convert2cloud(const cv::Mat& color, const cv::Mat& depth, PointCloud& cloud);

    /// compute uncertainty from normal vector
    Mat33 uncertinatyFromNormal(const Vec3& normal);

    /// compute uncertainty from rgb gradient vector
    Mat33 uncertinatyFromRGBGradient(const Vec3& grad);

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
                std::cout << "unable to load sensor config file: error = " << config.ErrorID() << std::endl;;

			// Get dataset config
			std::string datasetCfgFilename =
					config.FirstChildElement("Model")->Attribute("datasetFile");
			config.FirstChildElement("Model")->QueryIntAttribute("verbose", &verbose);
			tinyxml2::XMLDocument datasetCfg;
			filename = "../../resources/" + datasetCfgFilename;
			datasetCfg.LoadFile(filename.c_str());
			if (datasetCfg.ErrorID()) {
				std::cout << "Unable to load dataset config file: "
						<< configFilename << std::endl;
			}

            tinyxml2::XMLElement * model = datasetCfg.FirstChildElement( "Model" );
            model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fu", &focalLength[0]);
            model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fv", &focalLength[1]);
            model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cu", &focalAxis[0]);
            model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cv", &focalAxis[1]);
            model->FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaU", &varU);
            model->FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaV", &varV);
            model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c3", &distVarCoefs[0]);
            model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c2", &distVarCoefs[1]);
            model->FirstChildElement( "imageSize" )->QueryIntAttribute("sizeU", &imageSize[0]);
            model->FirstChildElement( "imageSize" )->QueryIntAttribute("sizeV", &imageSize[1]);
            model->FirstChildElement( "normalModel" )->QueryDoubleAttribute("scaleUncertaintyNormal", &scaleUncertaintyNormal);
            model->FirstChildElement( "gradientModel" )->QueryDoubleAttribute("scaleUncertaintyGradient", &scaleUncertaintyGradient);
            datasetCfg.FirstChildElement( "datasetPath" )->QueryDoubleAttribute("depthImageScale", &depthImageScale);

            tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
            double query[4];
            posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
            double queryPos[4];
            posXML->QueryDoubleAttribute("x", &queryPos[0]); posXML->QueryDoubleAttribute("y", &queryPos[1]); posXML->QueryDoubleAttribute("z", &queryPos[2]);
            pose = Quaternion (query[0], query[1], query[2], query[3])*Vec3(queryPos[0], queryPos[1], queryPos[2]);


            if ( verbose > 0)
            {
				std::cout<<"DepthSensorModel.h readParameters:" << std::endl;

				std::cout << "\t fu = " << focalLength[0] << std::endl;
				std::cout << "\t fv = " << focalLength[1] << std::endl;
				std::cout << "\t Cu = " << focalAxis[0] << std::endl;
				std::cout << "\t Cv = " << focalAxis[1] << std::endl;
				std::cout << "\t sigmaU = " << varU << std::endl;
				std::cout << "\t sigmaV = " << varV << std::endl;
				std::cout << "\t c3 = " << distVarCoefs[0] << std::endl;
				std::cout << "\t c2 = " << distVarCoefs[1] << std::endl;
				std::cout << "\t sizeU = " << imageSize[0] << std::endl;
				std::cout << "\t sizeV = " << imageSize[1] << std::endl;
                std::cout << "\t depthImageScale = " << depthImageScale << std::endl;
				std::cout << "\t scaleUncertaintyNormal = "
						<< scaleUncertaintyNormal << std::endl;
				std::cout << "\t scaleUncertaintyGradient = "
						<< scaleUncertaintyGradient << std::endl;

				if ( verbose > 1)
					getchar();
            }


        }
        public:
        	int verbose;
            float_type focalLength[2];
            float_type focalAxis[2];
            float_type varU, varV;// variance u,v
            float_type distVarCoefs[4];
            int imageSize[2];//[sizeU, sizeV]
            Mat34 pose; // kinect pose in robot's coordination frame
            float_type scaleUncertaintyNormal;
            float_type scaleUncertaintyGradient;
            float_type depthImageScale;
    };

    Config config;

    protected:
        Mat33 PHCPModel;//pin-hole camera projection model
        Mat33 Ruvd; //covariance matrix for [u,v,disp]

        /// normalize vector
        void normalizeVector(Vec3& normal) const;
};

#endif // DEPTH_SENSOR_MODEL_H_INCLUDED
