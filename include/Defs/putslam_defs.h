/** @file putslam_defs.h
*
* Handest definitions
*
*/

#ifndef PUTSLAM_DEFS_H_INCLUDED
#define PUTSLAM_DEFS_H_INCLUDED

#include <cstdint>
#include <vector>
#include <memory>
#include <cmath>
#include "opencv2/core/core.hpp"
#include <opencv2/features2d.hpp>
#include "../../3rdParty/Eigen/Geometry"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

/// putslam name space
namespace putslam {

    /// putslam default floating point
    typedef double float_type;

    /// 3 element vector class
    typedef Eigen::Translation<float_type,3> Vec3;

    /// Matrix representation of SO(3) group of rotations
    typedef Eigen::Matrix<float_type,3,3> Mat33;

    /// Information Matrix of SE(3) transformation
    typedef Eigen::Matrix<float_type,6,6> Mat66;

    /// Quaternion representation of SO(3) group of rotations
    typedef Eigen::Quaternion<float_type> Quaternion;

	/// Homogeneous representation of SE(3) rigid body transformations
    typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

    /// Homogenous representation of SE(3) rigid body transformations with scale
    class Mat44 : public Mat34{
        public:
            /// rotation
            float_type scale;

            /// Default constructor sets the default configuration
            inline Mat44() : scale(1){
            }
    };

    /// 3D point representation
    typedef pcl::PointXYZRGBA Point3D;

    /// 3D point cloud representation
    typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

    /// Sensor Frame representation
    class SensorFrame {
        public:
            /// sequence of images
            typedef std::vector<SensorFrame> Seq;

            /// 2D image
            cv::Mat image;

            /// 2D image
            cv::Mat depth;

            /// XYZRGBA point cloud
            PointCloud cloud;

            /// timestamp
            float_type timestamp;

            /// Default constructor
            inline SensorFrame() : timestamp(0){
            }
    };

    /// 2D image feature
    class ImageFeature {
        public:
            /// set of features
            typedef std::vector<ImageFeature> Seq;

            /// 2D feature location
            union {
                struct {
                    uint_fast16_t u;
                    uint_fast16_t v;
                };
                uint_fast16_t coord[2];
            };

            /// Image patch
            cv::Mat patch;

            /// Depth
            float_type depth;

            /// Default constructor
            inline ImageFeature() :  u(0), v(0) {
            }
    };

    class ExtendedDescriptor{
        public:
            /// set of descriptors
            typedef std::vector<ExtendedDescriptor> Seq;

            /// OpenCV descriptor
            std::unique_ptr<cv::DescriptorExtractor> descriptor;

            /// camera orientation
            Quaternion cameraOrientation;

            /// Constructor
            ExtendedDescriptor(){};

            /// Constructor
            ExtendedDescriptor(Quaternion& cameraOrient) : cameraOrientation(cameraOrient){};
    };

    class RGBDFeature{
        public:
            /// Position of the feature
            Vec3 position;

            /// set of descriptors
            std::vector<ExtendedDescriptor> desciptors;

            RGBDFeature(void){};

            RGBDFeature(Vec3& _position) : position(_position){
            };
    };

    class MapFeature : RGBDFeature{
        /// id of the feature
        unsigned int id;

        /// Constructor
        MapFeature(){};

        /// Constructor
        MapFeature(unsigned int _id, Vec3& _position) : RGBDFeature(_position), id(_id) {};

        /// Constructor
        MapFeature(unsigned int _id) : id(_id){};
    };

    /// Key Point
    class KeyPoint {
        public:
            /// set of keypoints
            typedef std::vector<KeyPoint> Seq;

            /// 3D feature
            Vec3 depthFeature;

            /// 2D feature
            ImageFeature imageFeature;

            /// Keypoint id
            uint_fast32_t keypointId;

            /// Default constructor
            inline KeyPoint(){
            }

            /// Overloaded constructor
            inline KeyPoint(Vec3& _depth) : depthFeature(_depth){
            }
    };

    /// Robot pose
    class RobotPose {
        public:
            /// set of keypoints
            typedef std::vector<RobotPose> Seq;

            /// Position
            Vec3 pos;

            /// Orientation
            Quaternion rot;

            /// Default constructor
            inline RobotPose(){
            }

            /// Overloaded constructor
            inline RobotPose(Vec3& _pos, Quaternion& _rot) : pos(_pos), rot(_rot){
            }
    };

    /// Edge of a graph
    class Edge {
        public:
            /// set of Edges
            typedef std::vector<Edge> Seq;
            /// Pointer
            typedef std::unique_ptr<Edge> Ptr;

            /// Vertex type
            enum Type {
                    /// Vertex 3D -- feature position
                    EDGE_3D,
                    /// Vertex SE(3) -- robot pose
                    EDGE_SE3
            };

            /// Vertex type
            Type type;

            /// Edge id
            unsigned int id;

            /// Node connected by the edge
            uint_fast32_t toVertexId;

            /// Node connected by the edge
            uint_fast32_t fromVertexId;

            /// Default constructor
            inline Edge(){
            }

            /// Overloaded constructor
            inline Edge(Type _type) : type(_type){
            }

            /// Overloaded constructor
            inline Edge(Type _type, uint_fast32_t _fromVertexId, uint_fast32_t _toVertexId) : type(_type),
                toVertexId(_toVertexId),
                fromVertexId(_fromVertexId) {
            }
    };

    /// 3D (x,y,z) Edge of a graph
    class Edge3D : public Edge {
        public:
            /// set of Edges3D
            typedef std::vector<Edge3D> Seq;

            /// translation between nodes
            Vec3 trans;

            /// Information matrix
            Mat33 info;

            /// Default constructor
            inline Edge3D() : Edge(EDGE_3D){
            }

            /// Overloaded constructor
            inline Edge3D(Vec3& _trans, Mat33& _info, uint_fast32_t _fromVertexId, uint_fast32_t _toVertexId) :
                Edge(EDGE_3D, _fromVertexId, _toVertexId),
                trans(_trans),
                info(_info){
            }
    };

    /// SE(3) Edge of a graph (x,y,z + quat(4))
    class EdgeSE3 : public Edge {
        public:
            /// set of EdgesSE3
            typedef std::vector<EdgeSE3> Seq;

            /// translation and rotation between nodes
            RobotPose trans;

            /// Information matrix
            Mat66 info;

            /// Default constructor
            inline EdgeSE3() : Edge(EDGE_SE3){
            }

            /// Overloaded constructor
            inline EdgeSE3(RobotPose& _trans, Mat66& _info, uint_fast32_t _fromVertexId, uint_fast32_t _toVertexId) :
                trans(_trans),
                Edge(EDGE_SE3, _fromVertexId, _toVertexId),
                info(_info){
            }
    };

    class Vertex {
        public:
            /// Set of Vertices
            typedef std::vector<Vertex> Seq;

            /// Vertex type
            enum Type {
                    /// Vertex 3D -- feature position
                    VERTEX3D,
                    /// Vertex SE(3) -- robot pose
                    VERTEXSE3
            };

            /// Vertex type
            Type type;

            /// Vertex / node id
            uint_fast32_t vertexId;

            /// timestamp
            float_type timestamp;

            /// Default constructor
            inline Vertex(){
            }

            /// Overloaded constructor
            inline Vertex(Type _type, uint_fast32_t _vertexId) : type(_type), vertexId(_vertexId){
            }
    };

    class Vertex3D : public Vertex {
        public:
            /// Set of Vertices
            typedef std::vector<Vertex3D> Seq;

            /// Vertex / node
            KeyPoint keypoint;

            /// Default constructor
            inline Vertex3D(void) : Vertex(VERTEX3D, 0){
            }

            /// Overloaded constructor
            inline Vertex3D(uint_fast32_t _vertexId, Vec3& _pos) :
                Vertex(VERTEX3D, _vertexId),
                keypoint(_pos){
            }
    };

    class VertexSE3 : public Vertex {
        public:
            /// Set of Vertices
            typedef std::vector<VertexSE3> Seq;

            /// Vertex / node
            RobotPose nodeSE3;

            /// Point cloud
            PointCloud cloud;

            /// Set of keypoints
            KeyPoint::Seq keypoints;

            /// Default constructor
            inline VertexSE3(void) : Vertex(VERTEXSE3, 0){
            }

            /// Overloaded constructor
            inline VertexSE3(uint_fast32_t _vertexId, Vec3& _pos, Quaternion& _rot) :
                Vertex(VERTEXSE3, _vertexId),
                nodeSE3(_pos, _rot) {
            }
    };

    /// Pose-based graph
    class PoseGraph {
        public:
            /// Robot poses -- nodes of the graph
            typedef std::vector<std::unique_ptr<Edge>> EdgeSet;

            /// Edges of the graph
            typedef std::vector<std::unique_ptr<Vertex>> VertexSet;

			/// Edges
			EdgeSet edges;

            /// Edges
            EdgeSet prunedEdges;

			/// Vertices
			VertexSet vertices;
    };
    //exception class goes here

}

#endif // PUTSLAM_DEFS_H_INCLUDED
