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

    /// 3D feature
    class DepthFeature {
        public:
            /// set of depth features
            typedef std::vector<DepthFeature> Seq;

            /// 3D feature location
            union {
                struct{
                    float_type x;
                    float_type y;
                    float_type z;
                };
                float_type coord[3];
            };
            /// Default constructor
            inline DepthFeature(){
            }
    };

    /// Key Point
    class KeyPoint {
        public:
            /// set of keypoints
            typedef std::vector<KeyPoint> Seq;

            /// 3D feature
            DepthFeature depth_feature;

            /// 2D feature
            ImageFeature image_feature;

            /// Keypoint id
            uint_fast32_t keypoint_id;

            /// Default constructor
            inline KeyPoint(){
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
                    /// Vertex 7D -- robot pose
                    EDGE_7D
            };

            /// Vertex type
            Type type;

            /// Nodes connected by the edge
            uint_fast32_t nodes[2];

            /// Default constructor
            inline Edge(){
            }

            /// Overloaded constructor
            inline Edge(Type _type) : type(_type) {
            }
    };

    /// 3D (x,y,z) Edge of a graph
    class Edge3D : Edge {
        public:
            /// set of Edges3D
            typedef std::vector<Edge3D> Seq;

            /// translation between nodes
            Vec3 trans;

            /// Information matrix
            Mat33 info;

            /// Overloaded constructor
            inline Edge3D() : Edge(EDGE_3D){
            }
    };

    /// 7D Edge of a graph (x,y,z + quat(4))
    class Edge7D : Edge {
        public:
            /// set of Edges7D
            typedef std::vector<Edge7D> Seq;

            /// translation between nodes
            Vec3 trans;

            /// Rotation between nodes
            Quaternion quat;

            /// Information matrix
            Mat66 info;

            /// Default constructor
            inline Edge7D() : Edge(EDGE_7D){
            }
    };

    class Vertex {
        public:
            /// Set of Vertexes
            typedef std::vector<Vertex> Seq;

            /// Vertex type
            enum Type {
                    /// Vertex 3D -- feature position
                    VERTEX_3D,
                    /// Vertex 7D -- robot pose
                    VERTEX_7D
            };

            /// Vertex type
            Type type;

            /// Default constructor
            inline Vertex(){
            }

            /// Overloaded constructor
            inline Vertex(Type _type) : type(_type){
            }
    };

    class Vertex3D : public Vertex {
        public:
            /// Set of Vertexes
            typedef std::vector<Vertex3D> Seq;

            /// Vertex / node
            KeyPoint keypoint;

            /// Default constructor
            inline Vertex3D() : Vertex(VERTEX_3D){
            }
    };

    class Vertex7D : public Vertex {
        public:
            /// Set of Vertexes
            typedef std::vector<Vertex7D> Seq;

            /// Vertex / node
            RobotPose node7D;

            /// Point cloud
            PointCloud cloud;

            /// Vertex / node id
            uint_fast32_t vertex_id;

            /// Vertex / node id
            KeyPoint::Seq keypoints;

            /// Default constructor
            inline Vertex7D() : Vertex(VERTEX_7D){
            }
    };

    /// Pose-based graph
    class PoseGraph {
        public:
            /// Robot poses -- nodes of the graph
            typedef std::vector<Edge*> EdgeSet;

            /// Edges of the graph
            typedef std::vector<Vertex*> VertexSet;

			/// Edges
			EdgeSet edges;

			/// Vertices
			VertexSet vertices;
    };
}

#endif // PUTSLAM_DEFS_H_INCLUDED
