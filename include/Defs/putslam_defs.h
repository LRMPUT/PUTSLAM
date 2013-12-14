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

/// putslam name space
namespace putslam {

    /// putslam default floating point
    typedef double float_type;

	/// 3 Element vector class
    class Vec3 {
        public:
            /// vector components
            union {
                struct {
                    float_type v1, v2, v3;
                };
                struct {
                    float_type x, y, z;
                };
                float_type v[3];
            };

            /// Default constructor sets the default configuration
            inline Vec3() : v1(0), v2(0), v3(0) {
            }
    };

	/// Matrix representation of SO(3) group of rotations
    class Mat33 {
        public:
            /// Matrix elements
            union {
                struct {
                    float_type m11, m12, m13;
                    float_type m21, m22, m23;
                    float_type m31, m32, m33;
                };
                float_type m[3][3];
            };
            /// Default constructor
            inline Mat33() : m11(1), m12(0), m13(0), m21(0), m22(1), m23(0), m31(0), m32(0), m33(1) {
            }
    };

    /// Quaternion representation of SO(3) group of rotations
    class Quaternion {
        public:
            ///quaternion vector
            union{
                float_type quat[4];
                struct {
                    float_type q1, q2, q3, q4;
                };
                struct {
                    float_type sw, sx, sy, sz;
                };
                float_type q[4];
                struct {
                    float_type w, x, y, z;
                };
            };
            /// Default constructor
            inline Quaternion() : w(1), x(0), y(0), z(0) {
            }
    };

	/// Homogeneous representation of SE(3) rigid body transformations
    class Mat34 {
        public:
            /// sequence of transformation/poses
            typedef std::vector<Mat34> Seq;

            /// rotation matrix
            Mat33 R;
            /// translation
            Vec3 p;

            /// Default constructor sets the default configuration
            inline Mat34() {
            }
    };

    /// Homogenous representation of SE(3) rigid body transformations with scale
    class Mat44 : public Mat34{
        public:
            /// rotation
            float_type scale;

            /// Default constructor sets the default configuration
            inline Mat44() : scale(1){
            }
    };

	/// RGBA colour space
    class RGBA {
        public:
            /// Colour representation
            union {
                union {
                    struct {
                        std::uint8_t r, g, b, a;
                    };
                    float rgb;
                    std::uint8_t rgba_color[4];
                };
                uint32_t rgba;
            };
            /// Default constructor
            inline RGBA() : r(255), g(255), b(255), a(255) {
            }
    };

	/// 3D point representation
    class Point3D {
        public:
            /// Point cloud
            typedef std::vector<Point3D> Cloud;

            /// Position
            Vec3 position;
            /// Colour
            RGBA colour;

            /// Default constructor
            inline Point3D() {
            }
    };

    /// Image representation
    class Image {
        public:
            /// sequence of images
            typedef std::vector<Image> Seq;

            /// 2D image
            cv::Mat data;
            /// XYZRGBA point cloud
            Point3D::Cloud depthData;
            /// timestamp
            double timestamp;

            /// Default constructor
            inline Image() : timestamp(0){
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

            /// image patch
            cv::Mat patch;

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
                Vec3 pose;
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
                    /// Vertex 3D -- feature pose
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

            /// Default constructor
            inline Edge7D() : Edge(EDGE_7D){
            }
    };

    class Vertex {
        public:
            /// Set of Vertexes
            typedef std::vector<Vertex> Seq;
            /// Pointer
            typedef std::unique_ptr<Vertex> Ptr;

            /// Vertex type
            enum Type {
                    /// Vertex 3D -- feature pose
                    VERTEX_3D,
                    /// Vertex 7D -- robot pose
                    VERTEX_7D
            };

            /// Vertex / node id
            uint_fast32_t node_id;

            /// Vertex type
            Type type;

            /// Point cloud
            Point3D::Cloud cloud;

            /// Default constructor
            inline Vertex() : node_id(0){
            }

            /// Overloaded constructor
            inline Vertex(Type _type) : node_id(0), type(_type){
            }
    };

    class Vertex3D : public Vertex {
        public:
            /// Set of Vertexes
            typedef std::vector<Vertex3D> Seq;

            /// Vertex / node
            KeyPoint node3D;

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

            /// Default constructor
            inline Vertex7D() : Vertex{VERTEX_7D}{
            }
    };

    /// Pose-based graph
    class PoseGraph {
        public:
            /// Robot poses -- nodes of the graph
            typedef std::vector<Edge::Ptr> EdgeSet;

            /// Edges of the graph
            typedef std::vector<Vertex::Ptr> VertexSet;
    };
}

#endif // PUTSLAM_DEFS_H_INCLUDED
