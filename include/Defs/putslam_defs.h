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
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <mutex>
#include <set>
#include <iostream>
#include <map>

/// putslam name space
namespace putslam {

/// putslam default floating point
typedef double float_type;

/// 3 element vector class
typedef Eigen::Translation<float_type, 3> Vec3;

/// Matrix representation of SO(3) group of rotations
typedef Eigen::Matrix<float_type, 3, 3> Mat33;

/// Information Matrix of SE(3) transformation
typedef Eigen::Matrix<float_type, 6, 6> Mat66;

/// Quaternion representation of SO(3) group of rotations
typedef Eigen::Quaternion<float_type> Quaternion;

/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

/// Homogenous representation of SE(3) rigid body transformations with scale
class Mat44: public Mat34 {
public:
	/// rotation
	float_type scale;

	/// Default constructor sets the default configuration
	inline Mat44() :
			scale(1) {
	}
};

/// 3D point representation
class Point3D{
public:
    float_type x,y,z,r,g,b,a;
};

/// 3D point cloud representation
typedef std::vector<Point3D> PointCloud;

/// Sensor Frame representation
class SensorFrame {
public:
	/// sequence of images
	typedef std::vector<SensorFrame> Seq;

	/// 2D image
	cv::Mat rgbImage;

	/// 2D image
	cv::Mat depthImage;

	/// Scale of the data in depthImage
	double depthImageScale;

	/// XYZRGBA point cloud
	PointCloud cloud;

	/// timestamp
	float_type timestamp;

	/// readCounter
	int readId;

	/// Default constructor
	inline SensorFrame() :
			timestamp(0) {
	}
};

/// 2D image feature
class ImageFeature {
public:
	/// set of features
	typedef std::vector<ImageFeature> Seq;

	/// 2D feature location
    float_type u;
    float_type v;

	/// Image patch
	cv::Mat patch;

	/// Depth
	float_type depth;

	/// Default constructor
	inline ImageFeature() :
			u(0), v(0) {
	}

    /// Overloaded constructor
    inline ImageFeature(float_type _u, float_type _v, float_type _depth) :
            u(_u), v(_v), depth(_depth) {
    }
};

class ExtendedDescriptor {
public:
	/// set of descriptors
	typedef std::vector<ExtendedDescriptor> Seq;

	/// OpenCV descriptor
	cv::Mat descriptor;

	/// id of the the camera pose (vertex in the graph)
	unsigned int poseId;

    /// feature location on the rgb image
    float_type u, v;

	/// Constructor TODO: is it needed?
    ExtendedDescriptor() {};

	/// Constructor TODO: is it needed?
	ExtendedDescriptor(unsigned int _poseId, cv::Mat _descriptor) :
            poseId(_poseId), descriptor(_descriptor) {};

	/// Constructor
	ExtendedDescriptor(unsigned int _poseId, float_type _u, float_type _v,
			cv::Mat _descriptor) :
			poseId(_poseId), u(_u), v(_v), descriptor(_descriptor) {};
};

class RGBDFeature {
public:
	/// Position of the feature
	Vec3 position;

	/// feature location on the rgb image
	float_type u;

	/// feature location on the rgb image
	float_type v;

    /// normal vector
    Vec3 normal;

    /// RGB gradient vector
    Vec3 RGBgradient;

	/// set of descriptors
	std::vector<ExtendedDescriptor> descriptors;

	RGBDFeature(void) {
    }

	RGBDFeature(const Vec3 _position, float_type _u, float_type _v,
			const std::vector<ExtendedDescriptor> _descriptors) :
			position(_position), descriptors(_descriptors), u(_u), v(_v) {

    }
};

class MapFeature: public RGBDFeature {
public:
	/// id of the feature
	unsigned int id;

	/// poses ids
	std::vector<unsigned int> posesIds;

    /// image coordinates: std::map<index_of_the_frame,<u,v,d>>
    std::map<unsigned int, ImageFeature> imageCoordinates;

    /// mapFeature life number
    unsigned int lifeValue;

	/// Constructor
	MapFeature() {
    };

	/// Constructor
	MapFeature(unsigned int _id, float_type u, float_type  v,
			const Vec3 _position, std::vector<unsigned int> _posesIds,
            std::vector<ExtendedDescriptor> _descriptors,
            std::map<unsigned int, ImageFeature> _imageCoordinates) :
			RGBDFeature(_position, u, v, _descriptors), id(_id), posesIds(
                    _posesIds), imageCoordinates(_imageCoordinates) {
		lifeValue = 8;
    };

	/// Constructor
	MapFeature(unsigned int _id) :
			id(_id) {
    };
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
	inline KeyPoint() {
	}

	/// Overloaded constructor
	inline KeyPoint(const Vec3 _depth) :
			depthFeature(_depth) {
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
		/// Edge 3D -- feature position
		EDGE_3D,
		/// Edge SE(3) -- robot pose
		EDGE_SE3,
		/// Edge SE(2) -- robot x,y,theta
		EDGE_SE2,
		/// EDGE 3D REPROJECTION -- feature position optimized with reproj error
		EDGE_3D_REPROJ
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
	inline Edge() {
	}

	/// Overloaded constructor
	inline Edge(Type _type) :
			type(_type) {
	}

	/// Overloaded constructor
	inline Edge(Type _type, uint_fast32_t _fromVertexId,
			uint_fast32_t _toVertexId) :
			type(_type), fromVertexId(_fromVertexId), toVertexId(_toVertexId) {
	}

	virtual ~Edge() {}
};

/// 3D (x,y,z) Edge of a graph
class Edge3D: public Edge {
public:
	/// set of Edges3D
	typedef std::vector<Edge3D> Seq;

	/// translation between nodes
	Vec3 trans;

	/// Information matrix
	Mat33 info;

	/// Default constructor
	inline Edge3D() :
			Edge(EDGE_3D) {
	}

	/// Overloaded constructor
	inline Edge3D(const Vec3 _trans, const Mat33 _info,
			uint_fast32_t _fromVertexId, uint_fast32_t _toVertexId) :
			Edge(EDGE_3D, _fromVertexId, _toVertexId), trans(_trans), info(
					_info) {
	}
};

/// 3D_REPROJ (u,v) Edge of a graph
class Edge3DReproj: public Edge {
public:
	/// set of Edges3D
	typedef std::vector<Edge3D> Seq;

	/// Image coordinates
	double u, v;

	/// Information matrix
	Eigen::Matrix<float_type, 2, 2> info;

	/// Default constructor
	inline Edge3DReproj() :
			Edge(EDGE_3D_REPROJ) {
	}

	/// Overloaded constructor
	inline Edge3DReproj(double _u, double _v, const Eigen::Matrix<float_type, 2, 2> _info,
			uint_fast32_t _fromVertexId, uint_fast32_t _toVertexId) :
			Edge(EDGE_3D_REPROJ, _fromVertexId, _toVertexId), info(
					_info), u(_u), v(_v) {
	}
};

/// SE2 (x,y,theta) Edge of a graph
class EdgeSE2: public Edge {
public:
	/// set of Edges3D
	typedef std::vector<EdgeSE2> Seq;

	/// translation between nodes
	Eigen::Vector2d trans;

	/// rotation between nodes
	float_type theta;

	/// Information matrix
	Mat33 info;

	/// Default constructor
	inline EdgeSE2() :
			Edge(EDGE_SE2) {
	}

	/// Overloaded constructor
	inline EdgeSE2(Eigen::Vector2d _trans, float_type _theta, Mat33 _info,
			uint_fast32_t _fromVertexId, uint_fast32_t _toVertexId) :
			Edge(EDGE_SE2, _fromVertexId, _toVertexId), theta(_theta), trans(
					_trans), info(_info) {
	}
};

/// SE(3) Edge of a graph (x,y,z + quat(4))
class EdgeSE3: public Edge {
public:
	/// set of EdgesSE3
	typedef std::vector<EdgeSE3> Seq;

	/// translation and rotation between nodes
    Mat34 trans;

	/// Information matrix
	Mat66 info;

	/// Default constructor
	inline EdgeSE3() :
			Edge(EDGE_SE3) {
	}

	/// Overloaded constructor
    inline EdgeSE3(Mat34 _trans, Mat66 _info, uint_fast32_t _fromVertexId,
			uint_fast32_t _toVertexId) :
			trans(_trans), Edge(EDGE_SE3, _fromVertexId, _toVertexId), info(
					_info) {
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
		VERTEXSE3,
		/// Vertex SE(2) -- x,y,theta
		VERTEXSE2
	};

	/// Vertex type
	Type type;

	/// Vertex / node id
	uint_fast32_t vertexId;

	/// timestamp
	float_type timestamp;

	/// Default constructor
	inline Vertex() {
	}

	/// Overloaded constructor
	inline Vertex(Type _type, uint_fast32_t _vertexId) :
			type(_type), vertexId(_vertexId), timestamp(0.0) {
	}

	/// Overloaded constructor
	inline Vertex(Type _type, uint_fast32_t _vertexId, float_type _timestamp) :
			type(_type), vertexId(_vertexId), timestamp(_timestamp) {
	}
};

class Vertex3D: public Vertex {
public:
	/// Set of Vertices
	typedef std::vector<Vertex3D> Seq;

	/// Vertex / node
	KeyPoint keypoint;

	/// Default constructor
	inline Vertex3D(void) :
			Vertex(VERTEX3D, 0) {
	}

	/// Overloaded constructor
	inline Vertex3D(uint_fast32_t _vertexId, const Vec3 _pos) :
			Vertex(VERTEX3D, _vertexId), keypoint(_pos) {
	}
};

class VertexSE2: public Vertex {
public:
	/// Set of Vertices
	typedef std::vector<VertexSE2> Seq;

	/// position
	Eigen::Vector2d pos;

	/// orientation
	float_type theta;

	/// Default constructor
	inline VertexSE2(void) :
			Vertex(VERTEXSE2, 0) {
	}

	/// Overloaded constructor
	inline VertexSE2(uint_fast32_t _vertexId, Eigen::Vector2d _pos,
			float_type _rot) :
			Vertex(VERTEXSE2, _vertexId), pos(_pos), theta(_rot) {
	}
};

class VertexSE3: public Vertex {
public:
	/// Set of Vertices
	typedef std::vector<VertexSE3> Seq;

	/// Vertex / node
    Mat34 pose;

	/// Point cloud
	PointCloud cloud;

    /// Set of keypoints
    //KeyPoint::Seq keypoints;

    std::set<int> featuresIds;

	/// Default constructor
	inline VertexSE3(void) :
			Vertex(VERTEXSE3, 0) {
	}

	/// Overloaded constructor
    inline VertexSE3(uint_fast32_t _vertexId, const Mat34 _pose) :
            Vertex(VERTEXSE3, _vertexId), pose(_pose) {
	}

	/// Overloaded constructor
    inline VertexSE3(uint_fast32_t _vertexId, const Mat34 _pose, float_type timestamp) :
            Vertex(VERTEXSE3, _vertexId, timestamp), pose(_pose) {
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

class MapModifier{
public:
    /// Features to update
    std::map<int,MapFeature> features2update;

    /// Features to remove
    std::vector<int> removeIds;

    /// Features to update
    std::map<int,MapFeature> features2add;

    ///poses to update
    std::vector<VertexSE3> poses2update;

    ///poses to add
    std::vector<VertexSE3> poses2add;

    /// Update features?
    inline bool updateFeatures() const { return (features2update.size()>0) ?  true : false;};
    /// Remove feaures?
    inline bool removeFeatures() const { return (removeIds.size()>0) ?  true : false;};
    /// add features?
    inline bool addFeatures() const { return (features2add.size()>0) ?  true : false;};
    /// add poses?
    inline bool addPoses() const { return (poses2add.size()>0) ?  true : false;};
    /// Update features?
    inline bool updatePoses() const { return (poses2update.size()>0) ?  true : false;};

    /// mutex to lock access
    std::recursive_mutex mtxBuffer;
};

}

#endif // PUTSLAM_DEFS_H_INCLUDED
