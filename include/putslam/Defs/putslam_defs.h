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
#include "opencvCore.h"
//#include "opencv2/core/core.hpp"
#include "Defs/eigen3.h"
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <mutex>
#include <set>
#include <iostream>
#include <map>

/// putslam name space
namespace putslam {

/// 3 element vector class
typedef Eigen::Translation<double, 3> Vec3;

/// Matrix representation of SO(3) group of rotations
typedef Eigen::Matrix<double, 3, 3> Mat33;

/// Information Matrix of SE(3) transformation
typedef Eigen::Matrix<double, 6, 6> Mat66;

/// Quaternion representation of SO(3) group of rotations
typedef Eigen::Quaternion<double> Quaternion;

/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

/// Homogenous representation of SE(3) rigid body transformations with scale
class Mat44: public Mat34 {
public:
	/// rotation
    double scale;

	/// Default constructor sets the default configuration
	inline Mat44() :
			scale(1) {
	}
};

/// 3D point representation
class Point3D{
public:
    double x,y,z,r,g,b,a;
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
    double timestamp;

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
    double u;
    double v;

	/// Image patch
    //cv::Mat patch;

	/// Depth
    double depth;

	/// Default constructor
	inline ImageFeature() :
			u(0), v(0) {
	}

    /// Overloaded constructor
    inline ImageFeature(double _u, double _v, double _depth) :
            u(_u), v(_v), depth(_depth) {
    }
};

class ExtendedDescriptor {
public:
	/// set of descriptors
	//typedef std::vector<ExtendedDescriptor> Seq;

	/// id of the the camera pose (vertex in the graph)
	//unsigned int poseId;

    /// feature location on the poseId-th rgb image
    cv::Point2f point2D, point2DUndist;

    /// 3D Position of the feature in the coordinate system of poseId-th image
    Vec3 point3D;

	/// OpenCV descriptor for poseId-th image
	cv::Mat descriptor;

    /// octave (pyramid layer) at which it was detected
    int octave;

    /// distance to the feature at the time it was described
    double detDist;

	/// Constructor
	ExtendedDescriptor() {
	}
	ExtendedDescriptor(cv::Point2f _point2D, cv::Point2f _point2DUndist,
			Vec3 _point3D, cv::Mat _descriptor, int _octave, double _detDist) :
			point2D(_point2D), point2DUndist(_point2DUndist), point3D(_point3D), descriptor(
					_descriptor), octave(_octave), detDist(_detDist) {
    }
};

class RGBDFeature {
public:
	/// Position of the feature (temporal or in global CS)
	Vec3 position;

	/// set of descriptors, std::map<poseId, computedDescriptor>
	std::map<unsigned int, ExtendedDescriptor> descriptors;
//	std::vector<ExtendedDescriptor> descriptors;

	/// feature location on the rgb image
    double u;

	/// feature location on the rgb image
    double v;

	/// normal vector
	Vec3 normal;

	/// RGB gradient vector
	Vec3 RGBgradient;

	RGBDFeature(void) {
    }

    RGBDFeature(const Vec3 _position, double _u, double _v,
			const std::map<unsigned int, ExtendedDescriptor> _descriptors) :
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
    }

	/// Constructor
    MapFeature(unsigned int _id, double u, double  v,
			const Vec3 _position, std::vector<unsigned int> _posesIds,
			std::map<unsigned int, ExtendedDescriptor> _descriptors,
            std::map<unsigned int, ImageFeature> _imageCoordinates) :
			RGBDFeature(_position, u, v, _descriptors), id(_id), posesIds(
                    _posesIds), imageCoordinates(_imageCoordinates) {
		lifeValue = 10;
    }

	/// Constructor
	MapFeature(unsigned int _id) :
			id(_id) {
    }
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
    unsigned int keypointId;

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
    unsigned int toVertexId;

	/// Node connected by the edge
    unsigned int fromVertexId;

	/// Default constructor
	inline Edge() {
	}

	/// Overloaded constructor
	inline Edge(Type _type) :
			type(_type) {
	}

	/// Overloaded constructor
    inline Edge(Type _type, unsigned int _fromVertexId,
            unsigned int _toVertexId) :
			type(_type), toVertexId(_toVertexId), fromVertexId(_fromVertexId) {
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
            unsigned int _fromVertexId, unsigned int _toVertexId) :
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
    Eigen::Matrix<double, 2, 2> info;

	/// Default constructor
	inline Edge3DReproj() :
			Edge(EDGE_3D_REPROJ) {
	}

	/// Overloaded constructor
    inline Edge3DReproj(double _u, double _v, const Eigen::Matrix<double, 2, 2> _info,
            unsigned int _fromVertexId, unsigned int _toVertexId) :
			Edge(EDGE_3D_REPROJ, _fromVertexId, _toVertexId), u(_u), v(_v), info(
					_info) {
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
    double theta;

	/// Information matrix
	Mat33 info;

	/// Default constructor
	inline EdgeSE2() :
			Edge(EDGE_SE2) {
	}

	/// Overloaded constructor
    inline EdgeSE2(Eigen::Vector2d _trans, double _theta, Mat33 _info,
            unsigned int _fromVertexId, unsigned int _toVertexId) :
			Edge(EDGE_SE2, _fromVertexId, _toVertexId), trans(_trans), theta(
					_theta), info(_info) {
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
    inline EdgeSE3(Mat34 _trans, Mat66 _info, unsigned int _fromVertexId,
            unsigned int _toVertexId) :
			Edge(EDGE_SE3, _fromVertexId, _toVertexId), trans(_trans), info(
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
    unsigned int vertexId;

	/// timestamp
    double timestamp;

	/// Default constructor
	inline Vertex() {
	}

	/// Overloaded constructor
    inline Vertex(Type _type, unsigned int _vertexId) :
			type(_type), vertexId(_vertexId), timestamp(0.0) {
	}

	/// Overloaded constructor
    inline Vertex(Type _type, unsigned int _vertexId, double _timestamp) :
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
    inline Vertex3D(unsigned int _vertexId, const Vec3 _pos) :
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
    double theta;

	/// Default constructor
	inline VertexSE2(void) :
			Vertex(VERTEXSE2, 0) {
	}

	/// Overloaded constructor
    inline VertexSE2(unsigned int _vertexId, Eigen::Vector2d _pos,
            double _rot) :
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

    /// is keyframe
    bool isKeyframe;

    /// Set of keypoints
    //KeyPoint::Seq keypoints;

    std::set<int> featuresIds;

	/// Default constructor
	inline VertexSE3(void) :
            Vertex(VERTEXSE3, 0), isKeyframe(false) {
	}

	/// Overloaded constructor
    inline VertexSE3(unsigned int _vertexId, const Mat34 _pose) :
            Vertex(VERTEXSE3, _vertexId), pose(_pose), isKeyframe(false) {
	}

	/// Overloaded constructor
    inline VertexSE3(unsigned int _vertexId, const Mat34 _pose, double timestamp) :
            Vertex(VERTEXSE3, _vertexId, timestamp), pose(_pose), isKeyframe(false) {
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
    /// Features to update <featureId, feature>
    std::map<int,MapFeature> features2update;

    /// Features measurements update <featureId, <poseId, feature>>
    std::map<int,std::pair<int, MapFeature>> measurements2update;

    /// Features to remove
    std::vector<int> removeIds;

    /// Features to update
    std::map<int,MapFeature> features2add;

    ///poses to update
    std::vector<VertexSE3> poses2update;

    ///poses to add
    std::vector<VertexSE3> poses2add;

    /// Update features?
    inline bool updateFeatures() const { return (features2update.size()>0) ?  true : false;}
    /// Update features?
    inline bool updateMeasurements() const { return (measurements2update.size()>0) ?  true : false;}
    /// Remove feaures?
    inline bool removeFeatures() const { return (removeIds.size()>0) ?  true : false;}
    /// add features?
    inline bool addFeatures() const { return (features2add.size()>0) ?  true : false;}
    /// add poses?
    inline bool addPoses() const { return (poses2add.size()>0) ?  true : false;}
    /// Update features?
    inline bool updatePoses() const { return (poses2update.size()>0) ?  true : false;}

    /// mutex to lock access
    std::recursive_mutex mtxBuffer;
};

}

#endif // PUTSLAM_DEFS_H_INCLUDED
