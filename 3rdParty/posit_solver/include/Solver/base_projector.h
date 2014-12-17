#pragma once
#include "cloud.h"

namespace PSolver {
  using namespace std;

  /**
     This class encapsulates algorithm and parameters to project a 3D model onto an image,
     through a general model. It is then specialized in pinhole or projectove
   */
  class BaseProjector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! ctor, initializes the inner fields
    BaseProjector();

    /** does the projection, taking into account the normals and the occlusions.
	@param zbuffer: a float matrix used to handle occlusions, filled by the method
	@param indices: an int image, filled by the algorithm. Each cell contains the index of the point in model, that projects onto a pixel
	@param T: the transform world_to_camera applied before the projection
	@param model: input
     */
    virtual void project(FloatImage& zbuffer, 
		 IndexImage& indices,
		 const Eigen::Isometry3f& T,
		 const Cloud& model) const = 0;

    /** does the projection, taking into account the normals and the occlusions.
	@param zbuffer: a float matrix used to handle occlusions, filled by the method
	@param indices: an int image, filled by the algorithm. Each cell contains the index of the point in model, that projects onto a pixel
	@param T: the transform world_to_camera applied before the projection
	@param src_zbuffer: the source z_buffer
	@param src_indices: the source indices
     */
    virtual void project(FloatImage& zbuffer, 
			 IndexImage& indices,
			 const Eigen::Isometry3f& T,
			 const FloatImage& src_zbuffer, 
			 const IntImage& src_indices,
			 float src_scale = 1, int subsample = 1) const = 0;


	//! size of the image used for projection
    inline int imageRows() const {return _image_rows;}
    inline int imageCols() const {return _image_cols;}
    virtual void setImageSize(int r, int c);
    
    //! minimum distance at which points are prjoected
    inline float minDistance() const {return _min_distance; }
    inline void setMinDistance(float d) {_min_distance = d;}
    
    //! maximum distance
    inline float maxDistance() const {return _max_distance; }
    inline void setMaxDistance(float d) {_max_distance = d;}
    
     //! maximum distance
    inline void setIncidenceAngle(float ia) {_incidence_angle = ia; _incidence_angle_cos = cos(ia);}
    inline float incidenceAngle() const {return _incidence_angle;}

    //! applies a scaling factor to the camera, not the image  
    //! computes the center so that it is the center of the image
    //! @param s: the scale (s=0.5 corresponds to half the image)
    virtual void scaleCamera(float s) = 0;
    
    //! pushes the state (projection parameters and image size);
    virtual void pushState() = 0;

    //! pops the pushed state
    virtual void popState() = 0;

    inline const Eigen::Isometry3f& offset() const { return _offset; }
    inline  void setOffset(const Eigen::Isometry3f& o)  {  _offset=o; _inverse_offset=o.inverse();}

  protected:
    Eigen::Isometry3f _offset, _inverse_offset;
    float _incidence_angle, _incidence_angle_cos;
    float _image_rows, _image_cols;
    float _min_distance, _max_distance;
  };

}
