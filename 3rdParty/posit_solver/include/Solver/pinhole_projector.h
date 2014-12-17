#pragma once
#include "cloud.h"
#include "base_projector.h"
#include <stack>

namespace PSolver {
  using namespace std;

  /**
     This class encapsulates algorithm and parameters to project a 3D model onto an image,
     through a pinhole model.
   */
  class PinholeProjector : public BaseProjector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! ctor, initializes the inner fields
    PinholeProjector();

    /** does the projection, taking into account the normals and the occlusions.
	@param zbuffer: a float matrix used to handle occlusions, filled by the method
	@param indices: an int image, filled by the algorithm. Each cell contains the index of the point in model, that projects onto a pixel
	@param T: the transform world_to_camera applied before the projection
	@param model: input
     */
    virtual void project(FloatImage& zbuffer, 
		 IndexImage& indices,
		 const Eigen::Isometry3f& T,
		 const Cloud& model) const;

    // projection from z buffer to z buffer, should be faster
    virtual void project(FloatImage& zbuffer, 
			 IndexImage& indices,
			 const Eigen::Isometry3f& T,
			 const FloatImage& src_zbuffer, 
			 const IntImage& src_indices,
			 float src_scale=1,
			 int subsample = 1) const;


    //! overridden from base class
    //! applies a scaling factor to the camera, not the image  
    //! computes the center so that it is the center of the image
    //! @param s: the scale (s=0.5 corresponds to half the image)
    virtual void scaleCamera(float s);


    //! overridden from base class
    virtual void pushState();

    //! overridden from base class
    virtual void popState();


    //! the camera matrix
    inline const Eigen::Matrix3f& K() const { return _K;}
    inline void setK(const Eigen::Matrix3f& K_) {  _K = K_;}

  protected:
   
    Eigen::Matrix3f _K;



    //! method used in a multithreaded run, that operates on a subset of the indices
    void _project(FloatImage& zbuffer, 
		  IndexImage& indices,
		  const Eigen::Isometry3f& T,
		  const Cloud& model, int imin, int imax, bool allocate_images = true) const;


    
    int _num_threads;
    mutable std::vector<FloatImage> _zbuffers;
    mutable std::vector<IntImage> _indicess;

    // for push and pop
    struct State {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      State(const Eigen::Matrix3f& K, int r, int c);
      Eigen::Matrix3f K;
      int rows;
      int cols;
    };

    std::stack<State, std::deque< State, Eigen::aligned_allocator<State> > > _states;
  private:
    mutable size_t _max_image_size;
    mutable std::vector< std::vector<int> > _indicess_buf;
    mutable std::vector <std::vector <float> >_zbuffers_buf;

    void allocateBuffers() const;
  };

}
