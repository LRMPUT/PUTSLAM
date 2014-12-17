#include "../../include/Solver/pinhole_projector.h"
#include <stdexcept>


#ifdef _GO_PARALLEL_
#include <omp.h>
#endif

namespace PSolver {
  using namespace std;


  void PinholeProjector::allocateBuffers() const {
    size_t current_size = _image_cols * _image_rows;
    if (current_size>_max_image_size){
      cerr << "realloc: " << _max_image_size << " ->" << current_size << endl;
      _max_image_size = current_size;
      for (size_t i = 0; i<_num_threads; i++){
	_zbuffers_buf[i].reserve(_max_image_size);
	_indicess_buf[i].reserve(_max_image_size);
      }
    }
    for (size_t i = 0; i<_num_threads; i++){
      _zbuffers_buf[i].resize(current_size);
      _indicess_buf[i].resize(current_size);
      _zbuffers[i]=FloatImage(_image_rows, _image_cols, &(_zbuffers_buf[i][0]));
      _indicess[i]=IntImage(_image_rows, _image_cols, &(_indicess_buf[i][0]));
    }
  }


  PinholeProjector::PinholeProjector(){
    _K <<  
      525, 0, 319.5, 
      0, 525, 239.5, 
      0, 0, 1;
    _image_cols=640;
    _image_rows=480;
    _min_distance = 0.4;
    _max_distance = 3;
    setIncidenceAngle(0.5* M_PI);

    _max_image_size = 0;
#ifdef _GO_PARALLEL_
    _num_threads = omp_get_max_threads();
    _zbuffers.resize(_num_threads);
    _indicess.resize(_num_threads);
    _zbuffers_buf.resize(_num_threads);
    _indicess_buf.resize(_num_threads);
    cerr << "parallel projector initialized with " << _num_threads << " threads" << endl;
#else //_GO_PARALLEL_
    _num_threads = 1;
    _max_image_size = 0;
    cerr << "sequential projector initialized" << endl;
#endif //_GO_PARALLEL_

  }


#ifndef _GO_PARALLEL_
  void PinholeProjector::project(FloatImage& zbuffer, IndexImage& indices,
			    const Eigen::Isometry3f& T,
			    const Cloud& model) const {
    _project(zbuffer, indices, _inverse_offset*T, model, 0, model.size());
    return;
  }

#else //_GO_PARALLEL_

  void PinholeProjector::project(FloatImage& zbuffer, IndexImage& indices,
			    const Eigen::Isometry3f& T,
			    const Cloud& model) const {


    int iterationsPerThread =  model.size() / _num_threads;
    allocateBuffers();
#pragma omp parallel
    {
      int threadId = omp_get_thread_num();
      int imin = iterationsPerThread * threadId;
      int imax = imin + iterationsPerThread;
      if((size_t)imax > model.size())
	imax = model.size();
      _project(_zbuffers.at(threadId), _indicess[threadId], _inverse_offset*T, model, imin, imax, false);
    }
    
    // cerr << "reduce" << endl;
    zbuffer.create(_image_rows, _image_cols);
    indices.create(_image_rows, _image_cols);

# pragma omp parallel for
    for (int r=0; r<zbuffer.rows; r++) {
      float *thread_z_ptr[_num_threads];
      int   *thread_idx_ptr[_num_threads];
      // prepare the row indices
      float* z_ptr = zbuffer.ptr<float>(r);
      int* idx_ptr = indices.ptr<int>(r);
      for (int k=0; k<_num_threads; k++){
	thread_z_ptr[k] = _zbuffers[k].ptr<float>(r);
	thread_idx_ptr[k] = _indicess[k].ptr<int>(r);
      }

      for (int c=0; c<zbuffer.cols; c++) {
	float& depth = *z_ptr;
	int& idx = *idx_ptr;
	idx = -1;
	depth = 1e9f;
	for (int k=0; k<_num_threads; k++) {
	  float cdepth = *thread_z_ptr[k];
	  int cidx = *thread_idx_ptr[k];
	  if (cdepth<depth) {
	    depth = cdepth;
	    idx = cidx;
	  }
	  thread_z_ptr[k]++;
	  thread_idx_ptr[k]++;
	}
	z_ptr ++;
	idx_ptr ++;
      }
    }
  }
#endif // _GO_PARALLEL_


  void PinholeProjector::project(FloatImage& zbuffer, 
				 IndexImage& indices,
				 const Eigen::Isometry3f& T,
				 const FloatImage& src_zbuffer, 
				 const IntImage& src_indices, 
				 float src_scale,
				 int subsample) const {

    zbuffer.create(_image_rows, _image_cols);
    indices.create(_image_rows, _image_cols);
    zbuffer = 1e9f;
    indices = -1;
    
    Eigen::Matrix3f KR = _K * _inverse_offset*T.linear();
    Eigen::Vector3f Kt = _K * _inverse_offset*T.translation();
    Eigen::Matrix3f scaled_K = _K;
    scaled_K.block<2,3>(0,0)*=src_scale;

    Eigen::Matrix3f iK = scaled_K.inverse();

    for(int r = 0; r< src_zbuffer.rows; r+=subsample){
      const float *src_z_ptr = src_zbuffer.ptr<float>(r);
      const int   *src_idx_ptr = src_indices.ptr<int>(r);
      for (int c = 0; c< src_zbuffer.cols; c+=subsample) {
	float src_z = *src_z_ptr;
	int src_idx = *src_idx_ptr;
	src_z_ptr +=subsample;
	src_idx_ptr +=subsample;
	if (src_idx < 0)
	  continue;
	
	Eigen::Vector3f p=iK*Eigen::Vector3f(c*src_z, r*src_z, src_z);
	Eigen::Vector3f pp = KR*p + Kt;
	float d = pp.z();
	if (d>_max_distance)
	  continue;
	int dest_c = pp.x()/d;
	int dest_r = pp.y()/d;

	if (dest_r<0 || dest_r>=_image_rows ||
	    dest_c<0 || dest_c>=_image_cols)
	  continue;

	float& pz=zbuffer.at<float>(dest_r,dest_c);
	if (pz>d){
	  pz=d;
	  indices.at<int>(dest_r,dest_c)=src_idx;
	}
      }
    }
  }


  void PinholeProjector::_project(FloatImage& zbuffer, IndexImage& indices,
				  const Eigen::Isometry3f& T,
				  const Cloud& model, int imin, int imax, 
				  bool allocate_images) const {
    if (allocate_images ) {
      zbuffer.create(_image_rows, _image_cols);
      indices.create(_image_rows, _image_cols);
    }
    zbuffer = 1e9f;
    indices = -1;
    for (size_t i = imin; i<(size_t)imax; i++){
      Eigen::Vector3f p=T*model[i].point();
      if (p.z()<_min_distance || p.z()>_max_distance) 
	continue;
      int idx = i;
      Eigen::Vector3f mn=T.linear()*model[i].normal();
      if (p.dot(mn)>-_incidence_angle_cos) {
	idx = -1;
      }
      Eigen::Vector3f pp=_K*p;
      pp *= (1./p.z());
      int c = pp.x();
      int r = pp.y();
      if (r<0 || r>=_image_rows ||
	  c<0 || c>=_image_cols)
	continue;

      float& pz=zbuffer.at<float>(r,c);
      {
	if (pz>p.z()){
	  pz=p.z();
	  indices.at<int>(r,c)=idx;
	}
      }
    }
  }


  void PinholeProjector::scaleCamera(float s) {
    _K.block<2,3>(0,0)*=s;
    _K(0,2)=.5*_image_rows;
    _K(1,2)=.5*_image_cols;
  }


  PinholeProjector::State::State(const Eigen::Matrix3f& K, int r, int c) {
    this->K=K;
    rows = r;
    cols = c;
  }

  void PinholeProjector::pushState() {
    State s(_K, _image_rows, _image_cols);
    _states.push(s);
  }

  void PinholeProjector::popState() {
    State s = _states.top();
    _K = s.K;
    _image_rows = s.rows;
    _image_cols = s.cols;
    _states.pop();
  }
}
