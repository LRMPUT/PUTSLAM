#include "../../include/Solver/base_projector.h"
#include <stdexcept>
#include <omp.h>

namespace PSolver {
  using namespace std;

  BaseProjector::BaseProjector(){
    _image_cols=640;
    _image_rows=480;
    _min_distance = 0.4;
    _max_distance = 3;
    _offset.setIdentity();
    _inverse_offset.setIdentity();
    setIncidenceAngle(0.4 * M_PI);
  }

  void BaseProjector::setImageSize(int r, int c) {
    _image_rows = r;
    _image_cols = c;
  }
}
