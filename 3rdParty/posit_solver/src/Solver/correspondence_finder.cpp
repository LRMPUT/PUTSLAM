#include "../../include/Solver/correspondence_finder.h"
#include "../../include/Solver/solver.h"

namespace PSolver {
  
  CorrespondenceFinder::CorrespondenceFinder(Solver* s) {
    _solver = s;
    _normal_angle = .5;
    _points_distance = 0.2;
  }

  CorrespondenceFinder::~CorrespondenceFinder() {}

}
