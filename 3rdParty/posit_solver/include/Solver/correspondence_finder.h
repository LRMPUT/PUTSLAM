#pragma once
#include "defs.h"
#include <utility>


namespace PSolver {
  class Solver;

  //! abstract correspondence finder
  class CorrespondenceFinder {
  public:
    //! this is the single correspondence
    //! first-> reference point index
    //! second-> current point index
    typedef std::pair <int,int> Correspondence;
    typedef std::vector<Correspondence> CorrespondenceVector;
    

    CorrespondenceFinder(Solver* s);

    //~ return the solver pbject linked to this
    inline Solver& solver() {return *_solver;}

    //! angle between normals to consider as correspondences
    inline float normalAngle() const {return _normal_angle;}
    inline void setNormalAngle(float a)  {_normal_angle = a;}

    //! distance between corresponding points
    inline float pointsDistance() const {return _points_distance;}
    inline void setPointsDistance(float d)  {_points_distance = d;}

    //! call this once before staring an alignment
    virtual void init() = 0;

    //! does the computation (to be overridden in base classes. It puts the resuld in  _correspondences)
    virtual void compute() = 0;

    virtual ~CorrespondenceFinder();
    //! returns the corresoidences
    inline const CorrespondenceVector& correspondences() const { return _correspondences; }
  protected:
    CorrespondenceVector _correspondences;
    Solver* _solver;
    float _normal_angle, _points_distance;

  };
}
