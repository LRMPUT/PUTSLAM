#ifndef G2OHEADERS_HPP_INCLUDED
#define G2OHEADERS_HPP_INCLUDED
#ifdef __GNUC__
// Avoid tons of warnings with g2o code
#pragma GCC system_header
#endif
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/g2o_core_api.h"

#include "g2o/core/factory.h"
//#include "g2o/stuff/command_args.h"
#include <g2o/types/slam3d/isometry3d_mappings.h>
//#include "g2o/core/eigen_types.h"
//#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_reprojectionError.h"
#endif
