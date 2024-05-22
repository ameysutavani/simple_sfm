#ifndef SIMPLE_SFM_CERES_BACK_END_H
#define SIMPLE_SFM_CERES_BACK_END_H

#include <simple_sfm/types.h>

namespace simple_sfm {
namespace ceres_back_end {

/// @brief Optimize the given SFM problem
/// @param [in] sfm_problem The SFM problem to optimize. The size of the
/// cameras, points and observations in the problem should be greater than 0.
/// @param [out] optimized_sfm_variables The optimized SFM variables (containing
/// the optimized camera parameters and the point positions) to be returned. The
/// size of the cameras and points should match the size of the cameras and
/// points in the SFM problem respectively.
/// @param [in] options
/// @return OptimizationResult
/// @note sfm_problem and optimized_sfm_variables should remain valid and
/// unmodified until this function returns.
types::OptimizationResult
optimize(const types::SfmProblem<>& sfm_problem,
         types::SfmVariables<>& optimized_sfm_variables,
         const types::Options& = {});

} // namespace ceres_back_end
} // namespace simple_sfm

#endif // SIMPLE_SFM_CERES_BACK_END_H
