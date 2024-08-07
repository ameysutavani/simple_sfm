#ifndef SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H
#define SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H

#include <simple_sfm/types.h>
// #include <memory> // Uncomment later, if desired

namespace simple_sfm {
namespace factor_graph_back_end {

// DESIGN-NOTE: Making the following design simplifications for the current
// use-case for the optimize function:
// 1. The function should try and do as little dynamic memory allocation as
// possible. This can make a signifant impact if the size of the variables is
// large for a given problem. Hence, the optimized_sfm_variables is passed by
// reference. The function can be easily wrapped in a helper function to
// allocate the memory for the optimized_sfm_variables and then call the
// optimize function.
// 2. Another variant of the wrapper can be one which uses smart pointers to
// pass the sfm_problem and optimized_sfm_variables, if a delayed destruction of
// the memory is required. But, for the current, use-case, keeping it simple and
// assuming that the caller is satisfying the function's preconditions.

// MATH-NOTE: For each observation, we set up following optimization problem:

// clang-format off
// (3D Point in the world) -> (3D Point in the camera frame) -> (2D pixel in the image) -> (Difference with the observed pixel i.e. reprojection error) <- [observed pixel]
//            ^                          ^                                 ^                                       ^
//            |                          |                                 |                                       |
//    {Point variables}            {Camera pose}                  {Camera intrinsics}                       Cost to minimize
// clang-format on
// where,
// {} - Represents the variables to optimize
// [] - Represents the observed (fixed) data
// GTSAM SFM factors automatically setup the above chain of factors through
// which the error can be backpropagated to optimize the variables.
// This is possible due to the analytical Jacobians created for each factor
// inside the GTSAM library.

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
         const types::Options& options = {});

// DESIGN-NOTE: Uncomment later, if desired:
// std::pair<types::SfmVariables<>, OptimizationResult>
// createOptimizedVariables(const types::SfmProblem<>& sfm_problem,
//                          const Options& options = Options{})
// {
//   auto optimized_sfm_variables = sfm_problem.variables;
//   auto optimization_result =
//       optimize(sfm_problem, optimized_sfm_variables, options);
//   return {optimized_sfm_variables, optimization_result};
// }

// DESIGN-NOTE: (shared_ptr version) Uncomment later, if desired:
// OptimizationResult
// optimize(const std::shared_ptr<types::SfmProblem<>>& sfm_problem,
//          std::shared_ptr<types::SfmVariables<>>& optimized_sfm_variables,
//          const Options& options = Options{})
// {
//   auto optimization_result =
//       optimize(*sfm_problem, *optimized_sfm_variables, options);
//   return optimization_result;
// }

} // namespace factor_graph_back_end
} // namespace simple_sfm

#endif // SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H
