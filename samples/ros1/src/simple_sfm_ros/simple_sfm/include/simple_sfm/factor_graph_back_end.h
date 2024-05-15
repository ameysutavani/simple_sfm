#ifndef SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H
#define SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H

#include <simple_sfm/types.h>
// #include <memory> // Uncomment later, if desired

namespace simple_sfm {
namespace factor_graph_back_end {

/// @brief Options (tuning) for the optimization
struct Options
{
  /// @brief Noise standard deviation for the prior factors on the first camera
  /// pose and the first point
  double prior_factor_sigma{0.1};
  /// @brief Noise standard deviation for the reprojection factors for each
  /// observation
  double projection_factor_sigma{1.0};
};

/// @brief Struct to hold the optimization result and additional information
struct OptimizationResult
{
  /// @brief Flag to indicate if the optimization converged
  bool converged{false};
  /// @brief Final (total) reprojection error after optimization
  double final_error{0.0};
};

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

// TODO: Add a math note about the problem formulation and the optimization.

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
OptimizationResult optimize(const types::SfmProblem<>& sfm_problem,
                            types::SfmVariables<>& optimized_sfm_variables,
                            const Options& options = Options{});

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
