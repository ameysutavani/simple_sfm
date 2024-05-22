#include <functional>
#include <iostream>

#include <simple_sfm/ceres_back_end.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>

namespace {

using simple_sfm::types::Pixel;

struct ReprojectionFactor
{
  ReprojectionFactor(const Pixel<>& observed_pixel)
      : observed_pixel_{observed_pixel}
  {
  }

  template <typename T>
  bool operator()(const T* const camera_axis_angle,
                  const T* const camera_translation,
                  const T* const focal_length, const T* const distortion_k1,
                  const T* const distortion_k2, const T* const point,
                  T* residuals) const
  {
    // Get transformation from world to camera frame
    // Inverse the camera transformation
    T camera_axis_angle_inv[3]; // R'
    camera_axis_angle_inv[0] = -camera_axis_angle[0];
    camera_axis_angle_inv[1] = -camera_axis_angle[1];
    camera_axis_angle_inv[2] = -camera_axis_angle[2];

    T camera_translation_inv[3];
    // R't
    ceres::AngleAxisRotatePoint(camera_axis_angle_inv, camera_translation,
                                camera_translation_inv);
    // t' = -R't
    camera_translation_inv[0] = -camera_translation_inv[0];
    camera_translation_inv[1] = -camera_translation_inv[1];
    camera_translation_inv[2] = -camera_translation_inv[2];

    // Transform point from world to camera frame
    T point_camera[3];
    ceres::AngleAxisRotatePoint(camera_axis_angle_inv, point, point_camera);
    point_camera[0] += camera_translation_inv[0];
    point_camera[1] += camera_translation_inv[1];
    point_camera[2] += camera_translation_inv[2];

    // Project point into image plane
    T x = point_camera[0] / point_camera[2];
    T y = point_camera[1] / point_camera[2];

    // Apply distortion
    T r2 = x * x + y * y;
    T distortion = T(1.0) + r2 * (distortion_k1[0] + distortion_k2[0] * r2);

    // Apply focal length
    T projected_x = focal_length[0] * distortion * x;
    T projected_y = focal_length[0] * distortion * y;

    // Compute residuals
    residuals[0] = projected_x - T(observed_pixel_.get()[0]);
    residuals[1] = projected_y - T(observed_pixel_.get()[1]);

    return true;
  }

  const std::reference_wrapper<const Pixel<>> observed_pixel_;
};

ceres::CostFunction* createReprojectionFactor(const Pixel<>& observed_pixel)
{
  return new ceres::AutoDiffCostFunction<ReprojectionFactor,
                                         /*Residuals*/ 2,
                                         /*Camera Axis Angle*/ 3,
                                         /*Camera Translation*/ 3,
                                         /*Focal Length*/ 1,
                                         /*Distortion K1*/ 1,
                                         /*Distortion K2*/ 1,
                                         /*Point*/ 3>(
      new ReprojectionFactor(observed_pixel));
}

} // namespace

namespace simple_sfm {
namespace ceres_back_end {

using types::OptimizationResult;
using types::Options;

OptimizationResult optimize(const types::SfmProblem<>& sfm_problem,
                            types::SfmVariables<>& optimized_sfm_variables,
                            const Options& options)
{

  ceres::Problem problem;
  for (const auto& observation : sfm_problem.observations)
  {
    auto& camera = optimized_sfm_variables.cameras[observation.camera_index];
    auto& point = optimized_sfm_variables.points[observation.point_index];
    const auto& observed_pixel = observation.pixel;

    ceres::CostFunction* cost_function =
        createReprojectionFactor(observed_pixel);
    problem.AddResidualBlock(cost_function, nullptr, camera.axis_angle.data(),
                             camera.translation.data(), &camera.focal_length,
                             &camera.distortion_k1, &camera.distortion_k2,
                             point.data());
  }

  // Set the camera pose of the first camera and the point as constant
  auto& first_camera = optimized_sfm_variables.cameras[0];
  problem.SetParameterBlockConstant(first_camera.axis_angle.data());
  problem.SetParameterBlockConstant(first_camera.translation.data());
  auto& first_point = optimized_sfm_variables.points[0];
  problem.SetParameterBlockConstant(first_point.data());

  ceres::Solver::Options solver_options;
  // Apperently, Ceres claims faster convergence with DENSE_SCHUR.
  // https://en.wikipedia.org/wiki/Schur_complement
  solver_options.linear_solver_type = ceres::DENSE_SCHUR;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.max_num_iterations = options.max_num_iterations;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  OptimizationResult optimization_result;
  optimization_result.converged = summary.IsSolutionUsable();
  optimization_result.final_error = summary.final_cost;

  return optimization_result;
}

} // namespace ceres_back_end
} // namespace simple_sfm
