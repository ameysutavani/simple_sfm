#include <cassert>
#include <fstream>
#include <iostream>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h> // Only for gtsam::SfmCamera
#include <simple_sfm/factor_graph_back_end.h>

namespace {

inline gtsam::Point2
convertToGtsamPoint2(const simple_sfm::types::Pixel<>& pixel)
{
  return gtsam::Point2{pixel[0], pixel[1]};
}

inline gtsam::Point3
convertToGtsamPoint3(const simple_sfm::types::Vector3<>& simple_sfm_point)
{
  return gtsam::Point3{simple_sfm_point[0], simple_sfm_point[1],
                       simple_sfm_point[2]};
}

inline simple_sfm::types::Vector3<>
convertToSimpleSfmVector3(const gtsam::Point3& point)
{
  return simple_sfm::types::Vector3<>{point.x(), point.y(), point.z()};
}

inline gtsam::SfmCamera
convertToGtsamCamera(const simple_sfm::types::Camera<>& camera)
{
  // Convert simple_sfm::types::AxisAngle to gtsam::Rot3
  const auto rotation = gtsam::Rot3::rodriguez(
      camera.axis_angle[0], camera.axis_angle[1], camera.axis_angle[2]);
  const auto translation = convertToGtsamPoint3(camera.translation);
  const auto camera_pose = gtsam::Pose3(rotation, translation);
  const auto camera_intrinsics = gtsam::Cal3Bundler(
      camera.focal_length, camera.distortion_k1, camera.distortion_k2);

  return gtsam::SfmCamera(camera_pose, camera_intrinsics);
}

} // namespace

namespace simple_sfm {
namespace factor_graph_back_end {

using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::PriorFactor;
using gtsam::Values;
using gtsam::symbol_shorthand::C; // For the Camera variable
using gtsam::symbol_shorthand::P; // For the Point variable

using SfmProjectionFactor =
    gtsam::GeneralSFMFactor<gtsam::SfmCamera, gtsam::Point3>;

OptimizationResult optimize(const types::SfmProblem<>& sfm_problem,
                            types::SfmVariables<>& optimized_sfm_variables,
                            const Options& options)
{
  // Assert that the number of cameras, points, and observations are greater
  // than 0
  assert(sfm_problem.variables.cameras.size() > 0);
  assert(sfm_problem.variables.points.size() > 0);
  assert(sfm_problem.observations.size() > 0);

  // Assert that size of the optimized_sfm_variables is the same as the size of
  // the input sfm_problem.variables
  assert(optimized_sfm_variables.cameras.size() ==
         sfm_problem.variables.cameras.size());
  assert(optimized_sfm_variables.points.size() ==
         sfm_problem.variables.points.size());

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Noise model for the projection factors (sigma * I(2), in the projection
  // coordinates)
  gtsam::noiseModel::Isotropic::shared_ptr noise =
      gtsam::noiseModel::Isotropic::Sigma(2, options.projection_factor_sigma);

  // Add a projection factor for each observation
  for (const auto& observation : sfm_problem.observations)
  {
    const auto uv = convertToGtsamPoint2(observation.pixel);
    graph.push_back(SfmProjectionFactor(uv, noise, C(observation.camera_index),
                                        P(observation.point_index)));
  }

  // Anchor the first camera and the first point by a prior factor. Essentially,
  // the first camera and the first point are fixed in the world frame.
  graph.push_back(PriorFactor<gtsam::SfmCamera>(
      C(0), convertToGtsamCamera(sfm_problem.variables.cameras[0]),
      gtsam::noiseModel::Isotropic::Sigma(9, options.prior_factor_sigma)));
  graph.push_back(PriorFactor<gtsam::Point3>(
      P(0), convertToGtsamPoint3(sfm_problem.variables.points[0]),
      gtsam::noiseModel::Isotropic::Sigma(3, options.prior_factor_sigma)));

  // Create initial estimate
  Values initial;
  for (size_t i{0U}; i < sfm_problem.variables.cameras.size(); ++i)
  {
    initial.insert(C(i),
                   convertToGtsamCamera(sfm_problem.variables.cameras[i]));
  }
  for (size_t i{0U}; i < sfm_problem.variables.points.size(); ++i)
  {
    initial.insert(P(i), convertToGtsamPoint3(sfm_problem.variables.points[i]));
  }

  /* Optimize the graph and print results */
  Values result;
  try
  {
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    result = lm.optimize();
  }
  catch (exception& e)
  {
    std::cout << e.what();
    return OptimizationResult{false, 0.0};
  }

  std::cout << "final error: " << graph.error(result) << std::endl;

  // Update the optimized_sfm_variables
  // for (size_t i{0U}; i < sfm_problem.variables.cameras.size(); ++i)
  // {
  //   optimized_sfm_variables.cameras[i] =
  //       convertToSimpleSfmCamera(result.at<gtsam::SfmCamera>(C(i)));
  // }

  for (size_t i{0U}; i < sfm_problem.variables.points.size(); ++i)
  {
    optimized_sfm_variables.points[i] =
        convertToSimpleSfmVector3(result.at<gtsam::Point3>(P(i)));
  }

  OptimizationResult optimization_result;
  optimization_result.converged = true;
  optimization_result.final_error = graph.error(result);
  return optimization_result;
}

} // namespace factor_graph_back_end
} // namespace simple_sfm
