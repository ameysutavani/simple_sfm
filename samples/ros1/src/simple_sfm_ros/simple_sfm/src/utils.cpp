#include <Eigen/Dense>
#include <simple_sfm/utils.h>

using simple_sfm::types::AxisAngle;
using simple_sfm::types::SfmProblem;
using simple_sfm::types::Translation;

namespace simple_sfm {
namespace utils {

// TODO: Find a nice way to test this function.
std::pair<AxisAngle<>, Translation<>>
convertPoseFromBALToWorld(const AxisAngle<>& bal_axis_angle,
                          const Translation<>& bal_translation)
{
  // NOTE: From the GTSAM camera's perspective, the OpenGL camera is rotated by
  // 180 degrees about the x-axis.
  // A simple way to think about this is that the OpenGL camera is looking in
  // the negative z-direction. The GTSAM camera is looking in the positive
  // z-direction. OpenGL's camera is mirrored about the x-axis. (i.e. rotated by
  // 180 degrees about the x-axis)
  const Eigen::AngleAxisd GTSAM_CAM_R_OPEN_GL_CAM{M_PI,
                                                  Eigen::Vector3d::UnitX()};

  // NOTE: BAL's pose is measured in the respective camera's frame. We need to
  // convert it to the world frame and flip the camera to GTSAM's convention.
  const Eigen::Vector3d bal_rodrigues_vector{
      bal_axis_angle[0], bal_axis_angle[1], bal_axis_angle[2]};

  // NOTE: Using GTSAM's convention to name the variables.
  // Rotation in the camera frame.
  const Eigen::AngleAxisd bal_cRw{bal_rodrigues_vector.norm(),
                                  bal_rodrigues_vector.normalized()};
  // Translation in the camera frame.
  const Eigen::Vector3d bal_cTw{bal_translation[0], bal_translation[1],
                                bal_translation[2]};

  // Roration in the world frame. But the camera is still flipped (is still in
  // the OpenGL convention).
  const Eigen::AngleAxisd bal_wRc{bal_cRw.inverse()};

  // NOTE: Post multiply GTSAM_CAM_R_OPEN_GL_CAM to flip the camera.
  const Eigen::AngleAxisd wRc{bal_wRc * GTSAM_CAM_R_OPEN_GL_CAM};

  // NOTE: Translation is not affected by the flipped camera issue. Just convert
  // translation to world frame. (R' * -t)
  const Eigen::Vector3d wTc = bal_wRc * -bal_cTw;

  // Convert to simple_sfm types.
  const Eigen::Vector3d axis_angle_vector = wRc.angle() * wRc.axis();
  const AxisAngle<> axis_angle{axis_angle_vector[0], axis_angle_vector[1],
                               axis_angle_vector[2]};
  const Translation<> translation{wTc[0], wTc[1], wTc[2]};

  return std::make_pair(axis_angle, translation);
}

SfmProblem<> loadFromBALFileStream(std::ifstream& file)
{
  SfmProblem<> problem;

  std::size_t num_cameras{0U};
  std::size_t num_points{0U};
  std::size_t num_observations{0U};

  // Get the number of cameras, points, and observations from the first line of
  // the file.
  file >> num_cameras >> num_points >> num_observations;

  problem.variables.cameras.resize(num_cameras);
  problem.variables.points.resize(num_points);
  problem.observations.resize(num_observations);

  // Read observations.
  for (auto& observation : problem.observations)
  {
    file >> observation.camera_index >> observation.point_index;
    file >> observation.pixel[0] >> observation.pixel[1];
    // Invert v coordinate to transform the camera from OpenGL to GTSAM (like
    // OpenCV) convention.
    observation.pixel[1] = -observation.pixel[1];
  }

  // Read camera parameters.
  for (auto& camera : problem.variables.cameras)
  {
    AxisAngle<> bal_axis_angle;
    Translation<> bal_translation;

    file >> bal_axis_angle[0] >> bal_axis_angle[1] >> bal_axis_angle[2];
    file >> bal_translation[0] >> bal_translation[1] >> bal_translation[2];

    const auto corrected_pose =
        convertPoseFromBALToWorld(bal_axis_angle, bal_translation);

    camera.axis_angle = corrected_pose.first;
    camera.translation = corrected_pose.second;

    file >> camera.focal_length >> camera.distortion_k1 >> camera.distortion_k2;
  }

  // Read point coordinates.
  for (auto& point : problem.variables.points)
  {
    file >> point[0] >> point[1] >> point[2];
  }

  return problem;
}

} // namespace utils
} // namespace simple_sfm
