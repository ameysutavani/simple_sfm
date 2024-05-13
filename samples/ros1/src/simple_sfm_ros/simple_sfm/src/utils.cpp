#include <simple_sfm/utils.h>

using simple_sfm::types::AxisAngle;
using simple_sfm::types::SfmProblem;
using simple_sfm::types::Translation;

namespace {

std::pair<AxisAngle<>, Translation<>>
convertPoseFromBALToWorld(const AxisAngle<>& bal_axis_angle,
                          const Translation<>& bal_translation)
{
  return {bal_axis_angle, bal_translation};
}

} // namespace

namespace simple_sfm {
namespace utils {

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
    file >> observation.camera_index >> observation.point_index >>
        observation.pixel[0] >> observation.pixel[1];
    observation.pixel[1] =
        -observation.pixel[1]; // Invert v coordinate to transform from OpenGL
                               // to GTSAM (like OpenCV) convention.
  }

  // Read camera parameters.
  for (auto& camera : problem.variables.cameras)
  {
    // TODO: transform to world frame and from OpenGL to GTSAM (like OpenCV)
    // convention.
    file >> camera.axis_angle[0] >> camera.axis_angle[1] >>
        camera.axis_angle[2];
    file >> camera.translation[0] >> camera.translation[1] >>
        camera.translation[2];
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
