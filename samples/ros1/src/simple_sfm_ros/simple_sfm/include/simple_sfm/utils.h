#ifndef SIMPLE_SFM_UTILS_H
#define SIMPLE_SFM_UTILS_H

#include <fstream>
#include <simple_sfm/types.h>

namespace simple_sfm {
namespace utils {

/// @brief Convert a pose from the BAL dataset to the standardized world
/// coordinate system.
/// @param [in] bal_axis_angle Rotation in BAL dataset. (In the OpenGL camera
/// frame)
/// @param [in] bal_translation Translation in BAL dataset. (In the camera
/// frame)
/// @return A pair of the converted axis-angle rotation and translation. (In the
/// world frame, using GTSAM/OpenCV camera frame convention)
std::pair<types::AxisAngle<>, types::Translation<>>
convertPoseFromBALToWorld(const types::AxisAngle<>& bal_axis_angle,
                          const types::Translation<>& bal_translation);

/// @brief Load a SfM problem from a BAL dataset
/// (https://grail.cs.washington.edu/projects/bal/) file stream.
types::SfmProblem<> loadFromBALFileStream(std::ifstream& file);

} // namespace utils
} // namespace simple_sfm

#endif // SIMPLE_SFM_UTILS_H
