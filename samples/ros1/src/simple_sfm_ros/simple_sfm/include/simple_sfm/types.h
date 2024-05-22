#ifndef SIMPLE_SFM_TYPES_H
#define SIMPLE_SFM_TYPES_H

#include <array>
#include <vector>

namespace simple_sfm {
namespace types {

/// @brief Default scalar type used in the library.
using DefaultScalarType = double;

// DESIGN NOTE: The DefaultScalarType is just nice to have here. It gives us the
// the option to choose a different scalat type if needed in the future. For the
// current library implementation, all the functions and utils assume
// DefaultScalarType. The library can be easily templated if needed in future.
// For now, we keep things simple.

/// @brief 2D vector type.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using Vector2 = std::array<ScalarType, 2>;

/// @brief 3D vector type.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using Vector3 = std::array<ScalarType, 3>;

/// @brief 3D point type.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using Point3 = Vector3<ScalarType>;

/// @brief Axis-angle representation of a rotation.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using AxisAngle = Vector3<ScalarType>;

/// @brief Representation of a 3D translation.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using Translation = Vector3<ScalarType>;

/// @brief Representation of a 2D pixel coordinates.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using Pixel = Vector2<ScalarType>;

/// @brief Structure representing a camera.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType> struct Camera
{
  // DESIGN NOTE: I would have preferred to use a quaternion for the rotation in
  // the types, but the BAL dataset has forced the hand by using axis-angle
  // representation. One side effect is that the representation is the SO(3)
  // group, so if we choose a different optimizer, it would be easier to stick
  // to axis angles.

  /// @brief Rotation of the camera with respect to the world frame.
  AxisAngle<ScalarType> axis_angle{};
  /// @brief Translation of the camera with respect to the world frame.
  Translation<ScalarType> translation{};
  /// @brief Focal length of the camera.
  ScalarType focal_length{};
  /// @brief Lens distortion parameters.
  ScalarType distortion_k1{};
  ScalarType distortion_k2{};
};

/// @brief Structure representing point to projection associations
/// (observations).
template <typename ScalarType = DefaultScalarType> struct Observation
{
  size_t camera_index{};
  size_t point_index{};
  Pixel<ScalarType> pixel{};
};

/// @brief A convenient structure to hold the variables of the SfM problem.
/// These parts will be optimized.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType> struct SfmVariables
{
  std::vector<Camera<ScalarType>> cameras;
  std::vector<Point3<ScalarType>> points;
};

/// @brief A convenient structure to hold the observations of the SfM problem.
/// These parts will be used as constants.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType>
using SfmObservations = std::vector<Observation<ScalarType>>;

/// @brief A convenient structure to hold the SfM problem.
/// @tparam ScalarType
template <typename ScalarType = DefaultScalarType> struct SfmProblem
{
  SfmVariables<ScalarType> variables;
  SfmObservations<ScalarType> observations;
};

/// @brief Options (tuning) for the optimization
struct Options
{
  /// @brief Noise standard deviation for the prior factors on the first camera
  /// pose and the first point
  DefaultScalarType prior_factor_sigma{0.1};
  /// @brief Noise standard deviation for the reprojection factors for each
  /// observation
  DefaultScalarType projection_factor_sigma{1.0};
  /// @brief Maximum number of iterations for the optimization
  std::size_t max_num_iterations{100};
};

/// @brief Struct to hold the optimization result and additional information
struct OptimizationResult
{
  /// @brief Flag to indicate if the optimization converged
  bool converged{false};
  /// @brief Final (total) reprojection error after optimization
  DefaultScalarType final_error{0.0};
};

} // namespace types
} // namespace simple_sfm

#endif // SIMPLE_SFM_TYPES_H
