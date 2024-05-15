#ifndef SIMPLE_SFM_TYPES_H
#define SIMPLE_SFM_TYPES_H

#include <array>
#include <vector>

namespace simple_sfm {
namespace types {

/// @brief Default scalar type used in the library.
using DefaultScalarType = double;

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

} // namespace types
} // namespace simple_sfm

#endif // SIMPLE_SFM_TYPES_H
