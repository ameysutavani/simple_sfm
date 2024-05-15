#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <simple_sfm/types.h>

namespace simple_sfm_ros {

using simple_sfm::types::Camera;

inline visualization_msgs::Marker
createDefaultCameraMarkers(float R = 1.0f, float G = 0.0f, float B = 0.0f)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.r = R;
  marker.color.g = G;
  marker.color.b = B;
  marker.color.a = 1.0;

  return marker;
}

struct Frustum
{
  // Default frustum looking in +Z direction
  double s{0.1}; // side length
  Eigen::Vector3d apex{0.0, 0.0, 0.0};
  Eigen::Vector3d corner1{s / 2, s / 2, s};
  Eigen::Vector3d corner2{-s / 2, s / 2, s};
  Eigen::Vector3d corner3{-s / 2, -s / 2, s};
  Eigen::Vector3d corner4{s / 2, -s / 2, s};
};

// Eigen::AngleAxisd * Frustum operator
inline Frustum operator*(const Eigen::AngleAxisd& R, const Frustum& frustum)
{
  Frustum rotated_frustum;
  rotated_frustum.apex = R * frustum.apex;
  rotated_frustum.corner1 = R * frustum.corner1;
  rotated_frustum.corner2 = R * frustum.corner2;
  rotated_frustum.corner3 = R * frustum.corner3;
  rotated_frustum.corner4 = R * frustum.corner4;

  return rotated_frustum;
}

// Frustum + Eigen::Vector3d operator
inline Frustum operator+(const Frustum& frustum, const Eigen::Vector3d& t)
{
  Frustum translated_frustum;
  translated_frustum.apex = frustum.apex + t;
  translated_frustum.corner1 = frustum.corner1 + t;
  translated_frustum.corner2 = frustum.corner2 + t;
  translated_frustum.corner3 = frustum.corner3 + t;
  translated_frustum.corner4 = frustum.corner4 + t;

  return translated_frustum;
}

inline geometry_msgs::Point createPointMsg(const Eigen::Vector3d& point)
{
  geometry_msgs::Point point_msg{};
  point_msg.x = static_cast<float>(point[0]);
  point_msg.y = static_cast<float>(point[1]);
  point_msg.z = static_cast<float>(point[2]);

  return point_msg;
}

visualization_msgs::Marker
createCameraMarkers(const std::vector<Camera<>>& cameras, float R = 1.0f,
                    float G = 0.0f, float B = 0.0f)
{
  auto camera_markers = createDefaultCameraMarkers(R, G, B);

  for (const auto& camera : cameras)
  {
    // Transform camera frustum according to camera pose
    const Eigen::Vector3d axis_angle{camera.axis_angle[0], camera.axis_angle[1],
                                     camera.axis_angle[2]};
    const Eigen::AngleAxisd R{axis_angle.norm(), axis_angle.normalized()};
    const Eigen::Vector3d t{camera.translation[0], camera.translation[1],
                            camera.translation[2]};

    const Frustum frustum = R * Frustum{} + t;

    // Add the 4 frustum faces as line list
    // Front face wireframe
    camera_markers.points.push_back(createPointMsg(frustum.corner1));
    camera_markers.points.push_back(createPointMsg(frustum.corner2));
    camera_markers.points.push_back(createPointMsg(frustum.corner2));
    camera_markers.points.push_back(createPointMsg(frustum.corner3));
    camera_markers.points.push_back(createPointMsg(frustum.corner3));
    camera_markers.points.push_back(createPointMsg(frustum.corner4));
    camera_markers.points.push_back(createPointMsg(frustum.corner4));
    camera_markers.points.push_back(createPointMsg(frustum.corner1));
    // Apex to corners wires
    camera_markers.points.push_back(createPointMsg(frustum.apex));
    camera_markers.points.push_back(createPointMsg(frustum.corner1));
    camera_markers.points.push_back(createPointMsg(frustum.apex));
    camera_markers.points.push_back(createPointMsg(frustum.corner2));
    camera_markers.points.push_back(createPointMsg(frustum.apex));
    camera_markers.points.push_back(createPointMsg(frustum.corner3));
    camera_markers.points.push_back(createPointMsg(frustum.apex));
    camera_markers.points.push_back(createPointMsg(frustum.corner4));
  }

  return camera_markers;
}

} // namespace simple_sfm_ros
