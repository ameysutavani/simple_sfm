#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <simple_sfm/factor_graph_back_end.h>
#include <simple_sfm/utils.h>

#include "camera_viz.h"

namespace simple_sfm_ros {

constexpr const char* INITIAL_POINTS_TOPIC = "initial_points";
constexpr const char* OPTIMIZED_POINTS_TOPIC = "optimized_points";
constexpr const char* INITIAL_CAMERAS_TOPIC = "initial_cameras";
constexpr const char* OPTIMIZED_CAMERAS_TOPIC = "optimized_cameras";

using simple_sfm::factor_graph_back_end::optimize;
using simple_sfm::utils::loadFromBALFileStream;

using PointType = pcl::InterestPoint;
using CloudType = pcl::PointCloud<PointType>;

class SimpleSfmNode
{
public:
  explicit SimpleSfmNode()
      : initial_points_pub_(
            nh_.advertise<sensor_msgs::PointCloud2>(INITIAL_POINTS_TOPIC, 1)),
        optimized_points_pub_(
            nh_.advertise<sensor_msgs::PointCloud2>(OPTIMIZED_POINTS_TOPIC, 1)),
        initial_cameras_pub_(nh_.advertise<visualization_msgs::Marker>(
            INITIAL_CAMERAS_TOPIC, 1)),
        optimized_cameras_pub_(nh_.advertise<visualization_msgs::Marker>(
            OPTIMIZED_CAMERAS_TOPIC, 1))
  {
  }

  void run()
  {
    // Fetch the needed private parameters
    std::string bal_dataset_path;
    if (!ros::param::get("~bal_dataset_path", bal_dataset_path))
    {
      ROS_ERROR_STREAM("Could not get bal_dataset_path parameter");
      return;
    }

    bool save_csvs{true};
    std::ignore = ros::param::get("~save_csvs", save_csvs);
    std::string output_csv_folder_path;
    if (save_csvs)
    {
      if (!ros::param::get("~output_csv_folder_path", output_csv_folder_path))
      {
        ROS_ERROR_STREAM("Could not get output_csv_folder_path parameter");
        return;
      }
    }

    // Load the sfm_problem from the dataset file
    std::ifstream file(bal_dataset_path, std::ifstream::in);
    if (!file.is_open())
    {
      ROS_ERROR_STREAM("Could not open dataset file: " << bal_dataset_path);
      return;
    }
    const auto sfm_problem = loadFromBALFileStream(file);
    file.close();

    // Create storage for optimized variables
    auto optimized_variables = sfm_problem.variables;

    // Optimize the problem
    const auto optimization_result = optimize(sfm_problem, optimized_variables);

    if (save_csvs)
    {
      // Lambda for writing points to a csv file
      auto write_points_to_csv =
          [](const std::string& filename,
             const std::vector<simple_sfm::types::Point3<>>& points) {
            std::ofstream out(filename);
            out << "X,Y,Z\n";
            for (const auto& point : points)
            {
              out << point[0] << "," << point[1] << "," << point[2] << "\n";
            }
            out.close();
          };

      // Write initial points to a csv file
      write_points_to_csv(output_csv_folder_path + "initial_points.csv",
                          sfm_problem.variables.points);

      // Write optimized points to a csv file
      write_points_to_csv(output_csv_folder_path + "optimized_points.csv",
                          optimized_variables.points);
    }

    // Lambda for getting l2 distance between two points
    auto get_l2_distance = [](const simple_sfm::types::Point3<>& point1,
                              const simple_sfm::types::Point3<>& point2) {
      return std::pow(point1[0] - point2[0], 2) +
             std::pow(point1[1] - point2[1], 2) +
             std::pow(point1[2] - point2[2], 2);
    };

    // Lambda for getting point-to-point corrections
    auto get_point_corrections =
        [&get_l2_distance](
            const std::vector<simple_sfm::types::Point3<>>& initial_points,
            const std::vector<simple_sfm::types::Point3<>>& optimized_points) {
          std::vector<float> point_corrections;
          point_corrections.reserve(initial_points.size());

          for (size_t i = 0; i < initial_points.size(); ++i)
          {
            const auto& initial_point = initial_points[i];
            const auto& optimized_point = optimized_points[i];
            point_corrections.push_back(static_cast<float>(
                get_l2_distance(initial_point, optimized_point)));
          }

          return point_corrections;
        };

    // Lambda for converting points to PointCloud message
    auto points_to_point_cloud =
        [](const std::vector<simple_sfm::types::Point3<>>& points,
           const std::vector<float>& corrections) {
          CloudType cloud;
          cloud.reserve(points.size());

          for (size_t i = 0; i < points.size(); ++i)
          {
            const auto& point = points[i];
            const auto& correction = corrections[i];

            PointType pcl_point;
            pcl_point.x = point[0];
            pcl_point.y = point[1];
            pcl_point.z = point[2];
            // DESIGN-NOTE: This is ugly; I would have preferred to define a
            // custom point type, but keeping it simple for now
            pcl_point.strength = correction;
            cloud.push_back(pcl_point);
          }

          sensor_msgs::PointCloud2 cloud_msg;
          pcl::toROSMsg(cloud, cloud_msg);
          cloud_msg.header.frame_id = "bal";

          return cloud_msg;
        };

    const auto point_corrections = get_point_corrections(
        sfm_problem.variables.points, optimized_variables.points);
    const auto initial_points_msg =
        points_to_point_cloud(sfm_problem.variables.points, point_corrections);
    const auto optimized_points_msg =
        points_to_point_cloud(optimized_variables.points, point_corrections);

    // Create markers for camera poses
    auto initial_cameras_msg = createCameraMarkers(
        sfm_problem.variables.cameras, /*White*/ 1.0f, 1.0f, 1.0f);
    initial_cameras_msg.header.frame_id = "bal";
    auto optimized_cameras_msg = createCameraMarkers(
        optimized_variables.cameras, /*Green*/ 0.0f, 1.0f, 0.0f);
    optimized_cameras_msg.header.frame_id = "bal";

    // Keep publishing the points and camera poses for visualization
    while (ros::ok())
    {
      initial_points_pub_.publish(initial_points_msg);
      optimized_points_pub_.publish(optimized_points_msg);
      initial_cameras_pub_.publish(initial_cameras_msg);
      optimized_cameras_pub_.publish(optimized_cameras_msg);
      ros::Duration(1.0).sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher initial_points_pub_;
  ros::Publisher optimized_points_pub_;
  ros::Publisher initial_cameras_pub_;
  ros::Publisher optimized_cameras_pub_;
};

} // namespace simple_sfm_ros

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_sfm_node");

  simple_sfm_ros::SimpleSfmNode simple_sfm_node;
  simple_sfm_node.run();

  ros::spin();

  return EXIT_SUCCESS;
}
