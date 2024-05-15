#include <fstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <simple_sfm/factor_graph_back_end.h>
#include <simple_sfm/utils.h>

namespace {

constexpr const char* INITIAL_POINTS_TOPIC = "initial_points";
constexpr const char* OPTIMIZED_POINTS_TOPIC = "optimized_points";

using simple_sfm::factor_graph_back_end::optimize;
using simple_sfm::utils::loadFromBALFileStream;

using CloudType = pcl::PointCloud<pcl::PointXYZI>;

class SimpleSfmNode
{
public:
  explicit SimpleSfmNode()
      : initial_points_pub_(
            nh_.advertise<sensor_msgs::PointCloud2>(INITIAL_POINTS_TOPIC, 1)),
        optimized_points_pub_(
            nh_.advertise<sensor_msgs::PointCloud2>(OPTIMIZED_POINTS_TOPIC, 1))
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

    // Lambda for converting points to PointCloud message
    auto points_to_point_cloud =
        [](const std::vector<simple_sfm::types::Point3<>>& points) {
          CloudType cloud;
          cloud.reserve(points.size());

          for (const auto& point : points)
          {
            pcl::PointXYZI pcl_point;
            pcl_point.x = point[0];
            pcl_point.y = point[1];
            pcl_point.z = point[2];
            pcl_point.intensity = 1.0;
            cloud.push_back(pcl_point);
          }

          sensor_msgs::PointCloud2 cloud_msg;
          pcl::toROSMsg(cloud, cloud_msg);
          cloud_msg.header.frame_id = "bal";

          return cloud_msg;
        };

    const auto initial_points_msg =
        points_to_point_cloud(sfm_problem.variables.points);
    const auto optimized_points_msg =
        points_to_point_cloud(optimized_variables.points);

    // Keep publishing initial and optimized points messages
    while (ros::ok())
    {
      initial_points_pub_.publish(initial_points_msg);
      optimized_points_pub_.publish(optimized_points_msg);
      ros::Duration(1.0).sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher initial_points_pub_;
  ros::Publisher optimized_points_pub_;
};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_sfm_node");

  SimpleSfmNode simple_sfm_node;
  simple_sfm_node.run();

  ros::spin();

  return EXIT_SUCCESS;
}
