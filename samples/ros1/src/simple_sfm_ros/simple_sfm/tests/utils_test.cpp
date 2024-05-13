#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>
#include <simple_sfm/utils.h>

using namespace simple_sfm::utils;

SCENARIO("Test loading from a BAL file stream")
{
  GIVEN("A test BAL file stream")
  {
    std::ifstream file(TEST_BAL_DATA, std::ifstream::in);
    REQUIRE(file.is_open());

    WHEN("Calling loadFromBALFileStream on the file stream")
    {
      const auto problem = loadFromBALFileStream(file);
      file.close();

      THEN("The SfmProblem should be loaded correctly")
      {
        REQUIRE(problem.variables.cameras.size() == 21U);
        REQUIRE(problem.variables.points.size() == 11315U);
        REQUIRE(problem.observations.size() == 36455U);

        // Check the loaded data matches that of the test file for a few
        // entries. Check parameters of the second observation.
        const auto& observation = problem.observations[1];
        REQUIRE(observation.camera_index == 1U);
        REQUIRE(observation.point_index == 0U);
        REQUIRE(observation.pixel[0] == Approx(7.217000e+02).margin(1e-6));
        REQUIRE(observation.pixel[1] ==
                Approx(-5.229800e+02).margin(1e-6)); // Inverted v coordinate.

        // Check parameters of the first camera.
        /*
        L35457 - L36465 in the test file:
        -3.4265630475549310e-03
        9.6448984178004217e-03
        4.6562227695769124e-03
        8.2039923232261648e-03
        4.1758870714924805e-04
        1.0139126763172436e-01
        2.8443148232166736e+03
        -2.0200951857532239e-08
        2.1246398231307891e-15
        */
        const auto& camera = problem.variables.cameras[0];
        // TODO: transform to world frame and from OpenGL to GTSAM (like OpenCV)
        // convention.
        REQUIRE(camera.axis_angle[0] ==
                Approx(-3.4265630475549310e-03).margin(1e-6));
        REQUIRE(camera.axis_angle[1] ==
                Approx(9.6448984178004217e-03).margin(1e-6));
        REQUIRE(camera.axis_angle[2] ==
                Approx(4.6562227695769124e-03).margin(1e-6));
        REQUIRE(camera.translation[0] ==
                Approx(8.2039923232261648e-03).margin(1e-6));
        REQUIRE(camera.translation[1] ==
                Approx(4.1758870714924805e-04).margin(1e-6));
        REQUIRE(camera.translation[2] ==
                Approx(1.0139126763172436e-01).margin(1e-6));
        REQUIRE(camera.focal_length ==
                Approx(2.8443148232166736e+03).margin(1e-6));
        REQUIRE(camera.distortion_k1 ==
                Approx(-2.0200951857532239e-08).margin(1e-6));
        REQUIRE(camera.distortion_k2 ==
                Approx(2.1246398231307891e-15).margin(1e-6));

        // Chect the last point.
        /*
        L70588 - L70590 in the test file:
        -4.0293219749836723e-01
        2.9001798329494261e-01
        -3.8793900522827331e+00
        */
        const auto& point = problem.variables.points.back();
        REQUIRE(point[0] == Approx(-4.0293219749836723e-01).margin(1e-6));
        REQUIRE(point[1] == Approx(2.9001798329494261e-01).margin(1e-6));
        REQUIRE(point[2] == Approx(-3.8793900522827331e+00).margin(1e-6));
      }
    }
  }
}
