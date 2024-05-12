#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <simple_sfm/types.h>

using namespace simple_sfm::types;

SCENARIO("Test simple_sfm::types::Camera construction")
{
    GIVEN("Default constructed camera struct")
    {
        Camera<double> camera;
        REQUIRE(camera.axis_angle[0] == 0);
        REQUIRE(camera.axis_angle[1] == 0);
        REQUIRE(camera.axis_angle[2] == 0);
        REQUIRE(camera.translation[0] == 0);
        REQUIRE(camera.translation[1] == 0);
        REQUIRE(camera.translation[2] == 0);
        REQUIRE(camera.focal_length == 0);
        REQUIRE(camera.distortion_k1 == 0);
        REQUIRE(camera.distortion_k2 == 0);
    }

    GIVEN("Camera struct constructued with non-default with values")
    {
        Camera<float> camera{
            {1.0f, 2.0f, 3.0f}, // axis_angle
            {4.0f, 5.0f, 6.0f}, // translation
            7.0f,               // focal_length
            8.0f,               // distortion_k1
            9.0f                // distortion_k2
        };
        REQUIRE(camera.axis_angle[0] == Approx(1.0f));
        REQUIRE(camera.axis_angle[1] == Approx(2.0f));
        REQUIRE(camera.axis_angle[2] == Approx(3.0f));
        REQUIRE(camera.translation[0] == Approx(4.0f));
        REQUIRE(camera.translation[1] == Approx(5.0f));
        REQUIRE(camera.translation[2] == Approx(6.0f));
        REQUIRE(camera.focal_length == Approx(7.0f));
        REQUIRE(camera.distortion_k1 == Approx(8.0f));
        REQUIRE(camera.distortion_k2 == Approx(9.0f));
    }
}

SCENARIO("Test simple_sfm::types::Observation construction")
{
    GIVEN("Default constructed observation struct")
    {
        Observation<double> observation;
        REQUIRE(observation.point_index == 0);
        REQUIRE(observation.camera_index == 0);
        REQUIRE(observation.pixel[0] == 0); // column, u
        REQUIRE(observation.pixel[1] == 0); // row, v
    }

    GIVEN("Observation struct constructed with non-default values")
    {
        Observation<float> observation{
            1,           // point_index
            2,           // camera_index
            {3.0f, 4.0f} // pixel
        };
        REQUIRE(observation.point_index == 1);
        REQUIRE(observation.camera_index == 2);
        REQUIRE(observation.pixel[0] == Approx(3.0f));
        REQUIRE(observation.pixel[1] == Approx(4.0f));
    }
}

SCENARIO("Test SfmProblem construction")
{
    GIVEN("Default constructed SfmProblem")
    {
        SfmProblem<double> problem;
        REQUIRE(problem.observations.size() == 0U);
        REQUIRE(problem.variables.points.size() == 0U);
        REQUIRE(problem.variables.cameras.size() == 0U);
    }
}
