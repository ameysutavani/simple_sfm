#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>

#include <simple_sfm/factor_graph_back_end.h>
#include <simple_sfm/utils.h>

using simple_sfm::factor_graph_back_end::optimize;
using simple_sfm::utils::loadFromBALFileStream;

SCENARIO("simple_sfm::factor_graph_back_end::optimize smoke test")
{
  GIVEN("A SfmProblem loaded from a TEST_BAL_DATA")
  {
    std::ifstream file(TEST_BAL_DATA, std::ifstream::in);
    REQUIRE(file.is_open());
    const auto problem = loadFromBALFileStream(file);
    file.close();

    WHEN("Calling optimize on the problem")
    {
      auto optimized_variables = problem.variables;
      const auto optimization_result = optimize(problem, optimized_variables);

      THEN("The optimization should succeed")
      {
        REQUIRE(optimization_result.converged);
        // As we do not have any ground truths, we cannot really check any error
        // metrics. Also, the final error can vary from platform to platform, so
        // choosing not to check it to avoid flakiness.

        // The size of optimized_variables should be the same as the size of the
        // problem.variables
        REQUIRE(optimized_variables.cameras.size() ==
                problem.variables.cameras.size());
        REQUIRE(optimized_variables.points.size() ==
                problem.variables.points.size());
      }
    }
  }
}
