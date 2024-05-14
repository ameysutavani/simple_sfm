#ifndef SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H
#define SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H

#include <simple_sfm/types.h>

namespace simple_sfm {
namespace factor_graph_back_end {

struct Options
{
  double prior_factor_sigma{0.1};
  double projection_factor_sigma{1.0};
};

types::SfmVariables<> optimize(const types::SfmProblem<>& sfm_problem,
                               const Options& options = Options{});

} // namespace factor_graph_back_end
} // namespace simple_sfm

#endif // SIMPLE_SFM_FACTOR_GRAPH_BACK_END_H
