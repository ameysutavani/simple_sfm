#ifndef SIMPLE_SFM_UTILS_H
#define SIMPLE_SFM_UTILS_H

#include <fstream>
#include <simple_sfm/types.h>

namespace simple_sfm {
namespace utils {

/// @brief Load a SfM problem from a BAL dataset
/// (https://grail.cs.washington.edu/projects/bal/) file stream.
types::SfmProblem<> loadFromBALFileStream(std::ifstream& file);

} // namespace utils
} // namespace simple_sfm

#endif // SIMPLE_SFM_UTILS_H
