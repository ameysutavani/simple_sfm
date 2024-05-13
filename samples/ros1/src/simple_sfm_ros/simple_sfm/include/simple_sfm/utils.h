#ifndef SIMPLE_SFM_UTILS_H
#define SIMPLE_SFM_UTILS_H

#include <simple_sfm/types.h>
#include <fstream>

namespace simple_sfm
{
    namespace utils
    {
        /// @brief Load a SfM problem from a BAL dataset (https://grail.cs.washington.edu/projects/bal/) file stream.
        types::SfmProblem<> loadFromBALFileStream(std::ifstream &file);
    }
}

#endif // SIMPLE_SFM_UTILS_H
