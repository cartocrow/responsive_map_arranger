#ifndef CARTOCROW_MODULENAME_REGION_H
#define CARTOCROW_MODULENAME_REGION_H


#include <string>
#include <vector>
#include <cstdint>

using RegionId = std::uint32_t;

struct Region
{
    std::string label;
    int weight = 0;

    // adjacency lists
    std::vector<RegionId> red_out;
    std::vector<RegionId> blue_out;
};

#endif //CARTOCROW_MODULENAME_REGION_H