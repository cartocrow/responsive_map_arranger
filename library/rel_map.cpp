//
// Created by arjen on 2/19/26.
//

#include "rel_map.h"

RegionId RELmap::addRegion(const std::string& name, int weight)
{
    if (name_to_id.contains(name))
        throw std::runtime_error("Duplicate region: " + name);

    RegionId id = regions.size();
    regions.push_back({name, weight});
    name_to_id[name] = id;
    return id;
}