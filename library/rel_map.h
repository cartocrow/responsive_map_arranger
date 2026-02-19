//
// Created by arjen on 2/19/26.
//

#ifndef CARTOCROW_MODULENAME_REGIONMAP_H
#define CARTOCROW_MODULENAME_REGIONMAP_H

#include "region.h"
#include <unordered_map>
#include <stdexcept>

class RELmap {
public:
    RegionId addRegion(const std::string& name, int weight);

    RegionId getId(const std::string& name) const {
        auto it = name_to_id.find(name);
        if (it == name_to_id.end())
            throw std::runtime_error("Unknown region: " + name);

        return it->second;
    }
    Region& get(RegionId id) { return regions.at(id); }
    const Region& get(RegionId id) const { return regions.at(id); }

    void addRedEdge(const std::string& from, const std::string& to)
    {
        regions[getId(from)].red_out.push_back(getId(to));
    }

    void addBlueEdge(const std::string& from, const std::string& to)
    {
        regions[getId(from)].blue_out.push_back(getId(to));
    }

private:
    std::vector<Region> regions;
    std::unordered_map<std::string, RegionId> name_to_id;
};


#endif //CARTOCROW_MODULENAME_REGIONMAP_H