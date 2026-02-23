//
// Created by arjen on 2/19/26.
//

#ifndef CARTOCROW_MODULENAME_REGIONMAP_H
#define CARTOCROW_MODULENAME_REGIONMAP_H

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>

using json = nlohmann::json;
using RegionId = std::uint32_t;

struct Region
{
    std::string label;
    int weight = 0;

    // adjacency lists
    std::vector<RegionId> red_out;
    std::vector<RegionId> blue_out;
};

struct ValidationOptions {
    bool detect_missing_references = true;
    bool detect_duplicate_labels = true;
    bool detect_self_loops = true;
    bool detect_duplicate_edges = true;
    bool detect_order_problems = true;
    bool detect_unreachable = true;
    bool detect_cycles = false; // can be a bit expensive on large graphs
    bool check_negative_weights = true;
    bool strict = false; // when true constructor throws on errors
};

struct Diagnostics {
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    bool hasErrors() const noexcept { return !errors.empty(); }
    bool hasWarnings() const noexcept { return !warnings.empty(); }
};

class RELmap {
public:
    RELmap() = default;
    explicit RELmap(const json& j, const ValidationOptions& opts = ValidationOptions());

    Diagnostics validate(const ValidationOptions& opts = ValidationOptions()) const;

    RegionId getId(const std::string& name) const;
    Region& get(RegionId id);
    const Region& get(RegionId id) const;

    RegionId addRegion(const std::string& name, int weight);
    void addRedEdge(const std::string& from, const std::string& to);
    void addBlueEdge(const std::string& from, const std::string& to);

    const std::vector<RegionId>& horizontalOrder() const { return horizontal_order; }
    const std::vector<RegionId>& verticalOrder() const { return vertical_order; }

    std::size_t size() const noexcept { return regions.size(); }

private:
    std::vector<Region> regions;
    std::unordered_map<std::string, RegionId> name_to_id;

    std::vector<RegionId> horizontal_order;
    std::vector<RegionId> vertical_order;
};


#endif //CARTOCROW_MODULENAME_REGIONMAP_H