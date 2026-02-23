//
// Created by arjen on 2/19/26.
//

#include "rel_map.h"
#include <stdexcept>
#include <sstream>
#include <queue>
#include <stack>
#include <algorithm>
#include <iostream>

using json = nlohmann::json;

RELmap::RELmap(const json& j, const ValidationOptions& opts)
{
    // PASS 1: create nodes
    for (const auto& r : j.at("regions")) {
        std::string label = r.at("label").get<std::string>();
        int weight = r.at("weight").get<int>();
        addRegion(label, weight);
    }

    // PASS 2: create edges
    for (const auto& r : j.at("regions")) {
        std::string label = r.at("label").get<std::string>();

        for (const auto& dst : r.at("red_out"))
            addRedEdge(label, dst.get<std::string>());

        for (const auto& dst : r.at("blue_out"))
            addBlueEdge(label, dst.get<std::string>());
    }

    // ORDERS (resolve to ids where possible)
    if (j.contains("horizontal_order")) {
        for (const auto& name : j.at("horizontal_order")) {
            const std::string nm = name.get<std::string>();
            auto it = name_to_id.find(nm);
            if (it != name_to_id.end()) horizontal_order.push_back(it->second);
            else horizontal_order.push_back(static_cast<RegionId>(-1));
        }
    }
    if (j.contains("vertical_order")) {
        for (const auto& name : j.at("vertical_order")) {
            const std::string nm = name.get<std::string>();
            auto it = name_to_id.find(nm);
            if (it != name_to_id.end()) vertical_order.push_back(it->second);
            else vertical_order.push_back(static_cast<RegionId>(-1));
        }
    }

    // Run validation (this prints diagnostics). Throw if strict && errors.
    Diagnostics d = validate(opts);
    if (opts.strict && d.hasErrors()) {
        std::ostringstream oss;
        oss << "RELmap validation failed with " << d.errors.size() << " error(s). See stderr for details.";
        throw std::runtime_error(oss.str());
    }
}

RegionId RELmap::addRegion(const std::string& name, int weight)
{
    if (name_to_id.contains(name))
        throw std::runtime_error("Duplicate region: " + name);

    RegionId id = regions.size();
    regions.push_back({name, weight});
    name_to_id[name] = id;
    return id;
}

RegionId RELmap::getId(const std::string& name) const
{
    auto it = name_to_id.find(name);
    if (it == name_to_id.end())
        throw std::runtime_error("Unknown region: " + name);
    return it->second;
}

Region& RELmap::get(RegionId id)
{
    return regions.at(id);
}

const Region& RELmap::get(RegionId id) const
{
    return regions.at(id);
}

void RELmap::addRedEdge(const std::string& from, const std::string& to)
{
    regions[getId(from)].red_out.push_back(getId(to));
}

void RELmap::addBlueEdge(const std::string& from, const std::string& to)
{
    regions[getId(from)].blue_out.push_back(getId(to));
}

// ---------------- Validation (prints diagnostics) ----------------

static std::string edgeToStr(const RELmap& m, RegionId from, RegionId to, const char* color) {
    std::ostringstream oss;
    oss << "'" << m.get(from).label << "' -" << color << "-> '" << m.get(to).label << "'";
    return oss.str();
}

Diagnostics RELmap::validate(const ValidationOptions& opts) const
{
    Diagnostics diag;
    const size_t n = regions.size();

    // Map label -> ids (for duplicate-detection)
    std::unordered_map<std::string, std::vector<RegionId>> label_to_ids;
    for (RegionId id = 0; id < static_cast<RegionId>(n); ++id) {
        label_to_ids[regions[id].label].push_back(id);
        if (opts.check_negative_weights && regions[id].weight < 0) {
            std::ostringstream ss;
            ss << "Region '" << regions[id].label << "' has negative weight " << regions[id].weight;
            diag.warnings.push_back(ss.str());
        }
    }

    if (opts.detect_duplicate_labels) {
        for (auto &kv : label_to_ids) {
            if (kv.second.size() > 1) {
                std::ostringstream ss;
                ss << "Duplicate label '" << kv.first << "' used for region ids: ";
                for (auto id : kv.second) ss << id << " ";
                diag.errors.push_back(ss.str());
            }
        }
    }

    // Validate edges (self-loops, duplicates, invalid ids)
    for (RegionId id = 0; id < static_cast<RegionId>(n); ++id) {
        auto checkOut = [&](const std::vector<RegionId>& outs, const char* color) {
            std::unordered_set<RegionId> seenEdges;
            for (RegionId to : outs) {
                if (to >= n) {
                    std::ostringstream ss;
                    ss << "Region '" << regions[id].label << "' has " << color << " edge to invalid id " << to;
                    diag.errors.push_back(ss.str());
                    continue;
                }
                if (opts.detect_self_loops && to == id) {
                    std::ostringstream ss;
                    ss << "Self-loop detected: " << edgeToStr(*this, id, to, color);
                    diag.errors.push_back(ss.str());
                }
                if (opts.detect_duplicate_edges) {
                    if (seenEdges.find(to) != seenEdges.end()) {
                        std::ostringstream ss;
                        ss << "Duplicate " << color << " edge: " << edgeToStr(*this, id, to, color);
                        diag.warnings.push_back(ss.str());
                    } else {
                        seenEdges.insert(to);
                    }
                }
            }
        };
        checkOut(regions[id].red_out, "red");
        checkOut(regions[id].blue_out, "blue");
    }

    // Check orders (invalid ids, duplicates, missing regions)
    auto checkOrder = [&](const std::vector<RegionId>& order, const char* name) {
        std::unordered_set<RegionId> seen;
        for (RegionId rid : order) {
            if (rid == static_cast<RegionId>(-1) || rid >= n) {
                std::ostringstream ss;
                ss << name << " contains invalid/unresolved region id " << rid;
                diag.errors.push_back(ss.str());
                continue;
            }
            if (seen.find(rid) != seen.end()) {
                std::ostringstream ss;
                ss << name << " contains duplicate entry for region '" << regions[rid].label << "'";
                diag.warnings.push_back(ss.str());
            }
            seen.insert(rid);
        }
        if (opts.detect_order_problems) {
            if (seen.size() != n - 4) { // Don't include West, North, East and South nodes
                std::ostringstream ss;
                ss << name << " length (" << order.size() << ") doesn't include all regions (" << n - 4 << ")";
                diag.warnings.push_back(ss.str());
            }
        }
    };

    if (!horizontal_order.empty())
        checkOrder(horizontal_order, "horizontal_order");
    if (!vertical_order.empty())
        checkOrder(vertical_order, "vertical_order");

    // Unreachable detection
    if (opts.detect_unreachable) {
        std::vector<int> indeg(n, 0);
        for (RegionId id = 0; id < static_cast<RegionId>(n); ++id) {
            for (auto to : regions[id].red_out) if (to < n) indeg[to]++;
            for (auto to : regions[id].blue_out) if (to < n) indeg[to]++;
        }

        std::queue<RegionId> q;
        std::vector<char> vis(n, 0);
        bool any_source = false;
        for (RegionId id = 0; id < static_cast<RegionId>(n); ++id) {
            if (indeg[id] == 0) {
                q.push(id);
                vis[id] = 1;
                any_source = true;
            }
        }
        if (!any_source) {
            RegionId start = 0;
            if (!horizontal_order.empty()) start = horizontal_order.front();
            q.push(start);
            vis[start] = 1;
        }

        while (!q.empty()) {
            RegionId cur = q.front(); q.pop();
            for (auto to : regions[cur].red_out) {
                if (to < n && !vis[to]) { vis[to]=1; q.push(to); }
            }
            for (auto to : regions[cur].blue_out) {
                if (to < n && !vis[to]) { vis[to]=1; q.push(to); }
            }
        }

        for (RegionId id = 0; id < static_cast<RegionId>(n); ++id) {
            if (!vis[id]) {
                std::ostringstream ss;
                ss << "Unreachable region '" << regions[id].label << "' (no path from any source)";
                diag.warnings.push_back(ss.str());
            }
        }
    }

    // Optional cycle detection
    if (opts.detect_cycles) {
        enum class Color { WHITE, GRAY, BLACK };
        std::vector<Color> color(n, Color::WHITE);
        std::vector<RegionId> parent(n, static_cast<RegionId>(-1));
        bool found_cycle = false;
        std::vector<RegionId> cycle_nodes;

        std::function<void(RegionId)> dfs = [&](RegionId u) {
            if (found_cycle) return;
            color[u] = Color::GRAY;
            auto visit = [&](RegionId v) {
                if (found_cycle) return;
                if (color[v] == Color::WHITE) {
                    parent[v] = u;
                    dfs(v);
                } else if (color[v] == Color::GRAY) {
                    found_cycle = true;
                    RegionId cur = u;
                    cycle_nodes.push_back(v);
                    while (cur != v && cur != static_cast<RegionId>(-1)) {
                        cycle_nodes.push_back(cur);
                        cur = parent[cur];
                    }
                }
            };

            for (auto v : regions[u].red_out) visit(v);
            for (auto v : regions[u].blue_out) visit(v);
            color[u] = Color::BLACK;
        };

        for (RegionId i = 0; i < static_cast<RegionId>(n) && !found_cycle; ++i) {
            if (color[i] == Color::WHITE) dfs(i);
        }

        if (found_cycle && !cycle_nodes.empty()) {
            std::ostringstream ss;
            ss << "Cycle detected in graph: ";
            for (auto id : cycle_nodes) {
                ss << "'" << regions[id].label << "' ";
            }
            diag.errors.push_back(ss.str());
        }
    }

    // --- PRINT DIAGNOSTICS (human-readable) ---
    {
        std::ostringstream header;
        header << "RELmap validation: " << (diag.hasErrors() ? "ERRORS" : "OK");
        header << " (" << diag.errors.size() << " errors, " << diag.warnings.size() << " warnings)";
        std::cerr << header.str() << std::endl;

        if (diag.hasErrors()) {
            std::cerr << "Errors:" << std::endl;
            for (const auto& e : diag.errors) {
                std::cerr << "  - " << e << std::endl;
            }
        }
        if (diag.hasWarnings()) {
            std::cerr << "Warnings:" << std::endl;
            for (const auto& w : diag.warnings) {
                std::cerr << "  - " << w << std::endl;
            }
        }
        if (!diag.hasErrors() && !diag.hasWarnings()) {
            std::cerr << "  (no issues found)" << std::endl;
        }
    }

    return diag;
}