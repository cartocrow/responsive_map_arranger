// rectangular_dual.h
#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <optional>

#include "regular_edge_labeling.h"

// forward-declare old RELmap (kept for compatibility)
class RELmap;

// forward-declare RegularEdgeLabeling (new path)
class RegularEdgeLabeling;

class RectangularDual {
public:
    struct Rect {
        double left;
        double right;
        double bottom;
        double top;
    };

    struct STGraph {
        int source = -1;
        int sink = -1;
        std::vector<std::vector<int>> out; // adjacency out: u -> list of v
        std::vector<std::vector<int>> in; // adjacency in: v -> list of u
    };

    RectangularDual() = default;

    bool buildSTGraphsFromREL(const RegularEdgeLabeling &rel);

    // Build rectangular dual from RELmap (legacy)
    // cell_size: scale for each grid cell (visual size).
    bool initializeFromREL(const RELmap &rel, double cell_size = 40.0);

    // NEW: Build rectangular dual directly from RegularEdgeLabeling
    // This reads explicit outgoing half-edges and interprets:
    //   BLUE  => horizontal constraint (left -> right)
    //   RED   => vertical constraint (bottom -> top)
    bool initializeFromREL(const RegularEdgeLabeling &rel, double cell_size = 40.0);

    std::size_t size() const noexcept { return rects.size(); }
    const Rect &getRect(std::uint32_t id) const;
    const std::vector<Rect> &rectangles() const noexcept { return rects; }

private:
    // helper functions (implemented in cpp)
    bool buildDAGsFromRELmap(const RELmap &rel);
    bool buildDAGsFromRegularEdgeLabeling(const RegularEdgeLabeling &rel);

    bool topoSort(const std::vector<std::vector<std::uint32_t>> &adj,
                  std::vector<std::uint32_t> &order) const;

    // packHorizontal / packVertical compute integer indices;
    // the semantics here: leftIndex[v] = integer column index for v;
    // bottomIndex[v] = integer row index for v.
    // These are simple longest-path packers; they do not currently produce multi-column spans.
    void packHorizontal(const std::vector<std::vector<std::uint32_t>> &adj,
                        const std::vector<std::uint32_t> &topo,
                        std::vector<int> &leftIndex, int &maxRight) const;

    void packVertical(const std::vector<std::vector<std::uint32_t>> &adj,
                      const std::vector<std::uint32_t> &topo,
                      std::vector<int> &bottomIndex, int &maxTop) const;

    STGraph G1;
    STGraph G2;

    // adjacency lists (size = number of rectangles/vertices)
    // horAdj: left -> right edges (blue)
    // verAdj: bottom -> top edges (red)
    std::vector<std::vector<std::uint32_t>> horAdj;
    std::vector<std::vector<std::uint32_t>> verAdj;

    std::vector<Rect> rects;
};