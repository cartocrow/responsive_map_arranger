// rectangular_dual.h -- (excerpt)
#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <optional>
#include "rel_map.h"

class RectangularDual {
public:
    struct Rect {
        double left;
        double right;
        double bottom;
        double top;
    };

    RectangularDual() = default;

    // Build rectangular dual from RELmap.
    // cell_size: scale for each grid cell (visual size).
    bool initializeFromREL(const RELmap &rel, double cell_size = 40.0);

    std::size_t size() const noexcept { return rects.size(); }
    const Rect &getRect(std::uint32_t id) const;
    const std::vector<Rect> &rectangles() const noexcept { return rects; }

private:
    bool buildDAGs(const RELmap &rel);
    bool topoSort(const std::vector<std::vector<std::uint32_t>> &adj,
                  std::vector<std::uint32_t> &order) const;

    void packHorizontal(const std::vector<std::vector<std::uint32_t>> &adj,
                        const std::vector<std::uint32_t> &topo,
                        std::vector<int> &leftIndex, int &maxRight) const;

    void packVertical(const std::vector<std::vector<std::uint32_t>> &adj,
                      const std::vector<std::uint32_t> &topo,
                      std::vector<int> &bottomIndex, int &maxTop) const;

    // Note: blue -> horizontal, red -> vertical in this implementation
    std::vector<std::vector<std::uint32_t>> horAdj; // from blue_out: left -> right
    std::vector<std::vector<std::uint32_t>> verAdj; // from red_out: bottom -> top

    std::vector<Rect> rects;
};