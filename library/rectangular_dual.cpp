// rectangular_dual.cpp
#include "rectangular_dual.h"

#include <queue>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>
#include <optional>

bool RectangularDual::initializeFromREL(const RELmap &rel, double cell_size) {
    if (!buildDAGs(rel)) {
        std::cerr << "RectangularDual: failed to build DAGs (size mismatch?)\n";
        return false;
    }

    const std::size_t n = horAdj.size();
    rects.clear();
    rects.resize(n);

    // Topological sort horizontal (horAdj)
    std::vector<std::uint32_t> topoH;
    if (!topoSort(horAdj, topoH)) {
        std::cerr << "RectangularDual: horizontal (blue) graph has cycle\n";
        return false;
    }
    // Topological sort vertical (verAdj)
    std::vector<std::uint32_t> topoV;
    if (!topoSort(verAdj, topoV)) {
        std::cerr << "RectangularDual: vertical (red) graph has cycle\n";
        return false;
    }

    // Pack to compact integer grid
    std::vector<int> leftIndex(n, 0), bottomIndex(n, 0);
    int maxRight = 0, maxTop = 0;

    packHorizontal(horAdj, topoH, leftIndex, maxRight); // blue -> horizontal
    packVertical(verAdj, topoV, bottomIndex, maxTop);   // red -> vertical

    // Now reserve frame rows/columns and shift other nodes to avoid overlap.
    // Find frame IDs if present
    std::optional<std::uint32_t> idWest, idEast, idNorth, idSouth;
    for (std::uint32_t i = 0; i < n; ++i) {
        const auto &lab = rel.get(i).label;
        if (lab == "West") idWest = i;
        else if (lab == "East") idEast = i;
        else if (lab == "North") idNorth = i;
        else if (lab == "South") idSouth = i;
    }

    // Reserve left column for West (shift all others right by 1)
    if (idWest) {
        const std::uint32_t wid = *idWest;
        for (std::uint32_t i = 0; i < n; ++i) {
            if (i == wid) continue;
            leftIndex[i] += 1;
        }
        leftIndex[wid] = 0;
        maxRight += 1;
    }

    // Place East at rightmost *new* column (append)
    if (idEast) {
        const std::uint32_t eid = *idEast;
        leftIndex[eid] = maxRight; // append as new rightmost
        maxRight += 1;
    }

    // Reserve bottom row for South (shift all others up by 1)
    if (idSouth) {
        const std::uint32_t sid = *idSouth;
        for (std::uint32_t i = 0; i < n; ++i) {
            if (i == sid) continue;
            bottomIndex[i] += 1;
        }
        bottomIndex[sid] = 0;
        maxTop += 1;
    }

    // Place North at the topmost appended row
    if (idNorth) {
        const std::uint32_t nid = *idNorth;
        bottomIndex[nid] = maxTop; // append as new topmost index
        maxTop += 1;
    }

    // After these shifts maxRight and maxTop represent number of columns/rows
    // If neither frame was present, pack* already computed correct maxima; but if we didn't update maxRight/maxTop above,
    // ensure they are set:
    if (maxRight == 0) {
        // find max right from leftIndex + 1
        int mr = 0;
        for (int v : leftIndex) mr = std::max(mr, v + 1);
        maxRight = mr;
    }
    if (maxTop == 0) {
        int mt = 0;
        for (int v : bottomIndex) mt = std::max(mt, v + 1);
        maxTop = mt;
    }

    // Build rectangles scaled by cell_size
    // Special-case frame nodes to span full vertical/horizontal as requested:
    for (std::size_t id = 0; id < n; ++id) {
        // handle frame nodes specially
        if (idWest && id == *idWest) {
            double left = 0.0;
            double right = 1.0 * cell_size;
            double bottom = 0.0;
            double top = static_cast<double>(maxTop) * cell_size;
            rects[id] = Rect{left, right, bottom, top};
            continue;
        }
        if (idEast && id == *idEast) {
            double left = static_cast<double>(maxRight - 1) * cell_size;
            double right = static_cast<double>(maxRight) * cell_size;
            double bottom = 0.0;
            double top = static_cast<double>(maxTop) * cell_size;
            rects[id] = Rect{left, right, bottom, top};
            continue;
        }
        if (idSouth && id == *idSouth) {
            double left = 0.0;
            double right = static_cast<double>(maxRight) * cell_size;
            double bottom = 0.0;
            double top = 1.0 * cell_size;
            rects[id] = Rect{left, right, bottom, top};
            continue;
        }
        if (idNorth && id == *idNorth) {
            double left = 0.0;
            double right = static_cast<double>(maxRight) * cell_size;
            double bottom = static_cast<double>(maxTop - 1) * cell_size;
            double top = static_cast<double>(maxTop) * cell_size;
            rects[id] = Rect{left, right, bottom, top};
            continue;
        }

        // default: unit cell at computed indices
        double left = static_cast<double>(leftIndex[id]) * cell_size;
        double right = static_cast<double>(leftIndex[id] + 1) * cell_size;
        double bottom = static_cast<double>(bottomIndex[id]) * cell_size;
        double top = static_cast<double>(bottomIndex[id] + 1) * cell_size;
        rects[id] = Rect{left, right, bottom, top};
    }

    return true;
}

bool RectangularDual::buildDAGs(const RELmap &rel) {
    const std::size_t n = rel.size();
    if (n == 0) return false;
    // **Important:** blue_out -> horizontal (left->right), red_out -> vertical (bottom->top)
    horAdj.assign(n, {});
    verAdj.assign(n, {});
    for (std::uint32_t u = 0; u < n; ++u) {
        // blue_out interpreted as left -> right edges
        const auto &outsB = rel.get(u).blue_out;
        for (auto v : outsB) {
            if (v >= n) continue;
            horAdj[u].push_back(v);
        }
        // red_out interpreted as bottom -> top edges
        const auto &outsR = rel.get(u).red_out;
        for (auto v : outsR) {
            if (v >= n) continue;
            verAdj[u].push_back(v);
        }
    }
    return true;
}

// Standard Kahn topo sort
bool RectangularDual::topoSort(const std::vector<std::vector<std::uint32_t>> &adj,
                               std::vector<std::uint32_t> &order) const {
    const std::size_t n = adj.size();
    order.clear();
    order.reserve(n);
    std::vector<int> indeg(n, 0);
    for (std::size_t u = 0; u < n; ++u) {
        for (auto v : adj[u]) if (v < n) indeg[v]++;
    }
    std::queue<std::uint32_t> q;
    for (std::size_t i = 0; i < n; ++i) if (indeg[i] == 0) q.push(static_cast<std::uint32_t>(i));
    while (!q.empty()) {
        auto u = q.front(); q.pop();
        order.push_back(u);
        for (auto v : adj[u]) {
            if (v >= n) continue;
            --indeg[v];
            if (indeg[v] == 0) q.push(v);
        }
    }
    return order.size() == n;
}

// Pack horizontally: for each v, left[v] = max_u ( left[u] + width(u) ), using unit widths
void RectangularDual::packHorizontal(const std::vector<std::vector<std::uint32_t>> &adj,
                                     const std::vector<std::uint32_t> &topo,
                                     std::vector<int> &leftIndex, int &maxRight) const {
    const std::size_t n = adj.size();
    leftIndex.assign(n, 0);

    // build preds
    std::vector<std::vector<std::uint32_t>> pred(n);
    for (std::size_t u = 0; u < n; ++u)
        for (auto v : adj[u]) if (v < n) pred[v].push_back(static_cast<std::uint32_t>(u));

    maxRight = 0;
    for (auto v : topo) {
        int best = 0;
        for (auto u : pred[v]) {
            int candidate = leftIndex[u] + 1; // unit width
            if (candidate > best) best = candidate;
        }
        leftIndex[v] = best;
        maxRight = std::max(maxRight, leftIndex[v] + 1);
    }
}

// Pack vertically similarly (bottom indices)
void RectangularDual::packVertical(const std::vector<std::vector<std::uint32_t>> &adj,
                                   const std::vector<std::uint32_t> &topo,
                                   std::vector<int> &bottomIndex, int &maxTop) const {
    const std::size_t n = adj.size();
    bottomIndex.assign(n, 0);

    std::vector<std::vector<std::uint32_t>> pred(n);
    for (std::size_t u = 0; u < n; ++u)
        for (auto v : adj[u]) if (v < n) pred[v].push_back(static_cast<std::uint32_t>(u));

    maxTop = 0;
    for (auto v : topo) {
        int best = 0;
        for (auto u : pred[v]) {
            int candidate = bottomIndex[u] + 1;
            if (candidate > best) best = candidate;
        }
        bottomIndex[v] = best;
        maxTop = std::max(maxTop, bottomIndex[v] + 1);
    }
}

const RectangularDual::Rect &RectangularDual::getRect(std::uint32_t id) const {
    if (id >= rects.size()) throw std::out_of_range("RectangularDual::getRect: id out of range");
    return rects[id];
}