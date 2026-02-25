// rectangular_dual.cpp
#include "rectangular_dual.h"

// include the new RegularEdgeLabeling header so we can build from it
#include "regular_edge_labeling.h"

// include legacy RELmap header if present (kept optional)
#include "rel_map.h"

#include <queue>
#include <iostream>
#include <limits>
#include <algorithm>

using namespace std;

// ---------- accessors ----------
const RectangularDual::Rect &RectangularDual::getRect(std::uint32_t id) const {
    if (id >= rects.size()) throw std::out_of_range("RectangularDual::getRect: id out of range");
    return rects[id];
}

// ---------- legacy: buildDAGsFromRELmap (kept if you still need it) ----------
bool RectangularDual::buildDAGsFromRELmap(const RELmap &rel) {
    // This function reconstructs horAdj and verAdj from RELmap
    // It's optional; if you have an existing implementation, call that instead.
    // For safety, we assume RELmap provides:
    //   rel.size(), rel.get(i).blue_out (vector<uint32_t>), rel.get(i).red_out (vector<uint32_t>)
    size_t n = rel.size();
    horAdj.clear(); horAdj.resize(n);
    verAdj.clear(); verAdj.resize(n);

    for (std::uint32_t u = 0; u < n; ++u) {
        const auto &r = rel.get(u);
        // blue_out: left -> right
        for (auto v : r.blue_out) {
            if (v < n) horAdj[u].push_back(v);
        }
        // red_out: bottom -> top
        for (auto v : r.red_out) {
            if (v < n) verAdj[u].push_back(v);
        }
    }
    return true;
}

bool RectangularDual::buildDAGsFromRegularEdgeLabeling(const RegularEdgeLabeling &rel) {
    const auto &verts = rel.getVertices();
    const auto &halfedges = rel.getHalfEdges();
    size_t n = verts.size();
    if (n == 0) return false;
    horAdj.clear(); horAdj.resize(n);
    verAdj.clear(); verAdj.resize(n);

    // iterate explicit outgoing halfedges and classify by color
    for (int hi = 0; hi < (int)halfedges.size(); ++hi) {
        const auto &h = halfedges[hi];
        if (!h.outgoing) continue;
        //if (!h.is_explicit) continue;
        int u = h.vertex;
        if (h.twin < 0 || h.twin >= (int)halfedges.size()) continue;
        int v = halfedges[h.twin].vertex;
        if (u < 0 || v < 0 || u >= (int)n || v >= (int)n) continue;
        if (h.color == BLUE) {
            horAdj[u].push_back((uint32_t)v); // left -> right
        } else if (h.color == RED) {
            verAdj[u].push_back((uint32_t)v); // bottom -> top
        } else {
            // ignore other colors in layout
        }
    }
    return true;
}

static bool auto_detect_exterior(const RegularEdgeLabeling &rel, std::array<int, 4> &outIndices) {
    outIndices = { -1, -1, -1, -1 };
    const auto &vertices = rel.getVertices();
    for (int i = 0; i < vertices.size(); ++i) {
        const std::string &label = vertices[i].label;

        if (label == "South") {
            outIndices[0] = i;
            continue;
        }
        if (label == "West") {
            outIndices[1] = i;
            continue;
        }
        if (label == "North") {
            outIndices[2] = i;
            continue;
        }
        if (label == "East") {
            outIndices[3] = i;
            continue;
        }
    }
    return (outIndices[0] >= 0 && outIndices[1] >= 0 && outIndices[2] >= 0 && outIndices[3] >= 0);
}


bool RectangularDual::buildSTGraphsFromREL(const RegularEdgeLabeling &rel) {
    const auto &relVertices = rel.getVertices();
    const auto &relHalfedges = rel.getHalfEdges();

    const int n = (int)relVertices.size();
    if (n == 0) return false;

    std:array<int, 4> extIndices{-1, -1, -1, -1};
    if (!auto_detect_exterior(rel, extIndices)) {
        std::cerr << "Failed to detect exterior edges. Make sure 'South', 'West', 'North' and 'East' node are defined in the data. " << std::endl;
        return false;
    }

    // validate exterior indices
    for (int k = 0; k < 4; ++k) {
        if (extIndices[k] < 0 || extIndices[k] >= n) {
            std::cerr << "RectangularDual::buildSTGraphsFromREL: invalid exterior index\n";
            return false;
        }
    }

    const int vS = extIndices[0];
    const int vW = extIndices[1];
    const int vN = extIndices[2];
    const int vE = extIndices[3];

    // initialize graphs (n vertices)
    G1.out.assign(n, {});
    G1.in.assign(n, {});
    G2.out.assign(n, {});
    G2.in.assign(n, {});
    G1.source = vS;
    G1.sink = vN;
    G2.source = vW;
    G2.sink = vE;

    auto is_exterior_pair = [&](const int u, const int v) -> bool {
        if (u < 0 || v < 0) return false;
        // set membership test: both u and v are one of the four exterior indices
        return ( (u==vS||u==vW||u==vN||u==vE) &&
                 (v==vS||v==vW||v==vN||v==vE) );
    };

    for (int i = 0; i < relHalfedges.size(); ++i) {
        const auto &h = relHalfedges[i];
        if (!h.outgoing) continue;;
        int u = h.vertex;
        int t = h.twin;

        if (u < 0 || u >= n) continue;
        if (t < 0 || t >= relHalfedges.size()) continue;
        int v = relHalfedges[t].vertex;
        if (v < 0 || v >= n) continue;

        if (is_exterior_pair(u, v)) continue;

        if (h.color == RED) {
            G1.out[u].push_back(v);
            G2.out[v].push_back(v);
        }
        else if (h.color == BLUE) {
            G2.out[u].push_back(v);
            G1.in[v].push_back(u);
        }
    }

    // Add the four exterior edges
    // For G1 (edges in T1 plus exterior):
    //    vS -> vW,  vS -> vE,  vW -> vN,  vE -> vN
    //
    // For G2 (edges in T2 plus exterior):
    //    vS -> vW,  vN -> vW,  vE -> vS,  vE -> vN
    auto add_edge = [&](STGraph &G, const int a, const int b) {
        G.out[a].push_back(b);
        G.in[b].push_back(a);
    };

    // add G1 exterior edges
    add_edge(G1, vS, vW);
    add_edge(G1, vS, vE);
    add_edge(G1, vW, vN);
    add_edge(G1, vE, vN);

    // add G2 exterior edges
    add_edge(G2, vS, vW);
    add_edge(G2, vN, vW);
    add_edge(G2, vE, vS);
    add_edge(G2, vE, vN);


    return true;
}

bool RectangularDual::initializeFromREL(const RELmap &rel, double cell_size) {
    // preserve old behaviour: build DAGs from RELmap, topo sort, pack, build rects
    if (!buildDAGsFromRELmap(rel)) return false;

    // topo sort
    vector<uint32_t> topoH, topoV;
    if (!topoSort(horAdj, topoH)) {
        cerr << "RectangularDual::initializeFromREL: horizontal graph contains a cycle\n";
        return false;
    }
    if (!topoSort(verAdj, topoV)) {
        cerr << "RectangularDual::initializeFromREL: vertical graph contains a cycle\n";
        return false;
    }

    vector<int> leftIndex(horAdj.size(), 0);
    int maxRight = 0;
    packHorizontal(horAdj, topoH, leftIndex, maxRight);

    vector<int> bottomIndex(verAdj.size(), 0);
    int maxTop = 0;
    packVertical(verAdj, topoV, bottomIndex, maxTop);

    // build unit-cell rectangles
    rects.clear();
    rects.resize(horAdj.size());
    for (size_t i = 0; i < horAdj.size(); ++i) {
        Rect r;
        r.left = static_cast<double>(leftIndex[i]) * cell_size;
        r.right = static_cast<double>(leftIndex[i] + 1) * cell_size;
        r.bottom = static_cast<double>(bottomIndex[i]) * cell_size;
        r.top = static_cast<double>(bottomIndex[i] + 1) * cell_size;
        rects[i] = r;
    }
    return true;
}

bool RectangularDual::initializeFromREL(const RegularEdgeLabeling &rel, double cell_size) {
    if (!buildDAGsFromRegularEdgeLabeling(rel)) return false;

    // topo sort
    vector<uint32_t> topoH, topoV;
    if (!topoSort(horAdj, topoH)) {
        cerr << "RectangularDual::initializeFromREL: horizontal graph contains a cycle\n";
        return false;
    }
    if (!topoSort(verAdj, topoV)) {
        cerr << "RectangularDual::initializeFromREL: vertical graph contains a cycle\n";
        return false;
    }

    vector<int> leftIndex(horAdj.size(), 0);
    int maxRight = 0;
    packHorizontal(horAdj, topoH, leftIndex, maxRight);

    vector<int> bottomIndex(verAdj.size(), 0);
    int maxTop = 0;
    packVertical(verAdj, topoV, bottomIndex, maxTop);

    // build unit-cell rectangles
    rects.clear();
    rects.resize(horAdj.size());
    for (size_t i = 0; i < horAdj.size(); ++i) {
        Rect r;
        r.left = static_cast<double>(leftIndex[i]) * cell_size;
        r.right = static_cast<double>(leftIndex[i] + 1) * cell_size;
        r.bottom = static_cast<double>(bottomIndex[i]) * cell_size;
        r.top = static_cast<double>(bottomIndex[i] + 1) * cell_size;
        rects[i] = r;
    }
    return true;
}

// ---------- topoSort: Kahn (returns false on cycle) ----------
bool RectangularDual::topoSort(const std::vector<std::vector<std::uint32_t>> &adj,
                               std::vector<std::uint32_t> &order) const {
    const size_t n = adj.size();
    order.clear();
    order.reserve(n);
    vector<int> indeg(n, 0);
    for (size_t u = 0; u < n; ++u) {
        for (auto v : adj[u]) if (v < n) ++indeg[v];
    }
    queue<uint32_t> q;
    for (uint32_t i = 0; i < n; ++i) if (indeg[i] == 0) q.push(i);
    while (!q.empty()) {
        uint32_t u = q.front(); q.pop();
        order.push_back(u);
        for (auto v : adj[u]) {
            if (v >= n) continue;
            if (--indeg[v] == 0) q.push(v);
        }
    }
    return order.size() == n;
}

// ---------- packHorizontal / packVertical (longest-path DP on DAG) ----------
void RectangularDual::packHorizontal(const std::vector<std::vector<std::uint32_t>> &adj,
                                     const std::vector<std::uint32_t> &topo,
                                     std::vector<int> &leftIndex, int &maxRight) const {
    const int n = static_cast<int>(adj.size());
    leftIndex.assign(n, 0);
    maxRight = 0;
    // process vertices in topo order: leftIndex[v] = max(leftIndex[u]+1 for u->v)
    for (auto v : topo) {
        int best = leftIndex[v];
        for (auto w : adj[v]) {
            if ((int)w < n) {
                // relax neighbor w from v: neighbor must be at least v+1
                if (leftIndex[w] < leftIndex[v] + 1) leftIndex[w] = leftIndex[v] + 1;
            }
        }
        if (leftIndex[v] > maxRight) maxRight = leftIndex[v];
    }
}

void RectangularDual::packVertical(const std::vector<std::vector<std::uint32_t>> &adj,
                                   const std::vector<std::uint32_t> &topo,
                                   std::vector<int> &bottomIndex, int &maxTop) const {
    const int n = static_cast<int>(adj.size());
    bottomIndex.assign(n, 0);
    maxTop = 0;
    for (auto v : topo) {
        for (auto w : adj[v]) {
            if ((int)w < n) {
                if (bottomIndex[w] < bottomIndex[v] + 1) bottomIndex[w] = bottomIndex[v] + 1;
            }
        }
        if (bottomIndex[v] > maxTop) maxTop = bottomIndex[v];
    }
}