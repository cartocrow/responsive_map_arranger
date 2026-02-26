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

// small DSU
struct DSU {
    std::vector<int> p, r;
    DSU(int n = 0) { init(n); }
    void init(int n) { p.resize(n); r.assign(n, 0); std::iota(p.begin(), p.end(), 0); }
    int find(int a) {
        if (p[a] == a) return a;
        return p[a] = find(p[a]);
    }
    void unite(int a, int b) {
        int ra = find(a), rb = find(b);
        if (ra == rb) return;
        if (r[ra] < r[rb]) p[ra] = rb;
        else if (r[rb] < r[ra]) p[rb] = ra;
        else { p[rb] = ra; r[ra]++; }
    }
};

bool RectangularDual::computeMaximalSegments(RegularEdgeLabeling &rel) {
    const auto &relHalfedges = rel.getHalfEdges();
    const auto &relVertices  = rel.getVertices();
    const int H = (int)relHalfedges.size();
    const int V = (int)relVertices.size();
    if (H == 0) { std::cerr << "computeMaximalSegments: no halfedges\n"; return false; }

    // small DSU (local copy)
    struct DSUlocal {
        std::vector<int> p, r;
        void init(int n) { p.resize(n); r.assign(n,0); std::iota(p.begin(), p.end(), 0); }
        int find(int a){ return p[a]==a ? a : p[a]=find(p[a]); }
        void unite(int a,int b){ int ra=find(a), rb=find(b); if(ra==rb) return;
                                if(r[ra]<r[rb]) p[ra]=rb;
                                else if(r[rb]<r[ra]) p[rb]=ra;
                                else { p[rb]=ra; r[ra]++; } }
    };
    DSUlocal dsu; dsu.init(H);

    // 1) union half-edge with twin (if present)
    for (int h = 0; h < H; ++h) {
        int t = relHalfedges[h].twin;
        if (t >= 0 && t < H) dsu.unite(h, t);
    }

    // 2) union consecutive half-edges at each vertex when color+direction match
    // create a repaired local copy of incident lists to ensure the half-edge belongs to vertex
    std::vector<std::vector<int>> vertexInc(V);
    for (int v = 0; v < V; ++v) vertexInc[v] = relVertices[v].edges;

    // repair incident entries: if an entry is not actually attached to this vertex, try its twin
    for (int v = 0; v < V; ++v) {
        for (int &he : vertexInc[v]) {
            if (he < 0 || he >= H) continue;
            if (relHalfedges[he].vertex == v) continue;
            int t = relHalfedges[he].twin;
            if (t >= 0 && t < H && relHalfedges[t].vertex == v) { he = t; continue; }
            // otherwise leave it (we'll skip invalid he later)
            std::cerr << "computeMaximalSegments: incident halfedge " << he << " at vertex " << v
                      << " not attached to vertex (owner=" << relHalfedges[he].vertex << ")\n";
        }
    }

    for (int v = 0; v < V; ++v) {
        const auto &inc = vertexInc[v];
        int deg = (int)inc.size();
        if (deg <= 1) continue;
        for (int i = 0; i < deg; ++i) {
            int j = (i + 1) % deg;
            int he_i = inc[i];
            int he_j = inc[j];
            if (he_i < 0 || he_i >= H || he_j < 0 || he_j >= H) continue;
            const HalfEdge &hi = relHalfedges[he_i];
            const HalfEdge &hj = relHalfedges[he_j];
            if (hi.color == hj.color && hi.outgoing == hj.outgoing) dsu.unite(he_i, he_j);
        }
    }

    // --- Critical additional step: enforce that any explicit outgoing half-edge and its twin are in same component.
    // This ensures every REL edge maps to a single segment.
    bool mergedAny = true;
    int iter = 0;
    while (mergedAny && iter < H+5) {
        mergedAny = false;
        ++iter;
        for (int h = 0; h < H; ++h) {
            const HalfEdge &hh = relHalfedges[h];
            if (!hh.outgoing) continue; // only explicit outgoing half-edges represent input edges
            int t = hh.twin;
            if (t < 0 || t >= H) continue;
            int r1 = dsu.find(h);
            int r2 = dsu.find(t);
            if (r1 != r2) { dsu.unite(r1, r2); mergedAny = true; }
        }
    }
    if (iter >= H+5) {
        std::cerr << "computeMaximalSegments: adjacency-enforcement iter exceeded\n";
    }

    // 3) gather components into a map root -> members
    std::unordered_map<int, std::vector<int>> comps;
    comps.reserve(H);
    for (int h = 0; h < H; ++h) comps[ dsu.find(h) ].push_back(h);

    // 4) deterministic order of roots (sort keys)
    std::vector<int> roots;
    roots.reserve(comps.size());
    for (auto &kv : comps) roots.push_back(kv.first);
    std::sort(roots.begin(), roots.end());

    // 5) build segments from components
    maximalSegments.clear();
    maximalSegments.reserve(roots.size());
    std::vector<int> he_to_seg(H, -1);

    for (int root : roots) {
        auto membersIt = comps.find(root);
        if (membersIt == comps.end()) continue;
        std::vector<int> members = membersIt->second; // copy (safe)

        Segment seg;
        seg.type = SEGMENT_UNKNOWN;
        seg.halfedges = std::move(members);

        // counts
        // simpler classification: any BLUE -> horizontal, else any RED -> vertical, else unknown
        bool seenBlue = false, seenRed = false;
        for (int he : seg.halfedges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = relHalfedges[he];
            if (h.color == BLUE) seenBlue = true;
            else if (h.color == RED) seenRed = true;
        }
        if (seenBlue) seg.type = SEGMENT_HORIZONTAL;
        else if (seenRed) seg.type = SEGMENT_VERTICAL;
        else seg.type = SEGMENT_UNKNOWN;

        // fill side vertex sets using outgoing half-edges as canonical origin->dest semantics
        std::unordered_set<int> leftSet, rightSet, bottomSet, topSet;
        for (int he : seg.halfedges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = relHalfedges[he];
            int twin = h.twin;
            int dest = (twin >= 0 && twin < H) ? relHalfedges[twin].vertex : -1;
            if (seg.type == SEGMENT_HORIZONTAL) {
                if (h.color == BLUE && h.outgoing) {
                    if (h.vertex >= 0) leftSet.insert(h.vertex);
                    if (dest >= 0) rightSet.insert(dest);
                }
            } else if (seg.type == SEGMENT_VERTICAL) {
                if (h.color == RED && h.outgoing) {
                    if (h.vertex >= 0) bottomSet.insert(h.vertex);
                    if (dest >= 0) topSet.insert(dest);
                }
            } else {
                if (h.vertex >= 0) seg.incoming_vertices.push_back(h.vertex);
                if (dest >= 0) seg.outgoing_vertices.push_back(dest);
            }
        }

        if (seg.type == SEGMENT_HORIZONTAL) {
            seg.incoming_vertices.assign(leftSet.begin(), leftSet.end());
            seg.outgoing_vertices.assign(rightSet.begin(), rightSet.end());
            std::sort(seg.incoming_vertices.begin(), seg.incoming_vertices.end());
            std::sort(seg.outgoing_vertices.begin(), seg.outgoing_vertices.end());
        } else if (seg.type == SEGMENT_VERTICAL) {
            seg.incoming_vertices.assign(bottomSet.begin(), bottomSet.end());
            seg.outgoing_vertices.assign(topSet.begin(), topSet.end());
            std::sort(seg.incoming_vertices.begin(), seg.incoming_vertices.end());
            std::sort(seg.outgoing_vertices.begin(), seg.outgoing_vertices.end());
        } else {
            std::sort(seg.incoming_vertices.begin(), seg.incoming_vertices.end());
            seg.incoming_vertices.erase(std::unique(seg.incoming_vertices.begin(), seg.incoming_vertices.end()), seg.incoming_vertices.end());
            std::sort(seg.outgoing_vertices.begin(), seg.outgoing_vertices.end());
            seg.outgoing_vertices.erase(std::unique(seg.outgoing_vertices.begin(), seg.outgoing_vertices.end()), seg.outgoing_vertices.end());
        }

        // write he->segment mapping
        int segIndex = (int)maximalSegments.size();
        for (int he : seg.halfedges) {
            if (he >= 0 && he < H) he_to_seg[he] = segIndex;
            else std::cerr << "computeMaximalSegments: invalid he in segment: " << he << "\n";
        }

        maximalSegments.push_back(std::move(seg));
    }

    // 6) set per-vertex left/right/top/bottom indices
    for (int v = 0; v < V; ++v) {
    int leftSeg = -1, rightSeg = -1, bottomSeg = -1, topSeg = -1;

    // Horizontal: left = segment of first incoming BLUE, right = segment of first outgoing BLUE
    int he_left = rel.getFirstIncomingBlue(v);   // returns half-edge index at v (incoming BLUE)
    if (he_left >= 0 && he_left < H) leftSeg = he_to_seg[he_left];

    int he_right = rel.getFirstOutgoingBlue(v);  // returns half-edge index at v (outgoing BLUE)
    if (he_right >= 0 && he_right < H) rightSeg = he_to_seg[he_right];

    // Vertical: bottom = segment of first incoming RED, top = segment of first outgoing RED
    int he_bottom = rel.getFirstIncomingRed(v);
    if (he_bottom >= 0 && he_bottom < H) bottomSeg = he_to_seg[he_bottom];

    int he_top = rel.getFirstOutgoingRed(v);
    if (he_top >= 0 && he_top < H) topSeg = he_to_seg[he_top];

    // Fallbacks: if any side still -1, try last-edge variants (cases where run wraps around)
    if (leftSeg == -1) {
        int he = rel.getlastIncomingBlue(v);
        if (he >= 0 && he < H) leftSeg = he_to_seg[he];
    }
    if (rightSeg == -1) {
        int he = rel.getlastOutgoingBlue(v);
        if (he >= 0 && he < H) rightSeg = he_to_seg[he];
    }
    if (bottomSeg == -1) {
        int he = rel.getlastIncomingRed(v);
        if (he >= 0 && he < H) bottomSeg = he_to_seg[he];
    }
    if (topSeg == -1) {
        int he = rel.getlastOutgoingRed(v);
        if (he >= 0 && he < H) topSeg = he_to_seg[he];
    }

    // Final fallback: if still missing, pick any candidate deterministically (min) — preserves robustness.
    if (leftSeg == -1) {
        // find any incoming BLUE half-edge and use its segment
        for (int he : rel.getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = rel.getHalfEdges()[he];
            if (h.color == BLUE && !h.outgoing) { leftSeg = he_to_seg[he]; break; }
        }
    }
    if (rightSeg == -1) {
        for (int he : rel.getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = rel.getHalfEdges()[he];
            if (h.color == BLUE && h.outgoing) { rightSeg = he_to_seg[he]; break; }
        }
    }
    if (bottomSeg == -1) {
        for (int he : rel.getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = rel.getHalfEdges()[he];
            if (h.color == RED && !h.outgoing) { bottomSeg = he_to_seg[he]; break; }
        }
    }
    if (topSeg == -1) {
        for (int he : rel.getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = rel.getHalfEdges()[he];
            if (h.color == RED && h.outgoing) { topSeg = he_to_seg[he]; break; }
        }
    }

    // write safe-to-call setter
    rel.setVertexSegmentIndices(v, leftSeg, rightSeg, bottomSeg, topSeg);
}

    // debug summary
    // std::cout << "computeMaximalSegments: " << maximalSegments.size() << " segments\n";
    // for (size_t i = 0; i < maximalSegments.size(); ++i) {
    //     const auto &s = maximalSegments[i];
    //     std::cout << " segment " << i << " type=" << static_cast<int>(s.type)
    //               << " halfedges=" << s.halfedges.size()
    //               << " incoming verts=" << s.incoming_vertices.size()
    //               << " outgoing verts=" << s.outgoing_vertices.size() << "\n";
    // }

    for (size_t si = 0; si < maximalSegments.size(); ++si) {
        const auto &s = maximalSegments[si];
        if (s.halfedges.empty()) {
            std::cerr << "Warning: segment " << si << " has zero halfedges\n";
        }
        for (int he : s.halfedges) {
            if (he < 0 || he >= (int)rel.getHalfEdges().size()) {
                std::cerr << "ERROR: segment " << si << " contains invalid halfedge index " << he << "\n";
            }
        }
    }

    return true;
}

bool RectangularDual::computeSegmentPositions(const RegularEdgeLabeling &rel, double cell_size) {
    // build compact index maps for existing segments
    const int S = (int)maximalSegments.size();
    int hcount = 0, vcount = 0;
    std::vector<int> seg_to_hidx(S, -1), seg_to_vidx(S, -1);
    for (int i = 0; i < S; ++i) {
        if (maximalSegments[i].type == SEGMENT_HORIZONTAL) seg_to_hidx[i] = hcount++;
        else if (maximalSegments[i].type == SEGMENT_VERTICAL) seg_to_vidx[i] = vcount++;
    }

    // create frame nodes (we'll append them after existing segments by concept)
    // We'll treat them as separate indices in the adjacency arrays.
    const int leftFrameIdx = hcount;    // index in horizontal node-space
    const int rightFrameIdx = hcount+1;
    const int bottomFrameIdx = vcount;  // index in vertical node-space
    const int topFrameIdx = vcount+1;

    // adjacency lists sized to include frame nodes
    std::vector<std::vector<std::uint32_t>> horAdj(hcount + 2);
    std::vector<std::vector<std::uint32_t>> verAdj(vcount + 2);

    const auto &verts = rel.getVertices();
    const int V = (int)verts.size();

    // helper to map possibly -1 segment id to horizontal/vertical node index
    auto hnode = [&](int segId)->int {
        if (segId < 0) return -1;
        if (segId >= 0 && segId < S) return seg_to_hidx[segId];
        return -1;
    };
    auto vnode = [&](int segId)->int {
        if (segId < 0) return -1;
        if (segId >= 0 && segId < S) return seg_to_vidx[segId];
        return -1;
    };

    // Build edges. If left or right is -1, map to the corresponding frame node.
    for (int v = 0; v < V; ++v) {
        int leftSeg = verts[v].left_segment;
        int rightSeg = verts[v].right_segment;
        int bottomSeg = verts[v].bottom_segment;
        int topSeg = verts[v].top_segment;

        // horizontal: left -> right
        int a = hnode(leftSeg);
        int b = hnode(rightSeg);
        if (leftSeg == -1 && rightSeg == -1) {
            // touches both frame? connect leftFrame -> rightFrame
            horAdj[leftFrameIdx].push_back((std::uint32_t)rightFrameIdx);
        } else if (leftSeg == -1 && b >= 0) {
            horAdj[leftFrameIdx].push_back((std::uint32_t)b);
        } else if (rightSeg == -1 && a >= 0) {
            horAdj[a].push_back((std::uint32_t)rightFrameIdx);
        } else if (a >= 0 && b >= 0 && a != b) {
            horAdj[a].push_back((std::uint32_t)b);
        }

        // vertical: bottom -> top
        int c = vnode(bottomSeg);
        int d = vnode(topSeg);
        if (bottomSeg == -1 && topSeg == -1) {
            verAdj[bottomFrameIdx].push_back((std::uint32_t)topFrameIdx);
        } else if (bottomSeg == -1 && d >= 0) {
            verAdj[bottomFrameIdx].push_back((std::uint32_t)d);
        } else if (topSeg == -1 && c >= 0) {
            verAdj[c].push_back((std::uint32_t)topFrameIdx);
        } else if (c >= 0 && d >= 0 && c != d) {
            verAdj[c].push_back((std::uint32_t)d);
        }
    }

    // deduplicate edges
    auto dedup = [](std::vector<std::vector<std::uint32_t>> &adj) {
        for (auto &nbrs : adj) {
            if (nbrs.size() <= 1) continue;
            std::sort(nbrs.begin(), nbrs.end());
            nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
        }
    };
    dedup(horAdj);
    dedup(verAdj);

    // Topo-sort with your topoSort helper; it must handle the new arrays (size > 0)
    std::vector<std::uint32_t> htopo, vtopo;
    if (!topoSort(horAdj, htopo)) { std::cerr << "horizontal DAG has cycle\n"; return false; }
    if (!topoSort(verAdj, vtopo)) { std::cerr << "vertical DAG has cycle\n"; return false; }

    // longest-path distances
    std::vector<int> hx(horAdj.size(), 0), vy(verAdj.size(), 0);
    for (auto u : htopo) for (auto w : horAdj[u]) hx[w] = std::max(hx[w], hx[u] + 1);
    for (auto u : vtopo) for (auto w : verAdj[u]) vy[w] = std::max(vy[w], vy[u] + 1);

    // Recover segment coordinates (map back to S-sized arrays)
    std::vector<int> seg_x(S, -1), seg_y(S, -1);
    for (int si = 0; si < S; ++si) {
        if (maximalSegments[si].type == SEGMENT_HORIZONTAL) {
            int hi = seg_to_hidx[si];
            if (hi >= 0) seg_x[si] = hx[hi];
        } else if (maximalSegments[si].type == SEGMENT_VERTICAL) {
            int vi = seg_to_vidx[si];
            if (vi >= 0) seg_y[si] = vy[vi];
        }
    }


    // Now use frame node coords too if needed: leftFrameIdx->hx[leftFrameIdx], rightFrameIdx->...
    int leftFrameX = hx[leftFrameIdx], rightFrameX = hx[rightFrameIdx];
    int bottomFrameY = vy[bottomFrameIdx], topFrameY = vy[topFrameIdx];

    // build rects for each region
    rects.clear(); rects.resize((size_t)V);
    for (int v = 0; v < V; ++v) {
        int lx = 0, rx = 0, by = 0, ty = 0;
        int leftSeg = verts[v].left_segment;
        int rightSeg = verts[v].right_segment;
        int bottomSeg = verts[v].bottom_segment;
        int topSeg = verts[v].top_segment;

        if (leftSeg == -1) lx = leftFrameX;
        else if (leftSeg >= 0 && leftSeg < S) lx = seg_x[leftSeg];
        if (rightSeg == -1) rx = rightFrameX;
        else if (rightSeg >= 0 && rightSeg < S) rx = seg_x[rightSeg];
        if (bottomSeg == -1) by = bottomFrameY;
        else if (bottomSeg >= 0 && bottomSeg < S) by = seg_y[bottomSeg];
        if (topSeg == -1) ty = topFrameY;
        else if (topSeg >= 0 && topSeg < S) ty = seg_y[topSeg];

        // sanity: ensure left < right etc.
        if (lx >= rx) rx = lx + 1;
        if (by >= ty) ty = by + 1;

        Rect r;
        r.left   = double(lx) * cell_size;
        r.right  = double(rx) * cell_size;
        r.bottom = double(by) * cell_size;
        r.top    = double(ty) * cell_size;
        rects[v] = r;
    }

    return true;
}

// debug: dump a segment's halfedges and the half-edge directions
void RectangularDual::debugDumpSegment(int segId, const RegularEdgeLabeling &rel) const {
    if (segId < 0 || segId >= (int)maximalSegments.size()) {
        std::cout << "debugDumpSegment: segId out of range\n"; return;
    }
    const auto &seg = maximalSegments[segId];
    std::cout << "Segment " << segId << " type=" << static_cast<int>(seg.type)
              << " halfedges=" << seg.halfedges.size() << "\n";
    for (int he : seg.halfedges) {
        const auto &h = rel.getHalfEdges()[he];
        int twin = h.twin;
        int dest = (twin >= 0 ? rel.getHalfEdges()[twin].vertex : -1);
        std::cout << " he#" << he
                  << " origin=" << h.vertex << "('" << rel.getVertices()[h.vertex].label << "')"
                  << " -> dest=" << dest
                  << (dest >= 0 ? ("('" + rel.getVertices()[dest].label + "')") : std::string(""))
                  << " color=" << (h.color==BLUE?"BLUE":(h.color==RED?"RED":"UNK"))
                  << " out=" << h.outgoing << "\n";
    }
    std::cout << " incoming verts (side A): ";
    for (int v : seg.incoming_vertices) std::cout << rel.getVertices()[v].label << " ";
    std::cout << "\n outgoing verts (side B): ";
    for (int v : seg.outgoing_vertices) std::cout << rel.getVertices()[v].label << " ";
    std::cout << "\n";
}

// debug: dump vertex candidate segments and chosen ones
void RectangularDual::debugDumpVertexSegments(const RegularEdgeLabeling &rel, int v) const {
    const auto &verts = rel.getVertices();
    const auto &hes  = rel.getHalfEdges();
    std::cout << "Vertex " << v << " '" << verts[v].label << "' incident half-edges (CCW):\n";
    for (int he : verts[v].edges) {
        const auto &h = hes[he];
        int twin = h.twin;
        int dest = (twin >= 0 ? hes[twin].vertex : -1);
        std::cout << " he#" << he << " -> " << dest
                  << " color=" << (h.color==BLUE?"BLUE":(h.color==RED?"RED":"UNK"))
                  << " out=" << h.outgoing << "\n";
    }
    std::cout << "Assigned segments: left=" << verts[v].left_segment
              << " right=" << verts[v].right_segment
              << " bottom=" << verts[v].bottom_segment
              << " top=" << verts[v].top_segment << "\n";
}

bool RectangularDual::buildSTandDUal(const RegularEdgeLabeling &rel) {
    auto one = buildSTGraphsFromREL(rel);

    // Print basic graph info
    std::cout << "G1.source=" << G1.source << " G1.sink=" << G1.sink << " n=" << G1.out.size() << "\n";
    for (int u = 0; u < (int)G1.out.size(); ++u) {
        std::cout << " G1 " << u << " ->";
        for (int v : G1.out[u]) std::cout << " " << v;
        std::cout << "\n";
    }
    std::cout << "G2.source=" << G2.source << " G2.sink=" << G2.sink << " n=" << G2.out.size() << "\n";
    for (int u = 0; u < (int)G2.out.size(); ++u) {
        std::cout << " G2 " << u << " ->";
        for (int v : G2.out[u]) std::cout << " " << v;
        std::cout << "\n";
    }

    // quick validity tests
    auto checkST = [&](const RectangularDual::STGraph &G, const std::string &name) {
        int n = (int)G.out.size();
        std::vector<int> indeg(n, 0);
        for (int u = 0; u < n; ++u) for (int v : G.out[u]) if (v >= 0 && v < n) ++indeg[v];
        int nsrc = 0, nsink = 0;
        for (int i = 0; i < n; ++i) {
            if (indeg[i] == 0) ++nsrc;
            if (G.out[i].empty()) ++nsink;
        }
        std::cout << name << " has " << nsrc << " sources and " << nsink << " sinks\n";
    };
    checkST(G1, "G1");
    checkST(G2, "G2");

    std::cout << "-------------------" << std::endl;
    int gw = G2.source; // should be vW
    int ge = G2.sink;   // should be vE
    std::cerr << "G2.source (vW) index = " << gw << " label=" << rel.getVertices()[gw].label << "\n";
    std::cerr << " G2.in[" << gw << "] =";
    for (int u : G2.in[gw]) std::cerr << " " << u << "('" << rel.getVertices()[u].label << "')";
    std::cerr << "\n";
    std::cerr << "G2.sink (vE) index = " << ge << " label=" << rel.getVertices()[ge].label << "\n";
    std::cerr << " G2.out[" << ge << "] =";
    for (int v : G2.out[ge]) std::cerr << " " << v << "('" << rel.getVertices()[v].label << "')";
    std::cerr << "\n";

    std::cout << "++=======+++" << std::endl;
    // print outdeg/indeg with labels for G2
    int n = (int)G2.out.size();
    std::vector<int> indeg(n,0);
    for (int u=0; u<n; ++u) for (int v: G2.out[u]) if (v >= 0 && v < n) ++indeg[v];

    std::cout << "G2 nodes (index:label) outdeg indeg\n";
    for (int i=0;i<n;++i) {
        std::cout << " " << i << ":" << rel.getVertices()[i].label
                  << " out=" << G2.out[i].size()
                  << " in=" << indeg[i] << "\n";
    }


    std::cout << "==" << std::endl;
    // run Kahn and show remaining nodes if cycle
    int N = (int)G2.out.size();
    std::vector<int> indeg2(N,0);
    for (int u=0; u<N; ++u) for (int v: G2.out[u]) if (v>=0 && v<N) ++indeg2[v];

    std::queue<int> q;
    for (int i=0;i<N;++i) if (indeg2[i]==0) q.push(i);

    std::vector<int> order;
    while (!q.empty()) {
        int u = q.front(); q.pop();
        order.push_back(u);
        for (int v: G2.out[u]) {
            if (v<0||v>=N) continue;
            if (--indeg2[v]==0) q.push(v);
        }
    }

    if ((int)order.size() == N) {
        std::cout << "G2 is acyclic (topo size=" << order.size() << ")\n";
    } else {
        std::cout << "G2 has cycle: processed " << order.size() << " / " << N << " nodes\n";
        std::cout << " nodes left (likely in cycles):";
        std::vector<char> seen(N,false);
        for (int x: order) seen[x]=true;
        for (int i=0;i<N;++i) if (!seen[i]) std::cout << " " << i << "(" << rel.getVertices()[i].label << ")";
        std::cout << "\n";
    }

    for (int x : { 0, 1, 2, 3, 4, 5, 6, 7 }) {
        std::cout << "Blue outgoing halfedges from node " << x << " ("<<rel.getVertices()[x].label<<"):\n";
        for (int he : rel.getVertices()[x].edges) {
            if (he<0||he>=(int)rel.getHalfEdges().size()) continue;
            const auto &h = rel.getHalfEdges()[he];
            if (!h.outgoing) continue;
            if (h.color != BLUE) continue;
            int twin = h.twin;
            int v = (twin>=0 && twin < (int)rel.getHalfEdges().size()) ? rel.getHalfEdges()[twin].vertex : -1;
            std::cout << " he#" << he << " -> " << v << "(" << (v>=0?rel.getVertices()[v].label:"?") << ")\n";
        }
    }

    bool two = buildDualsFromREL(rel);


    return one && two;
}

bool RectangularDual::buildSTGraphsFromREL(const RegularEdgeLabeling &rel)
{
    const auto &V  = rel.getVertices();
    const auto &HE = rel.getHalfEdges();

    const int n = (int)V.size();
    if (n == 0) return false;

    std::array<int,4> ext;
    if (!auto_detect_exterior(rel, ext)) {
        std::cerr << "Failed to detect exterior vertices\n";
        return false;
    }

    const int vS = ext[0];
    const int vW = ext[1];
    const int vN = ext[2];
    const int vE = ext[3];

    // clear graphs
    G1.out.assign(n, {});
    G1.in .assign(n, {});
    G2.out.assign(n, {});
    G2.in .assign(n, {});

    G1.source = vS;  G1.sink = vN;
    G2.source = vW;  G2.sink = vE;

    // ---- process each EDGE ONCE ----
    std::vector<bool> used(HE.size(), false);

    for (int i=0;i<HE.size();++i)
    {
        if (used[i]) continue;
        int t = HE[i].twin;
        if (t < 0) continue;

        used[i]=used[t]=true;

        int u = HE[i].vertex;
        int v = HE[t].vertex;

        if (u<0||v<0||u>=n||v>=n) continue;

        // skip edges between two exterior vertices
        auto is_ext = [&](int x){
            return x==vS||x==vW||x==vN||x==vE;
        };
        if (is_ext(u)&&is_ext(v)) continue;

        auto c = HE[i].color;

        // orientation is defined by the outgoing flag of the pair
        int from,to;
        if (HE[i].outgoing) { from=u; to=v; }
        else                { from=v; to=u; }

        if (c == RED)
        {
            // vertical relation (below -> above)
            G1.out[from].push_back(to);
            G1.in[to].push_back(from);
        }
        else if (c == BLUE)
        {
            // horizontal relation (left -> right)
            G2.out[from].push_back(to);
            G2.in[to].push_back(from);
        }
    }

    // ---- add exterior frame edges (REQUIRED FOR ST) ----

    auto add = [&](STGraph& G,int a,int b){
        G.out[a].push_back(b);
        G.in[b].push_back(a);
    };

    // vertical graph (bottom → top)
    add(G1,vS,vW);
    add(G1,vS,vE);
    add(G1,vW,vN);
    add(G1,vE,vN);

    // horizontal graph (left → right)
    add(G2,vW,vS);
    add(G2,vW,vN);
    add(G2,vS,vE);
    add(G2,vN,vE);

    // ---- deduplicate ----
    auto dedup=[&](STGraph& G){
        for(int i=0;i<n;++i){
            auto &o=G.out[i];
            std::sort(o.begin(),o.end());
            o.erase(std::unique(o.begin(),o.end()),o.end());

            auto &in=G.in[i];
            std::sort(in.begin(),in.end());
            in.erase(std::unique(in.begin(),in.end()),in.end());
        }
    };
    dedup(G1); dedup(G2);

    std::cout<<"G1.source="<<G1.source<<" G1.sink="<<G1.sink<<"\n";
    std::cout<<"G2.source="<<G2.source<<" G2.sink="<<G2.sink<<"\n";

    return true;
}

int RectangularDual::findNextActive(int next, int H,
                                    const RegularEdgeLabeling &rel,
                                    const std::vector<HalfEdge> &relHalfedges,
                                    const std::unordered_set<int> &activeHE)
{
    for (int safety = 0; safety <= H; ++safety) {
        if (next < 0 || next >= H) return -1;
        if (activeHE.find(next) != activeHE.end()) return next;
        int twin = relHalfedges[next].twin;
        if (twin < 0) return -1;
        next = rel.getNextCyclicEdge(twin);
    }
    return -1;
}

void RectangularDual::walkFaceFrom(int start, int H, const RegularEdgeLabeling &rel,
                                   const std::vector<HalfEdge> &relHalfedges,
                                   const std::unordered_set<int> &activeHE,
                                   std::vector<char> &visited,
                                   std::vector<int> &faceOfHE,
                                   int &nextFaceId)
{
    int cur = start;
    int fid = nextFaceId++;
    int steps = 0;

    // Loop until we find the start half edge again: i.e. we found a cycle
    do {
        visited[cur] = 1;
        faceOfHE[cur] = fid;

        int twin = relHalfedges[cur].twin;
        if (twin < 0) break;

        int next = rel.getNextCyclicEdge(twin);
        next = findNextActive(next, H, rel, relHalfedges, activeHE);
        if (next < 0) break;

        cur = next;
    } while (cur != start);
}

void RectangularDual::buildDualAdjacency(bool right_to_left, int F,
                                         const std::vector<int> &activeHE_list,
                                         const std::vector<HalfEdge> &relHalfedges,
                                         const std::vector<int> &faceOfHE,
                                         std::vector<std::vector<int>> &out,
                                         std::vector<std::vector<int>> &in)
{
    out.assign(F, {});
    in.assign(F, {});
    for (int h : activeHE_list) {
        int twin = relHalfedges[h].twin;
        if (twin < 0) continue;
        int fL = faceOfHE[h];
        int fR = faceOfHE[twin];
        if (fL == -1 || fR == -1 || fL == fR) continue;
        if (right_to_left) {
            out[fR].push_back(fL); // right -> left
            in[fL].push_back(fR);
        } else {
            out[fL].push_back(fR); // left -> right (opposite)
            in[fR].push_back(fL);
        }
    }
    // dedupe each list
    for (int i = 0; i < F; ++i) {
        auto &o = out[i]; std::sort(o.begin(), o.end()); o.erase(std::unique(o.begin(), o.end()), o.end());
        auto &ii = in[i]; std::sort(ii.begin(), ii.end()); ii.erase(std::unique(ii.begin(), ii.end()), ii.end());
    }
}

void RectangularDual::testST(const std::vector<std::vector<int>> &outAdj,
                             const std::vector<std::vector<int>> &inAdj,
                             int &srcRes, int &sinkRes, int &nSrc, int &nSink)
{
    int n = (int)outAdj.size();
    srcRes = -1; sinkRes = -1; nSrc = 0; nSink = 0;
    for (int i = 0; i < n; ++i) {
        if (inAdj[i].empty()) { srcRes = i; ++nSrc; }
        if (outAdj[i].empty()) { sinkRes = i; ++nSink; }
    }
}

void RectangularDual::printSourceSinks(const std::vector<std::vector<int>> &outAdj,
                                       const std::vector<std::vector<int>> &inAdj,
                                       const std::string &label)
{
    int n = (int)outAdj.size();
    std::vector<int> srcs, sinks;
    for (int i = 0; i < n; ++i) {
        if (inAdj[i].empty()) srcs.push_back(i);
        if (outAdj[i].empty()) sinks.push_back(i);
    }
    std::cerr << label << " sources:";
    for (int s : srcs) std::cerr << " " << s;
    std::cerr << " ; sinks:";
    for (int s : sinks) std::cerr << " " << s;
    std::cerr << "\n";
}

bool RectangularDual::buildDualForColor(const RegularEdgeLabeling &rel, const STGraph &primal, EdgeColor color,
    STGraph &dualOut) {
    const auto &relVertices = rel.getVertices();
    const auto &relHalfedges = rel.getHalfEdges();

    if (relVertices.empty()) return false;
    const int H = (int)relHalfedges.size();

    auto is_exterior_pair_by_label = [&](int a, int b) -> bool {
        if (a < 0 || b < 0) return false;
        const std::string &la = relVertices[a].label;
        const std::string &lb = relVertices[b].label;
        auto isExt = [](const std::string &s) {
            return s == "South" || s == "West" || s == "North" || s == "East";
        };
        return isExt(la) && isExt(lb);
    };

    // collect active half-edges that correspond to primal edges of this color
    std::unordered_set<int> activeHE;
    std::unordered_map<uint64_t, int> uv_to_he;
    auto uvkey = [](int u, int v)->uint64_t { return ((uint64_t)(uint32_t)u << 32) | (uint32_t)v; };

    int relColorOutCount = 0;
    std::vector<int> skippedByPrimal; skippedByPrimal.reserve(H);

    for (int h = 0; h < H; ++h) {
        const HalfEdge &e = relHalfedges[h];
        if (!e.outgoing) continue;
        if (e.color != color) continue;
        ++relColorOutCount;

        int u = e.vertex;
        int t = e.twin;
        if (t < 0 || t >= H) { skippedByPrimal.push_back(h); continue; }
        int v = relHalfedges[t].vertex;
        if (is_exterior_pair_by_label(u, v)) { skippedByPrimal.push_back(h); continue; }

        // verify edge exists in primal graph
        bool found = false;
        if (u >= 0 && u < (int)primal.out.size()) {
            for (int nb : primal.out[u]) if (nb == v) { found = true; break; }
        }
        if (!found) { skippedByPrimal.push_back(h); continue; }

        activeHE.insert(h);
        activeHE.insert(t);
        uv_to_he[uvkey(u, v)] = h;
        uv_to_he[uvkey(v, u)] = t;
    }

    //DEBUGGING
    std::cerr << "buildDualForColor(color=" << (color==RED?"RED":"BLUE") << "): rel outgoing halfedges=" << relColorOutCount
              << " activeHE=" << activeHE.size() << " skipped(primal/exterior)=" << skippedByPrimal.size() << "\n";
    if (!skippedByPrimal.empty()) {
        std::cerr << "  skipped edges:";
        int printed=0;
        for (int he : skippedByPrimal) {
            if (printed++ > 10) break;
            const HalfEdge &e = relHalfedges[he];
            int t = e.twin; int v = (t>=0 && t < H) ? relHalfedges[t].vertex : -1;
            std::cerr << " he#" << he << "(" << relVertices[e.vertex].label << "->" << (v>=0?relVertices[v].label:"?") << ")";
        }
        std::cerr << "\n";
    }

    if (activeHE.empty()) {
        std::cerr << "No halfedges found for color\n";
        return false;
    }

    // prepare face mapping containers
    std::vector<int> faceOfHE(H, -1);
    std::vector<char> visited(H, 0);
    int nextFaceId = 0;

    // convert activeHE to a stable vector (deterministic iteration)
    std::vector<int> activeHE_list;
    activeHE_list.reserve(activeHE.size());
    for (int h : activeHE) activeHE_list.push_back(h);
    std::sort(activeHE_list.begin(), activeHE_list.end());

    // run face walks for all active halfedges
    for (int h : activeHE_list) if (!visited[h]) walkFaceFrom(h, H, rel, relHalfedges, activeHE, visited, faceOfHE, nextFaceId);


    int F = nextFaceId;
    if (F == 0) {
        std::cerr << "buildDualForColor: no faces found\n";
        return false;
    }

    // DEBUGGING: how many activeHE got assigned
    int assigned = 0;
    for (int h : activeHE_list) if (faceOfHE[h] != -1) ++assigned;
    std::cerr << "buildDualForColor: assignedHalfEdges = " << assigned << " / " << activeHE_list.size() << " faces=" << F << "\n";

    // store face map into appropriate member vector
    if (color == RED) {
        faceOfHalfEdge_G1.assign(H, -1);
        for (int h = 0; h < H; ++h) faceOfHalfEdge_G1[h] = faceOfHE[h];
        this->F1 = F;
    } else {
        faceOfHalfEdge_G2.assign(H, -1);
        for (int h = 0; h < H; ++h) faceOfHalfEdge_G2[h] = faceOfHE[h];
        this->F2 = F;
    }

    // Build primary orientation (right->left) and test ST property
    std::vector<std::vector<int>> out1, in1;
    buildDualAdjacency(true, F, activeHE_list, relHalfedges, faceOfHE, out1, in1);

    int srcA=-1, sinkA=-1, nSrcA=0, nSinkA=0;
    testST(out1, in1, srcA, sinkA, nSrcA, nSinkA);

    if (nSrcA == 1 && nSinkA == 1) {
        // Accept primary orientation
        dualOut.source = srcA; dualOut.sink = sinkA; dualOut.out = std::move(out1); dualOut.in = std::move(in1);
        return true;
    }

    // Otherwise try opposite orientation (left->right)
    std::vector<std::vector<int>> out2, in2;
    buildDualAdjacency(false, F, activeHE_list, relHalfedges, faceOfHE, out2, in2);
    int srcB=-1, sinkB=-1, nSrcB=0, nSinkB=0;
    testST(out2, in2, srcB, sinkB, nSrcB, nSinkB);

    if (nSrcB == 1 && nSinkB == 1) {
        dualOut.source = srcB; dualOut.sink = sinkB; dualOut.out = std::move(out2); dualOut.in = std::move(in2);
        std::cerr << "Note: dual orientation flipped (used left->right) to obtain valid st-graph\n";
        return true;
    }

    // Neither orientation gave a valid st-graph -> provide detailed diagnostics and return failure
    std::cerr << "Dual is not an st-graph (primary had " << nSrcA << " sources, " << nSinkA << " sinks; "
              << "flipped had " << nSrcB << " sources, " << nSinkB << " sinks)\n";

    printSourceSinks(out1, in1, "primary");
    printSourceSinks(out2, in2, "flipped");

    // store primary into dualOut anyway so caller can inspect
    dualOut.source = srcA; dualOut.sink = sinkA; dualOut.out = std::move(out1); dualOut.in = std::move(in1);
    return false;
}

bool RectangularDual::buildDualsFromREL(const RegularEdgeLabeling &rel) {
    const bool buildG1Dual = buildDualForColor(rel, G1, RED, G1dual);
    const bool buildG2Dual = buildDualForColor(rel, G2, BLUE, G2Dual);

    return buildG1Dual && buildG2Dual;
}

void RectangularDual::debugPrintFacesForColor(const RegularEdgeLabeling &rel, EdgeColor color) const {
    const auto &hes = rel.getHalfEdges();
    const auto &verts = rel.getVertices();
    const auto &faceMap = (color==RED ? faceOfHalfEdge_G1 : faceOfHalfEdge_G2);
    int F = (color==RED ? F1 : F2);
    if (F <= 0) { std::cout << "no faces\n"; return; }
    std::vector<std::vector<int>> faceEdges(F);
    for (int h = 0; h < (int)hes.size(); ++h) {
        int f = (h < (int)faceMap.size()) ? faceMap[h] : -1;
        if (f >= 0) faceEdges[f].push_back(h);
    }
    for (int f = 0; f < F; ++f) {
        std::cout << "Face " << f << " (size " << faceEdges[f].size() << "):";
        for (int he : faceEdges[f]) {
            const HalfEdge &h = hes[he];
            int u = h.vertex;
            int twin = h.twin;
            int v = (twin>=0 && twin < (int)hes.size()) ? hes[twin].vertex : -1;
            std::cout << " [" << u << "->" << v << " he#" << he << "]";
        }
        std::cout << "\n";
    }
}

void RectangularDual::debugListUnassignedHalfEdges(const RegularEdgeLabeling &rel, EdgeColor color) const {
    const auto &hes = rel.getHalfEdges();
    const auto &faceMap = (color==RED ? faceOfHalfEdge_G1 : faceOfHalfEdge_G2);
    std::cout << "Unassigned halfedges (color " << (color==RED?"RED":"BLUE") << "):\n";
    for (int h = 0; h < (int)hes.size(); ++h) {
        if (!hes[h].outgoing) continue;
        if (hes[h].color != color) continue;
        if (h >= (int)faceMap.size() || faceMap[h] == -1) {
            std::cout << " he#" << h << " (" << rel.otherLabelOfHalfEdge(h) << " ?) vertex=" << hes[h].vertex << "\n";
        }
    }
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