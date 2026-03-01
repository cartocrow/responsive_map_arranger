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

bool RectangularDual::hasValidSegmentCoords() const {
    for (int i = 4; i < rects.size(); ++i) {
        if (rects[i].bottom >= rects[i].top || rects[i].left >= rects[i].right) {
            //std::cout << "rect:" << std::endl;
            //std::cout << "bottom: " << rects[i].bottom <<  " top: " << rects[i].top << " left: " << rects[i].left << " right: " << rects[i].right << std::endl;

            return false;
        };
    }

    return true;
}

double RectangularDual::computeAreaDeviation(RegularEdgeLabeling &rel) {
    double total = 0;

    auto &vertices = rel.getVertices();

    for (int i = 4; i < vertices.size(); ++i) {
        auto rectArea = rects[i].computeArea();
        double targetArea = vertices[i].weight;

        total += (rectArea - targetArea) * (rectArea - targetArea);
    }
    return sqrt(total);

}

void RectangularDual::fixRectangleAreas(RegularEdgeLabeling &rel) {
    if (!hasValidSegmentCoords()) {
        std::cerr << "Invalid segment coordinates." << std::endl;
        return;
    }
    std::cout << "fixing areas..." << std::endl;

    auto &vertices = rel.getVertices();
    const double frameArea = rel.getBoundingBox()->area();
    double epsilon = 1.0;
    double deviation = computeAreaDeviation(rel);

    while (deviation > 0.0001 * frameArea) {
        for (Segment &segment : maximalSegments) {

            if (segment.fixedSegment) continue;
            segment.gradientValue = 0.0;
        }

        // set gradients for maximal segments
        for (int i = 4; i < vertices.size(); ++i) {
            auto &rect = rects[i];
            auto &v = vertices[i];

            double width = rect.right - rect.left;
            double height = rect.top - rect.bottom;
            double area = width * height;
            double targetArea = v.weight;

            maximalSegments[v.top_segment].gradientValue += (targetArea - area) * width;
            maximalSegments[v.bottom_segment].gradientValue -= (targetArea - area) * width;
            maximalSegments[v.right_segment].gradientValue += (targetArea - area) * height;
            maximalSegments[v.left_segment].gradientValue -= (targetArea - area) * height;
        }

        for (Segment &segment : maximalSegments){//  (Segment segment : maximalSegments) {
            if (segment.fixedSegment) continue;
            segment.coord += segment.gradientValue * epsilon ;
        }

        computeRectanglesFromSegments(rel);

        if (hasValidSegmentCoords() && computeAreaDeviation(rel) < deviation) {
            epsilon *= 2;
        } else {
            while (!hasValidSegmentCoords() || computeAreaDeviation(rel) > deviation) { // If rects not valid -> trace back sliding segment
                epsilon *= 0.5;
                for (Segment &segment : maximalSegments) {
                    if (segment.fixedSegment) continue;
                    segment.coord -= segment.gradientValue * epsilon ;
                }
                computeRectanglesFromSegments(rel);

            }
        }

        deviation = computeAreaDeviation(rel);
    }

    std::cout << "fixed areas" << std::endl;
}

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

        if (verts[v].label == "West" || verts[v].label == "East" || verts[v].label == "South" || verts[v].label == "North") {
            if (leftSeg != -1) maximalSegments[leftSeg].fixedSegment = true;
            if (rightSeg != -1) maximalSegments[rightSeg].fixedSegment = true;
            if (bottomSeg != -1) maximalSegments[bottomSeg].fixedSegment = true;
            if (topSeg != -1) maximalSegments[topSeg].fixedSegment = true;
        }

        // horizontal: left -> right
        int a = hnode(leftSeg);
        int b = hnode(rightSeg);
        if (leftSeg == -1 && rightSeg == -1) {
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

    // Topo-sort
    std::vector<std::uint32_t> htopo, vtopo;
    if (!topoSort(horAdj, htopo)) { std::cerr << "horizontal DAG has cycle\n"; return false; }
    if (!topoSort(verAdj, vtopo)) { std::cerr << "vertical DAG has cycle\n"; return false; }

    // longest-path distances
    std::vector<int> hx(horAdj.size(), 0), vy(verAdj.size(), 0);
    for (auto u : htopo) for (auto w : horAdj[u]) hx[w] = std::max(hx[w], hx[u] + 1);
    for (auto u : vtopo) for (auto w : verAdj[u]) vy[w] = std::max(vy[w], vy[u] + 1);

    // Recover segment integer levels (map back to S-sized arrays)
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

    // Frame node integer levels (for fallback)
    int leftFrameX = hx[leftFrameIdx], rightFrameX = hx[rightFrameIdx];
    int bottomFrameY = vy[bottomFrameIdx], topFrameY = vy[topFrameIdx];

    // If REL provides a bounding box, map integer levels into that box.
    if (rel.hasBoundingBox()) {
        auto obb = rel.getBoundingBox();
        if (obb) {
            const BoundingBox &bb = *obb;
            // bounding box is assumed to satisfy right > left and top > bottom
            double spanX = bb.right - bb.left;
            double spanY = bb.top - bb.bottom;

            // Compute min/max among actual horizontal nodes (0 .. hcount-1)
            bool haveH = false;
            double minHX = std::numeric_limits<double>::infinity();
            double maxHX = -std::numeric_limits<double>::infinity();
            for (int hi = 0; hi < hcount; ++hi) {
                // consider only finite hx values (should be integers)
                if (std::isfinite(static_cast<double>(hx[hi]))) {
                    haveH = true;
                    minHX = std::min(minHX, (double)hx[hi]);
                    maxHX = std::max(maxHX, (double)hx[hi]);
                }
            }
            // fallback to frame nodes if no interior horizontal nodes found
            if (!haveH) {
                minHX = static_cast<double>(leftFrameX);
                maxHX = static_cast<double>(rightFrameX);
            }

            // Compute min/max among actual vertical nodes (0 .. vcount-1)
            bool haveV = false;
            double minVY = std::numeric_limits<double>::infinity();
            double maxVY = -std::numeric_limits<double>::infinity();
            for (int vi = 0; vi < vcount; ++vi) {
                if (std::isfinite(static_cast<double>(vy[vi]))) {
                    haveV = true;
                    minVY = std::min(minVY, (double)vy[vi]);
                    maxVY = std::max(maxVY, (double)vy[vi]);
                }
            }
            if (!haveV) {
                minVY = static_cast<double>(bottomFrameY);
                maxVY = static_cast<double>(topFrameY);
            }

            // guard against degenerate ranges
            if (maxHX == minHX) maxHX = minHX + 1.0;
            if (maxVY == minVY) maxVY = minVY + 1.0;

            // assign coordinates by linear interpolation into bbox
            for (int si = 0; si < S; ++si) {
                if (maximalSegments[si].type == SEGMENT_HORIZONTAL) {
                    int hi = seg_to_hidx[si];
                    if (hi >= 0) {
                        double val = static_cast<double>(hx[hi]);
                        double t = (val - minHX) / (maxHX - minHX);
                        maximalSegments[si].coord = bb.left + t * spanX;
                    } else {
                        // fallback: place at left boundary + small epsilon (one percent)
                        maximalSegments[si].coord = bb.left + 0.01 * spanX;
                    }
                } else if (maximalSegments[si].type == SEGMENT_VERTICAL) {
                    int vi = seg_to_vidx[si];
                    if (vi >= 0) {
                        double val = static_cast<double>(vy[vi]);
                        double t = (val - minVY) / (maxVY - minVY);
                        maximalSegments[si].coord = bb.bottom + t * spanY;
                    } else {
                        // fallback: place at bottom boundary + small epsilon
                        maximalSegments[si].coord = bb.bottom + 0.01 * spanY;
                    }
                } else {
                    maximalSegments[si].coord = 0.0;
                }
            }
        } else {
            // Defensive: if optional empty, fall back to old behaviour using cell_size
            for (int si = 0; si < S; ++si) {
                if (maximalSegments[si].type == SEGMENT_HORIZONTAL) {
                    int hi = seg_to_hidx[si];
                    if (hi >= 0) maximalSegments[si].coord = double(hx[hi]) * cell_size;
                    else maximalSegments[si].coord = 0.0;
                } else if (maximalSegments[si].type == SEGMENT_VERTICAL) {
                    int vi = seg_to_vidx[si];
                    if (vi >= 0) maximalSegments[si].coord = double(vy[vi]) * cell_size;
                    else maximalSegments[si].coord = 0.0;
                } else {
                    maximalSegments[si].coord = 0.0;
                }
            }
        }
    } else {
        // original behaviour: place segments on integer grid * cell_size
        for (int si = 0; si < S; ++si) {
            if (maximalSegments[si].type == SEGMENT_HORIZONTAL) {
                if (seg_x[si] >= 0) maximalSegments[si].coord = double(seg_x[si]) * cell_size;
                else maximalSegments[si].coord = 0.0;
            } else if (maximalSegments[si].type == SEGMENT_VERTICAL) {
                if (seg_y[si] >= 0) maximalSegments[si].coord = double(seg_y[si]) * cell_size;
                else                 maximalSegments[si].coord = 0.0;
            } else {
                maximalSegments[si].coord = 0.0;
            }
        }
    }

    return true;
}

bool RectangularDual::computeRectanglesFromSegments(const RegularEdgeLabeling &rel, double cell_size) {
    //const auto &


    const auto &verts = rel.getVertices();
    const int V = static_cast<int>(verts.size());
    if (V == 0) {
        std::cerr << "computeRectanglesFromSegments: REL has no vertices\n";
        return false;
    }
    if (maximalSegments.empty()) {
        std::cerr << "computeRectanglesFromSegments: no maximalSegments available (call computeMaximalSegments)\n";
        return false;
    }

    // collect min/max of real segment coordinates
    double minSegX = std::numeric_limits<double>::infinity();
    double maxSegX = -std::numeric_limits<double>::infinity();
    double minSegY = std::numeric_limits<double>::infinity();
    double maxSegY = -std::numeric_limits<double>::infinity();

    for (const auto &s : maximalSegments) {
        if (s.type == SEGMENT_HORIZONTAL) {
            if (std::isfinite(s.coord)) {
                minSegX = std::min(minSegX, s.coord);
                maxSegX = std::max(maxSegX, s.coord);
            }
        } else if (s.type == SEGMENT_VERTICAL) {
            if (std::isfinite(s.coord)) {
                minSegY = std::min(minSegY, s.coord);
                maxSegY = std::max(maxSegY, s.coord);
            }
        }
    }

    // If any direction has no real segments, provide reasonable defaults.
    // We choose a 1-cell frame around 0..cell_size so vertices touching frame behave correctly.
    if (!std::isfinite(minSegX) || !std::isfinite(maxSegX)) {
        minSegX = 0.0;
        maxSegX = cell_size;
    }
    if (!std::isfinite(minSegY) || !std::isfinite(maxSegY)) {
        minSegY = 0.0;
        maxSegY = cell_size;
    }

    // define frame coordinates OUTSIDE the segment extremes
    // (important: frame should be beyond the outermost segment coords)
    double leftFrameX  = minSegX - cell_size;
    double rightFrameX = maxSegX + cell_size;
    double bottomFrameY = minSegY - cell_size;
    double topFrameY    = maxSegY + cell_size;

    // Resize rects to number of vertices
    rects.clear();
    rects.resize((size_t)V);

    // helpers: read coordinate from a segment id but validate type
    auto getHorizCoord = [&](int segId)->double {
        if (segId < 0 || segId >= (int)maximalSegments.size()) return std::numeric_limits<double>::quiet_NaN();
        const Segment &s = maximalSegments[segId];
        if (s.type != SEGMENT_HORIZONTAL) return std::numeric_limits<double>::quiet_NaN();
        return s.coord;
    };
    auto getVertCoord = [&](int segId)->double {
        if (segId < 0 || segId >= (int)maximalSegments.size()) return std::numeric_limits<double>::quiet_NaN();
        const Segment &s = maximalSegments[segId];
        if (s.type != SEGMENT_VERTICAL) return std::numeric_limits<double>::quiet_NaN();
        return s.coord;
    };

    for (int v = 0; v < V; ++v) {
        // pull coords (may be NaN if invalid / missing)
        double lx = getHorizCoord(verts[v].left_segment);
        double rx = getHorizCoord(verts[v].right_segment);
        double by = getVertCoord(verts[v].bottom_segment);
        double ty = getVertCoord(verts[v].top_segment);

        // If a side index is -1, map to frame coordinates (which are outside extremes).
        if (verts[v].left_segment  < 0) lx = leftFrameX;
        if (verts[v].right_segment < 0) rx = rightFrameX;
        if (verts[v].bottom_segment < 0) by = bottomFrameY;
        if (verts[v].top_segment  < 0) ty = topFrameY;

        // If a segment id was present but coord invalid (NaN), fall back to frame or nearby value.
        if (!std::isfinite(lx)) {
            // fallback: use minSegX (but keep some spacing)
            lx = leftFrameX + cell_size; // push one cell inside frame
        }
        if (!std::isfinite(rx)) {
            rx = rightFrameX - cell_size;
        }
        if (!std::isfinite(by)) {
            by = bottomFrameY + cell_size;
        }
        if (!std::isfinite(ty)) {
            ty = topFrameY - cell_size;
        }

        Rect r;
        r.left   = lx;
        r.right  = rx;
        r.bottom = by;
        r.top    = ty;
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
    return one;
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