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
    if (id >= rects.size()) {
        std::cout << rects.size() << std::endl;
        std::cout << id << std::endl;
        throw std::out_of_range("RectangularDual::getRect: id out of range");
    }
    return rects[id];
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

void RectangularDual::setFromREL() {

    assert(rel.hasBoundingBox());
    BoundingBox bb = m_REL->getBoundingBox().value();
    box = Rectangle(bb.left, bb.bottom, bb.right, bb.top);

    //cout << "computing max segments" << endl;
    computeMaximalSegments();
    //cout << "computing segment positions" << endl;
    computeSegmentPositions();
    //cout << "computing rectangles" << endl;
    computeRectanglesFromSegments();
    //cout << "fixing rectangles" << endl;
    fixRectangleAreas();
}

bool RectangularDual::hasValidSegmentCoords() const {
    //const size_t n = rects.size();
    for (size_t i = 4; i < rects.size(); ++i) {
        const auto &r = rects[i];
        if (r.isDisabled) continue;
        if (r.bottom >= r.top -0.1 || r.left >= r.right - 0.1) {
            // std::cout << "no valid segments for: " <<  m_REL->getVertices()[i].label << std::endl;
            // std::cout << "bottom: " << rects[i].bottom << ", top: " << rects[i].top << std::endl;
            // std::cout << "left: " << rects[i].left << ", right: " << rects[i].right << std::endl;
            // std::cout << "leftGradient: " << maximalSegments[m_REL->getVertices()[i].left_segment].gradientValue << std::endl;
            // std::cout << "rightGradient: " << maximalSegments[m_REL->getVertices()[i].right_segment].gradientValue << std::endl;
            // std::cout << "topGradient: " << maximalSegments[m_REL->getVertices()[i].top_segment].gradientValue << std::endl;
            // std::cout << "bottomGradient: " << maximalSegments[m_REL->getVertices()[i].bottom_segment].gradientValue << std::endl;
            return false;
        }
    }
    return true;
}

double RectangularDual::computeAreaDeviation() {
    double total = 0;
    auto &vertices = m_REL->getVertices();

    for (size_t i = 4; i < vertices.size(); ++i) {
        if (rects[i].isDisabled) continue;

        //double rectDeviation = vertices[i].weight / rects[i].area() - 1;
        double rectDeviation = rects[i].area() - vertices[i].weight;
        total += rectDeviation*rectDeviation;  // (rects[i].area() - vertices[i].weight) * (rects[i].area() - vertices[i].weight); //  rectDeviation * rectDeviation; // (rectArea - (targetArea) * (rectArea - targetArea);// / (rectArea);
    }
    return sqrt(total);// total; sqrt(total);
}

void RectangularDual::fixRectangleAreas() {
    if (!hasValidSegmentCoords()) {
        std::cerr << "Invalid segment coordinates." << std::endl;
        return;
    }

    auto &vertices = m_REL->getVertices();
    const double frameArea = m_REL->getBoundingBox()->area();
    double epsilon = 1.0;
    double deviation = computeAreaDeviation();

    int totalIters = 0;
    while (deviation > 0.01 * frameArea){// > 0.1) {// * frameArea) {
        totalIters++;

        for (Segment &segment : maximalSegments) {
            if (segment.fixedSegment) continue;
            segment.gradientValue = 0.0;
        }

        // set gradients for maximal segments
        for (int i = 4; i < vertices.size(); ++i) {
            auto &rect = rects[i];
            if (rect.isDisabled) continue;
            auto &v = vertices[i];

            double width = rect.right - rect.left;
            double height = rect.top - rect.bottom;
            double area = width * height;
            double targetArea = v.weight;

            //double gradientShift = targetArea * (targetArea / area - 1) / (area*area);
            double gradientShift = (targetArea - area); // /(area*area);

            maximalSegments[v.top_segment].gradientValue +=    gradientShift * width; // (targetArea - area) / (area*area) * width;//
            maximalSegments[v.bottom_segment].gradientValue -= gradientShift * width; // (targetArea - area) / (area*area) * width;// width;
            maximalSegments[v.right_segment].gradientValue += gradientShift * height; // (targetArea - area) / (area*area) * height;// height;
            maximalSegments[v.left_segment].gradientValue -= gradientShift * height; //(targetArea - area) / (area*area) * height;// height;
        }

        for (Segment &segment : maximalSegments){
            if (segment.fixedSegment) {
                segment.gradientValue = 0.0;
            }
        }

        bool changed = true;
        while (changed) {
            changed = false;
            for (int i = 4; i < vertices.size(); i++) {
                auto v = vertices[i];

                if (rects[i].right - rects[i].left < 0.2) {
                    auto &leftSegment = maximalSegments[v.left_segment];
                    auto &rightSegment = maximalSegments[v.right_segment];
                    if (leftSegment.gradientValue > rightSegment.gradientValue) {
                        changed = true;
                        if (abs(leftSegment.gradientValue) < abs(rightSegment.gradientValue)) {
                            if (!rightSegment.fixedSegment)
                                rightSegment.gradientValue = leftSegment.gradientValue;
                        }
                        else {
                            if (!leftSegment.fixedSegment)
                                leftSegment.gradientValue = rightSegment.gradientValue;
                        }
                    }
                }

                if (rects[i].top - rects[i].bottom < 0.2) {
                    auto &bottomSegment = maximalSegments[v.bottom_segment];
                    auto &topSegment = maximalSegments[v.top_segment];
                    if (bottomSegment.gradientValue > topSegment.gradientValue) {
                        changed = true;
                        if (abs(bottomSegment.gradientValue) < abs(topSegment.gradientValue)) {
                            if (!topSegment.fixedSegment)
                                topSegment.gradientValue = bottomSegment.gradientValue;
                        } else {
                            if (!bottomSegment.fixedSegment)
                                bottomSegment.gradientValue = topSegment.gradientValue;
                        }
                    }
                }
            }
        }

        for (Segment &segment : maximalSegments){//  (Segment segment : maximalSegments) {
            if (segment.fixedSegment) {
                segment.gradientValue = 0.0;
                continue;
            }
            segment.coord += segment.gradientValue * epsilon ;
        }

        computeRectanglesFromSegments();

        if (hasValidSegmentCoords() && computeAreaDeviation() < deviation) {
            epsilon *= 2;
        } else {
             while (!hasValidSegmentCoords() || (computeAreaDeviation() > deviation)){// If rects not valid -> trace back sliding segment
                 epsilon *= 0.5;

                if (epsilon == 0) {
                    std::cout << "deviation: " << computeAreaDeviation() << std::endl;
                    std::cout << "frameArea: " << frameArea << std::endl;
                    std::cout << "[WARNING] epsilon is 0. Ending gradient" << endl;
                    return;
                }

                for (Segment &segment : maximalSegments) {
                    if (segment.fixedSegment) continue;
                    segment.coord -= segment.gradientValue * epsilon ;
                }

                computeRectanglesFromSegments();
            }
        }

        deviation = computeAreaDeviation();
    }
}

bool RectangularDual::computeMaximalSegments() {
    const auto &relHalfedges = m_REL->getHalfEdges();
    const auto &relVertices  = m_REL->getVertices();
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
    int he_left = m_REL->getFirstIncomingBlue(v);   // returns half-edge index at v (incoming BLUE)
    if (he_left >= 0 && he_left < H) leftSeg = he_to_seg[he_left];

    int he_right = m_REL->getFirstOutgoingBlue(v);  // returns half-edge index at v (outgoing BLUE)
    if (he_right >= 0 && he_right < H) rightSeg = he_to_seg[he_right];

    // Vertical: bottom = segment of first incoming RED, top = segment of first outgoing RED
    int he_bottom = m_REL->getFirstIncomingRed(v);
    if (he_bottom >= 0 && he_bottom < H) bottomSeg = he_to_seg[he_bottom];

    int he_top = m_REL->getFirstOutgoingRed(v);
    if (he_top >= 0 && he_top < H) topSeg = he_to_seg[he_top];

    // Fallbacks: if any side still -1, try last-edge variants (cases where run wraps around)
    if (leftSeg == -1) {
        int he = m_REL->getLastIncomingBlue(v);
        if (he >= 0 && he < H) leftSeg = he_to_seg[he];
    }
    if (rightSeg == -1) {
        int he = m_REL->getLastOutgoingBlue(v);
        if (he >= 0 && he < H) rightSeg = he_to_seg[he];
    }
    if (bottomSeg == -1) {
        int he = m_REL->getLastIncomingRed(v);
        if (he >= 0 && he < H) bottomSeg = he_to_seg[he];
    }
    if (topSeg == -1) {
        int he = m_REL->getLastOutgoingRed(v);
        if (he >= 0 && he < H) topSeg = he_to_seg[he];
    }

    // Final fallback: if still missing, pick any candidate deterministically (min) — preserves robustness.
    if (leftSeg == -1) {
        // find any incoming BLUE half-edge and use its segment
        for (int he : m_REL->getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = m_REL->getHalfEdges()[he];
            if (h.color == BLUE && !h.outgoing) { leftSeg = he_to_seg[he]; break; }
        }
    }
    if (rightSeg == -1) {
        for (int he : m_REL->getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = m_REL->getHalfEdges()[he];
            if (h.color == BLUE && h.outgoing) { rightSeg = he_to_seg[he]; break; }
        }
    }
    if (bottomSeg == -1) {
        for (int he : m_REL->getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = m_REL->getHalfEdges()[he];
            if (h.color == RED && !h.outgoing) { bottomSeg = he_to_seg[he]; break; }
        }
    }
    if (topSeg == -1) {
        for (int he : m_REL->getVertices()[v].edges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = m_REL->getHalfEdges()[he];
            if (h.color == RED && h.outgoing) { topSeg = he_to_seg[he]; break; }
        }
    }

    // write safe-to-call setter
    m_REL->setVertexSegmentIndices(v, leftSeg, rightSeg, bottomSeg, topSeg);
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
            if (he < 0 || he >= (int)m_REL->getHalfEdges().size()) {
                std::cerr << "ERROR: segment " << si << " contains invalid halfedge index " << he << "\n";
            }
        }
    }

    return true;
}

bool RectangularDual::computeSegmentPositions(double cell_size) {
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

    const auto &verts = m_REL->getVertices();
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
        //if (verts[v].isDeleted) continue;

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
    if (m_REL->hasBoundingBox()) {
        auto obb = m_REL->getBoundingBox();
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

bool RectangularDual::computeRectanglesFromSegments() {
    //const auto &
    const auto &verts = m_REL->getVertices();
    const int V = static_cast<int>(verts.size());
    if (V == 0) {
        std::cerr << "computeRectanglesFromSegments: REL has no vertices\n";
        return false;
    }
    if (maximalSegments.empty()) {
        std::cerr << "computeRectanglesFromSegments: no maximalSegments available (call computeMaximalSegments)\n";
        return false;
    }

    // Resize rects to number of vertices
    rects.clear();
    rects.resize(static_cast<size_t>(V));

    for (int v = 4; v < V; ++v) {
        Rect r;
        r.isDisabled = verts[v].isDeleted;
        r.left   = maximalSegments[verts[v].left_segment].coord;
        r.right  = maximalSegments[verts[v].right_segment].coord;
        r.bottom = maximalSegments[verts[v].bottom_segment].coord;
        r.top    = maximalSegments[verts[v].top_segment].coord;
        r.color = verts[v].color;
        rects[v] = r;
    }

    return true;
}

double RectangularDual::aspectRatioDeviation(int rectId) const {
    if (rects[rectId].isDisabled) return 0.0;

    double preveredAspectRatio = m_REL->getVertices()[rectId].preferred_aspect_ratio;
    double aspectRatio = rects[rectId].aspectRatio();

    return abs(preveredAspectRatio - aspectRatio);
}

double RectangularDual::totalAspectRatioDeviation() const {
    double totalDeviation = 0.0;
    for (size_t i = 4; i < rects.size(); ++i) {
        if (rects[i].isDisabled) return 0.0;
        totalDeviation += aspectRatioDeviation(i);
    }

    return totalDeviation;
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