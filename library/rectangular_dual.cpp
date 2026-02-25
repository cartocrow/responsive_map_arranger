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
    if (H == 0) return false;

    DSU dsu(H);

    // 1) union half-edge with twin
    for (int h = 0; h < H; ++h) {
        int t = relHalfedges[h].twin;
        if (t >= 0 && t < H) dsu.unite(h, t);
    }

    // 2) at each vertex, union consecutive half-edges that belong to same run
    for (int v = 0; v < V; ++v) {
        const auto &inc = relVertices[v].edges;
        int deg = (int)inc.size();
        if (deg <= 1) continue;
        for (int i = 0; i < deg; ++i) {
            int j = (i + 1) % deg;
            int he_i = inc[i];
            int he_j = inc[j];
            if (he_i < 0 || he_i >= H || he_j < 0 || he_j >= H) continue;

            const HalfEdge &hi = relHalfedges[he_i];
            const HalfEdge &hj = relHalfedges[he_j];

            // Union if they are the same color AND they have the same incoming/outgoing flag.
            // This covers outgoing-blue, incoming-blue, outgoing-red, incoming-red, etc.
            if (hi.color == hj.color && hi.outgoing == hj.outgoing) {
                dsu.unite(he_i, he_j);
            }
        }
    }

    // 3) gather components
    std::unordered_map<int, std::vector<int>> comps;
    comps.reserve(H);
    for (int h = 0; h < H; ++h) {
        int root = dsu.find(h);
        comps[root].push_back(h);
    }

    // 4) build segments and classify
    maximalSegments.clear();
    maximalSegments.reserve(comps.size());
    // build half-edge -> seg index map
    std::vector<int> he_to_seg(H, -1);
    int sidx = 0;
    for (auto &kv : comps) {
        Segment seg;
        seg.halfedges = kv.second;

        bool hasOutgoingBlue = false;
        bool hasIncomingRed = false;
        bool hasOutgoingRed = false;
        bool hasIncomingBlue = false;

        for (int he : seg.halfedges) {
            if (he < 0 || he >= H) continue;
            const HalfEdge &h = relHalfedges[he];
            if (h.color == BLUE) {
                if (h.outgoing) hasOutgoingBlue = true;
                else hasIncomingBlue = true;
            } else if (h.color == RED) {
                if (h.outgoing) hasOutgoingRed = true;
                else hasIncomingRed = true;
            }
        }

        // classify: horizontal if outgoing-blue present (dominant), vertical if incoming-red present (dominant).
        // (use whichever rule matches your data; here we prefer BLUE-outgoing => horizontal, RED-incoming => vertical)
        if (hasOutgoingBlue && !hasIncomingRed && !hasOutgoingRed) {
            seg.type = SEGMENT_HORIZONTAL;
        } else if (hasIncomingRed && !hasOutgoingBlue && !hasIncomingBlue) {
            seg.type = SEGMENT_VERTICAL;
        } else {
            // mixed or ambiguous -> try to decide by majority or mark unknown
            int cntBlueOut = 0, cntRedIn = 0;
            for (int he : seg.halfedges) {
                const HalfEdge &h = relHalfedges[he];
                if (h.color == BLUE && h.outgoing) ++cntBlueOut;
                if (h.color == RED  && !h.outgoing) ++cntRedIn;
            }
            if (cntBlueOut >= cntRedIn && cntBlueOut > 0) seg.type = SEGMENT_HORIZONTAL;
            else if (cntRedIn > 0) seg.type = SEGMENT_VERTICAL;
            else seg.type = SEGMENT_UNKNOWN;
        }

        // 5) compute side vertex sets using *outgoing* half-edges only
        std::unordered_set<int> leftSet, rightSet, bottomSet, topSet;
        if (seg.type == SEGMENT_HORIZONTAL) {
            for (int he : seg.halfedges) {
                if (he < 0 || he >= H) continue;
                const HalfEdge &h = relHalfedges[he];
                if (h.color == BLUE && h.outgoing) {
                    int u = h.vertex;                 // origin (left)
                    int twin = h.twin;
                    if (twin >= 0 && twin < H) {
                        int vdest = relHalfedges[twin].vertex; // destination origin (right)
                        leftSet.insert(u);
                        rightSet.insert(vdest);
                    } else {
                        // half-edge missing twin? treat origin only
                        leftSet.insert(u);
                    }
                }
            }
            seg.incoming_vertices.assign(leftSet.begin(), leftSet.end());   // rename semantics: incoming_vertices -> left
            seg.outgoing_vertices.assign(rightSet.begin(), rightSet.end()); // outgoing_vertices -> right
        } else if (seg.type == SEGMENT_VERTICAL) {
            for (int he : seg.halfedges) {
                if (he < 0 || he >= H) continue;
                const HalfEdge &h = relHalfedges[he];
                if (h.color == RED && h.outgoing) {
                    int u = h.vertex;                 // origin (bottom)
                    int twin = h.twin;
                    if (twin >= 0 && twin < H) {
                        int vdest = relHalfedges[twin].vertex; // destination origin (top)
                        bottomSet.insert(u);
                        topSet.insert(vdest);
                    } else {
                        bottomSet.insert(u);
                    }
                }
            }
            seg.incoming_vertices.assign(bottomSet.begin(), bottomSet.end());   // incoming_vertices -> bottom
            seg.outgoing_vertices.assign(topSet.begin(), topSet.end());         // outgoing_vertices -> top
        } else {
            // unknown/mixed: populate both sides conservatively by checking origins/twin-origins
            for (int he : seg.halfedges) {
                if (he < 0 || he >= H) continue;
                int u = relHalfedges[he].vertex;
                int twin = relHalfedges[he].twin;
                int vdest = (twin >= 0 && twin < H) ? relHalfedges[twin].vertex : -1;
                if (u >= 0) seg.incoming_vertices.push_back(u);
                if (vdest >= 0) seg.outgoing_vertices.push_back(vdest);
            }
            // unique-ify
            std::sort(seg.incoming_vertices.begin(), seg.incoming_vertices.end());
            seg.incoming_vertices.erase(std::unique(seg.incoming_vertices.begin(), seg.incoming_vertices.end()), seg.incoming_vertices.end());
            std::sort(seg.outgoing_vertices.begin(), seg.outgoing_vertices.end());
            seg.outgoing_vertices.erase(std::unique(seg.outgoing_vertices.begin(), seg.outgoing_vertices.end()), seg.outgoing_vertices.end());
        }

        // write mapping for this segment's half-edges
        for (int he : seg.halfedges) if (he >= 0 && he < H) he_to_seg[he] = sidx;

        maximalSegments.push_back(std::move(seg));
        ++sidx;
    } // end components

    // 6) set per-vertex left/right/top/bottom using the he_to_seg mapping and deterministic pick
    for (int v = 0; v < V; ++v) {
        std::unordered_set<int> leftCandidates, rightCandidates, bottomCandidates, topCandidates;
        const auto &inc = relVertices[v].edges;
        for (int he : inc) {
            if (he < 0 || he >= H) continue;
            int segid = he_to_seg[he];
            if (segid < 0) continue;
            const HalfEdge &h = relHalfedges[he];
            if (h.color == BLUE) {
                if (h.outgoing) rightCandidates.insert(segid);
                else leftCandidates.insert(segid);
            } else if (h.color == RED) {
                if (h.outgoing) topCandidates.insert(segid);
                else bottomCandidates.insert(segid);
            }
        }
        auto pick_one = [](const std::unordered_set<int> &S)->int {
            if (S.empty()) return -1;
            int m = INT_MAX;
            for (int x : S) if (x < m) m = x;
            return (m==INT_MAX) ? -1 : m;
        };
        int leftSeg   = pick_one(leftCandidates);
        int rightSeg  = pick_one(rightCandidates);
        int bottomSeg = pick_one(bottomCandidates);
        int topSeg    = pick_one(topCandidates);

        // write into rel (non-const)
        rel.setVertexSegmentIndices(v, leftSeg, rightSeg, bottomSeg, topSeg);
    }

    return true;
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

    // 3) Optionally deduplicate adjacency lists (still useful)
    auto dedup = [](std::vector<std::vector<int>> &adj) {
        for (auto &nbrs : adj) {
            if (nbrs.empty()) continue;
            std::sort(nbrs.begin(), nbrs.end());
            nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
        }
    };
    dedup(G1.out); dedup(G1.in); dedup(G2.out); dedup(G2.in);


    std::cout << "G1.source=" << G1.source << " G1.sink=" << G1.sink << std::endl;
    std::cout << "G2.source=" << G2.source << " G2.sink=" << G2.sink << std::endl;
    return true;
}

// helper: make a 64-bit key from directed (u->v)
// static inline uint64_t uvkey(int u, int v) {
//     return ( (uint64_t)(uint32_t)u << 32 ) | (uint32_t)v;
// }

bool RectangularDual::buildDualForColor(const RegularEdgeLabeling &rel, const STGraph &primal, EdgeColor color,
    STGraph &dualOut) {
    const auto &relVertices = rel.getVertices();
    const auto &relHalfedges = rel.getHalfEdges();

    if (relVertices.empty()) return false;
    const int H = (int)relHalfedges.size();

    // 1) get half edges beloning to primal ST graph
    std::unordered_set<int> activeHE;
    std::unordered_map<uint64_t, int> uv_to_he;
    auto uvkey = [](int u, int v)->uint64_t { return ((uint64_t)(uint32_t)u << 32) | (uint32_t)v; };

    for (int h = 0; h < relHalfedges.size(); ++h) {
        const HalfEdge &e = relHalfedges[h];
        if (!e.outgoing) continue;
        if (e.color != color) continue;

        int u = e.vertex;
        int t = e.twin;
        if (t < 0 || t >= (int)relHalfedges.size()) continue;
        int v = relHalfedges[t].vertex;

        // verify edge exists in primal graph
        bool found = false;
        if (u >= 0 && u < (int)primal.out.size()) {
            for (int nb : primal.out[u])
                if (nb == v) { found = true; break; }
        }
        if (!found) continue;
        activeHE.insert(h);
        uv_to_he[uvkey(u, v)] = h;
    }

    if (activeHE.empty()) {
        std::cerr << "No halfedges found for color\n";
        return false;
    }

// prepare face mapping containers
    std::vector<int> faceOfHE(H, -1);
    std::vector<char> visited(H, 0);
    int nextFaceId = 0;

    // face walk using REL's getNextCyclicEdge
    auto walkFaceFrom = [&](int start) {
        int cur = start;
        int fid = nextFaceId++;
        while (true) {
            visited[cur] = 1;
            faceOfHE[cur] = fid;
            int twin = relHalfedges[cur].twin;
            if (twin < 0) break;
            int next = rel.getNextCyclicEdge(twin);
            if (next < 0) break;
            cur = next;
            if (cur == start) break;
        }
    };

    // run walks for all active halfedges
    for (int h : activeHE) if (!visited[h]) walkFaceFrom(h);

    // sanity: if any active HE remain unassigned, try to start at them
    for (int h : activeHE) if (faceOfHE[h] == -1) walkFaceFrom(h);

    int F = nextFaceId;
    if (F == 0) {
        std::cerr << "buildDualForColor: no faces found\n";
        return false;
    }

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

    // build dual adjacency: for each active halfedge h, right=face(twin), left=face(h)
    std::vector<std::vector<int>> out(F), in(F);
    for (int h : activeHE) {
        int twin = relHalfedges[h].twin;
        if (twin < 0) continue;
        int fL = faceOfHE[h];
        int fR = faceOfHE[twin];
        if (fL == -1 || fR == -1 || fL == fR) continue;
        out[fR].push_back(fL); // right->left
        in[fL].push_back(fR);
    }

    // dedupe
    for (int i=0;i<F;++i) {
        auto &o = out[i]; std::sort(o.begin(), o.end()); o.erase(std::unique(o.begin(), o.end()), o.end());
        auto &ii = in[i]; std::sort(ii.begin(), ii.end()); ii.erase(std::unique(ii.begin(), ii.end()), ii.end());
    }

    // try to find unique source & sink by indeg/outdeg
    int source = -1, sink = -1, nSrc = 0, nSink = 0;
    for (int i=0;i<F;++i) {
        if (in[i].empty()) { source = i; ++nSrc; }
        if (out[i].empty()) { sink = i; ++nSink; }
    }

    if (nSrc != 1 || nSink != 1) {
        std::cerr << "Dual is not an st-graph ("<<nSrc<<" sources, "<<nSink<<" sinks)\n";
    }

    std::cout << nSrc << " " << nSink << std::endl;

    dualOut.source = source;
    dualOut.sink   = sink;
    dualOut.out    = std::move(out);
    dualOut.in     = std::move(in);

    return nSrc == 1 && nSink == 1;
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