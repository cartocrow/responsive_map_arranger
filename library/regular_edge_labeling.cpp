// regular_edge_labeling.cpp
#include "regular_edge_labeling.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <stdexcept>
#include <set>
#include <algorithm>
#include <queue>
#include <CGAL/MP_Float_impl.h>

#include <cartocrow/core/core.h>
#include <cartocrow/core/region_map.h>
#include <cartocrow/core/region_arrangement.h>
#include <cartocrow/renderer/geometry_renderer.h>

using json = nlohmann::json;
using namespace std;
using namespace cartocrow;

// ---------------- static helpers ----------------
string RegularEdgeLabeling::dirKey(const string &a, const string &b) {
    return a + "->" + b;
}
string RegularEdgeLabeling::undirKey(const string &a, const string &b) {
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
}

// ---------------- buildFromJson ----------------
void RegularEdgeLabeling::buildFromJson(const json &j, bool useSquareAspectRatios) {
    // clear existing
    m_vertices.clear();
    m_labelToIndex.clear();
    m_halfEdges.clear();

    if (!j.contains("regions") || !j["regions"].is_array()) {
        throw runtime_error("JSON must contain 'regions' array");
    }

    if (!j.contains("horizontal_order") || !j["horizontal_order"].is_array()) {
        throw runtime_error("JSON must contain 'horizontal_order' array");
    }
    if (!j.contains("vertical_order") || !j["vertical_order"].is_array()) {
        throw runtime_error("JSON must contain 'vertical_order' array");
    }

    int i = 0;
    // 1) create vertices for all provided region labels
    for (const auto &r : j["regions"]) {
        if (!r.contains("label")) throw runtime_error("Each region must have a 'label'");
        if (!r.contains("weight")) throw runtime_error("Each region must have a 'weight'");
        if (!r.contains("preferred_aspect")) throw runtime_error("Each region must have a 'preferred_aspect'");
        string lbl = r["label"].get<string>();
        if (m_labelToIndex.find(lbl) == m_labelToIndex.end()) {
            int idx = (int)m_vertices.size();
            Vertex v;
            v.label = lbl;
            v.weight = r["weight"].get<int>();
            v.oldWeight = v.weight;

            if (lbl.starts_with("sea_")) {
                v.color = cartocrow::Color{230, 230, 230};
                v.isLandRegion = false;
            } else {
                v.color = m_vertColors[i % m_vertColors.size()];
                v.isLandRegion = true;
            }
            i++;
            if (useSquareAspectRatios)
                v.preferred_aspect_ratio = 1.0;
            else v.preferred_aspect_ratio = r["preferred_aspect"].get<double>();
            m_vertices.push_back(std::move(v));
            m_labelToIndex[lbl] = idx;
        }
    }

    // set horizontal and vertical order placement of vertices
    {
        int idx = 0;
        for (const auto &entry : j["horizontal_order"]) {
            if (!entry.is_string()) { throw runtime_error("JSON must contain 'horizontal_order' string"); }
            std::string lbl = entry.get<std::string>();
            auto it = m_labelToIndex.find(lbl);
            if (it != m_labelToIndex.end()) {
                int vid = it->second;
                m_vertices[vid].horizontal_order_index = idx;
            } else {
                std::cerr << "buildFromJson: horizontal_order contains unknown label '" << lbl << "'\n";
            }
            ++idx;
        }
        idx = 0;
        for (const auto &entry : j["vertical_order"]) {
            if (!entry.is_string()) { throw runtime_error("JSON must contain 'vertical_order' string"); }
            std::string lbl = entry.get<std::string>();
            auto it = m_labelToIndex.find(lbl);
            if (it != m_labelToIndex.end()) {
                int vid = it->second;
                m_vertices[vid].vertical_order_index = idx;
            } else {
                std::cerr << "buildFromJson: vertical_order contains unknown label '" << lbl << "'\n";
            }
            ++idx;
        }
    }

    // 2) create explicit half-edges for outgoing lists, preserving per-vertex input order
    unordered_map<int, vector<int>> explicitOutPerVertex; // vertexIdx -> list of explicit halfedge indices (order)
    unordered_map<string,int> directedMap; // "A->B" -> halfedge index

    auto addOut = [&](const json &region, const string &field, EdgeColor color) {
        if (!region.contains(field)) return;
        if (!region[field].is_array()) return;
        string fromLabel = region["label"].get<string>();
        int fromIdx = m_labelToIndex.at(fromLabel);
        for (const auto &t : region[field]) {
            string toLabel = t.get<string>();
            // ensure target vertex exists in map (it might not be declared as a region label)
            if (m_labelToIndex.find(toLabel) == m_labelToIndex.end()) {
                int newIdx = (int)m_vertices.size();
                Vertex v;
                v.label = toLabel;
                m_vertices.push_back(std::move(v));
                m_labelToIndex[toLabel] = newIdx;
            }
            int toIdx = m_labelToIndex.at(toLabel);

            HalfEdge he;
            he.vertex = fromIdx;
            he.twin = -1; // link later
            he.color = color;
            he.outgoing = true;
            he.id_str = fromLabel + "->" + toLabel;

            int heIdx = (int)m_halfEdges.size();
            m_halfEdges.push_back(he);
            directedMap[ dirKey(fromLabel, toLabel) ] = heIdx;
            explicitOutPerVertex[fromIdx].push_back(heIdx);
        }
    };

    for (const auto &r : j["regions"]) {
        addOut(r, "red_out", RED);
        addOut(r, "blue_out", BLUE);
    }

    // 3) build undirected map and link/create twins (same as before)
    unordered_map<string, pair<int,int>> undirMap; // undirKey -> (leftSlot,rightSlot) where left corresponds to smaller label
    for (const auto &kv : directedMap) {
        string key = kv.first; // "A->B"
        size_t p = key.find("->");
        string a = key.substr(0,p);
        string b = key.substr(p+2);
        string ukey = undirKey(a,b);
        if (!undirMap.count(ukey)) undirMap[ukey] = {-1,-1};
        if (a <= b) undirMap[ukey].first = kv.second;
        else undirMap[ukey].second = kv.second;
    }

    // create implicit twin if missing or link explicit twins
    for (auto &kv : undirMap) {
        string ukey = kv.first;
        int left = kv.second.first;
        int right = kv.second.second;

        size_t bar = ukey.find('|');
        string a = ukey.substr(0, bar);
        string b = ukey.substr(bar+1);

        if (left != -1 && right != -1) {
            // both explicit: link twins
            m_halfEdges[left].twin = right;
            m_halfEdges[right].twin = left;
        } else if (left != -1 && right == -1) {
            // create implicit twin attached to b
            HalfEdge twin;
            twin.vertex = m_labelToIndex.at(b);
            twin.twin = left;
            twin.color = m_halfEdges[left].color;
            twin.outgoing = false;
            twin.id_str = b + "<-" + a;
            int twinIdx = (int)m_halfEdges.size();
            m_halfEdges.push_back(twin);
            m_halfEdges[left].twin = twinIdx;
        } else if (left == -1 && right != -1) {
            // create implicit twin attached to a
            HalfEdge twin;
            twin.vertex = m_labelToIndex.at(a);
            twin.twin = right;
            twin.color = m_halfEdges[right].color;
            twin.outgoing = false;
            twin.id_str = a + "<-" + b;
            int twinIdx = (int)m_halfEdges.size();
            m_halfEdges.push_back(twin);
            m_halfEdges[right].twin = twinIdx;
        }
    }

    // 4) build incomingMap: for vertex v, incomingMap[v][uLabel] = halfedge idx at v representing u->v
    vector<unordered_map<string,int>> incomingMap(m_vertices.size());
    for (int hi = 0; hi < (int)m_halfEdges.size(); ++hi) {
        const HalfEdge &h = m_halfEdges[hi];
        if (!h.outgoing) {
            // h is an incoming-styled halfedge attached at h.vertex, coming from twin
            if (h.twin != -1) {
                int otherV = m_halfEdges[h.twin].vertex;
                incomingMap[h.vertex][ m_vertices[otherV].label ] = hi;
            }
        } else {
            // outgoing: h.twin corresponds to incoming at other vertex
            if (h.twin != -1) {
                int otherV = m_halfEdges[h.twin].vertex;
                incomingMap[ otherV ][ m_vertices[h.vertex].label ] = h.twin;
            }
        }
    }

    // Helper: ensure (and create if needed) an incoming half-edge at dest from src of given color.
    // Returns the index of the incoming half-edge at dest (attached to dest) representing src->dest.
    auto ensureIncomingHalfEdgeAt = [&](int srcIdx, int destIdx, EdgeColor color) -> int {
        const string &srcLabel = m_vertices[srcIdx].label;
        // if incoming already exists, return it
        auto it = incomingMap[destIdx].find(srcLabel);
        if (it != incomingMap[destIdx].end()) return it->second;

        // attempt to link to an explicit outgoing if present
        string dir = dirKey(m_vertices[srcIdx].label, m_vertices[destIdx].label);
        auto dit = directedMap.find(dir);
        if (dit != directedMap.end()) {
            int outHe = dit->second;
            // if outHe exists but hasn't a twin (should have from undir step), create twin to attach here
            if (outHe >= 0 && outHe < (int)m_halfEdges.size() && m_halfEdges[outHe].twin != -1) {
                int twinIdx = m_halfEdges[outHe].twin;
                incomingMap[destIdx][srcLabel] = twinIdx;
                return twinIdx;
            }
            // else if outHe exists but no twin, create one now and link
            if (outHe >= 0 && outHe < (int)m_halfEdges.size() && m_halfEdges[outHe].twin == -1) {
                HalfEdge twin;
                twin.vertex = destIdx;
                twin.twin = outHe;
                twin.color = color;
                twin.outgoing = false;
                twin.id_str = m_vertices[destIdx].label + "<-" + srcLabel;
                int twinIdx = (int)m_halfEdges.size();
                m_halfEdges.push_back(twin);
                m_halfEdges[outHe].twin = twinIdx;
                incomingMap[destIdx][srcLabel] = twinIdx;
                return twinIdx;
            }
        }

        // last resort: create an incoming half-edge at dest (no explicit outgoing exists)
        HalfEdge twin;
        twin.vertex = destIdx;
        twin.twin = -1;
        twin.color = color;
        twin.outgoing = false;
        twin.id_str = m_vertices[destIdx].label + "<-" + srcLabel;
        int twinIdx = (int)m_halfEdges.size();
        m_halfEdges.push_back(twin);
        incomingMap[destIdx][srcLabel] = twinIdx;
        return twinIdx;
    };

    // 5) build outNeighbors list per vertex preserving input order
    vector<vector<string>> outNeighbors(m_vertices.size());
    for (const auto &kv : explicitOutPerVertex) {
        int vIdx = kv.first;
        for (int heIdx : kv.second) {
            const string &id = m_halfEdges[heIdx].id_str; // "A->B"
            size_t p = id.find("->");
            string toLabel = id.substr(p+2);
            outNeighbors[vIdx].push_back(toLabel);
        }
    }

    // 5b) parse optional prescribed incoming orders blue_in / red_in from JSON
    // store them as label sequences per vertex (may be empty)
    vector<vector<string>> prescribedBlueIn(m_vertices.size()), prescribedRedIn(m_vertices.size());
    for (const auto &r : j["regions"]) {
        string lbl = r["label"].get<string>();
        int vid = m_labelToIndex.at(lbl);

        if (r.contains("blue_in") && r["blue_in"].is_array()) {
            for (const auto &entry : r["blue_in"]) {
                if (!entry.is_string()) continue;
                string src = entry.get<string>();
                if (m_labelToIndex.find(src) == m_labelToIndex.end()) {
                    std::cerr << "buildFromJson: blue_in contains unknown label '" << src << "'\n";
                    continue;
                }
                int srcIdx = m_labelToIndex.at(src);
                // ensure incoming half-edge exists (create if needed)
                ensureIncomingHalfEdgeAt(srcIdx, vid, BLUE);
                prescribedBlueIn[vid].push_back(src);
            }
        }

        if (r.contains("red_in") && r["red_in"].is_array()) {
            for (const auto &entry : r["red_in"]) {
                if (!entry.is_string()) continue;
                string src = entry.get<string>();
                if (m_labelToIndex.find(src) == m_labelToIndex.end()) {
                    std::cerr << "buildFromJson: red_in contains unknown label '" << src << "'\n";
                    continue;
                }
                int srcIdx = m_labelToIndex.at(src);
                ensureIncomingHalfEdgeAt(srcIdx, vid, RED);
                prescribedRedIn[vid].push_back(src);
            }
        }
    }

    // 6) assemble final incident list per vertex into 4 REL blocks (prescribed order preferred)
    // Desired CCW order (user request): [ incoming blue ] [ incoming red ] [ outgoing blue ] [ outgoing red ]
    for (int vIdx = 0; vIdx < (int)m_vertices.size(); ++vIdx) {
        vector<int> incomingBlue;
        vector<int> incomingRed;
        vector<int> outgoingBlue;
        vector<int> outgoingRed;

        // 6a) incoming blue: prefer prescribed order if present, otherwise derive deterministically
        if (!prescribedBlueIn[vIdx].empty()) {
            for (const string &src : prescribedBlueIn[vIdx]) {
                auto it = incomingMap[vIdx].find(src);
                if (it != incomingMap[vIdx].end()) {
                    int inHe = it->second;
                    if (inHe >= 0 && inHe < (int)m_halfEdges.size() && m_halfEdges[inHe].color == BLUE)
                        incomingBlue.push_back(inHe);
                }
            }
        } else {
            // fallback: collect incoming blue edges deterministically by source vertex index order
            for (const auto &kv : incomingMap[vIdx]) {
                int he = kv.second;
                if (he >= 0 && he < (int)m_halfEdges.size() && m_halfEdges[he].color == BLUE) incomingBlue.push_back(he);
            }
            sort(incomingBlue.begin(), incomingBlue.end(), [&](int a, int b) {
                int sa = (m_halfEdges[a].twin >= 0) ? m_halfEdges[m_halfEdges[a].twin].vertex : -1;
                int sb = (m_halfEdges[b].twin >= 0) ? m_halfEdges[m_halfEdges[b].twin].vertex : -1;
                return sa < sb;
            });
        }

        // 6b) incoming red
        if (!prescribedRedIn[vIdx].empty()) {
            for (const string &src : prescribedRedIn[vIdx]) {
                auto it = incomingMap[vIdx].find(src);
                if (it != incomingMap[vIdx].end()) {
                    int inHe = it->second;
                    if (inHe >= 0 && inHe < (int)m_halfEdges.size() && m_halfEdges[inHe].color == RED)
                        incomingRed.push_back(inHe);
                }
            }
        } else {
            for (const auto &kv : incomingMap[vIdx]) {
                int he = kv.second;
                if (he >= 0 && he < (int)m_halfEdges.size() && m_halfEdges[he].color == RED) incomingRed.push_back(he);
            }
            sort(incomingRed.begin(), incomingRed.end(), [&](int a, int b) {
                int sa = (m_halfEdges[a].twin >= 0) ? m_halfEdges[m_halfEdges[a].twin].vertex : -1;
                int sb = (m_halfEdges[b].twin >= 0) ? m_halfEdges[m_halfEdges[b].twin].vertex : -1;
                return sa < sb;
            });
        }

        // 6c) outgoing lists: preserve earlier explicit order you built (explicitOutPerVertex)
        auto itExp = explicitOutPerVertex.find(vIdx);
        if (itExp != explicitOutPerVertex.end()) {
            for (int heIdx : itExp->second) {
                if (heIdx < 0 || heIdx >= (int)m_halfEdges.size()) continue;
                const HalfEdge &hOut = m_halfEdges[heIdx];
                if (hOut.color == BLUE) outgoingBlue.push_back(heIdx);
                else if (hOut.color == RED) outgoingRed.push_back(heIdx);
            }
        }

        // final concatenation in requested order: IB | IR | OB | OR
        vector<int> incident;
        incident.reserve(incomingBlue.size() + incomingRed.size() + outgoingBlue.size() + outgoingRed.size());
        incident.insert(incident.end(), incomingBlue.begin(), incomingBlue.end());
        incident.insert(incident.end(), incomingRed.begin(), incomingRed.end());
        incident.insert(incident.end(), outgoingBlue.begin(), outgoingBlue.end());
        incident.insert(incident.end(), outgoingRed.begin(), outgoingRed.end());

        m_vertices[vIdx].edges.swap(incident);
    }

    m_initVertices = m_vertices;
    m_initHalfEdges = m_halfEdges;


    // final REL validity check (prints diagnostics on failure)
    isValidREL();
}

void RegularEdgeLabeling::setDataValuesFromJson(const json &j) {
    if (!j.is_object()) {
        throw std::runtime_error("setDataValuesFromJson: expected a JSON object");
    }

    std::unordered_map<std::string, Vertex*> vertexByLabel;
    for (auto &v : m_initVertices) {
        vertexByLabel[v.label] = &v;
    }

    // Assign values from JSON
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string &region = it.key();

        auto found = vertexByLabel.find(region);
        if (found == vertexByLabel.end()) {
            std::cout << "[WARNING]: region '" << region
                      << "' not found in vertices\n";
            continue;
        }

        if (!it.value().is_number()) {
            std::cerr << "[WARNING]: value for region '" << region
                      << "' is not numeric, skipping\n";
            continue;
        }

        found->second->oldWeight = it.value().get<double>();
        found->second->weight = it.value().get<double>();
    }

    adjustToBB();
}

PolygonWithHoles<Exact> getShape(const RegionArrangement::Face_const_handle face) {
    Polygon<Exact> polygon;
    const auto circ = face->outer_ccb();
    auto curr = circ;
    int n = 0;
    do {
        const auto p = curr->source()->point();
        n++;
        polygon.push_back(p);
    } while (++curr != circ);
    std::cout << n << std::endl;
    return cartocrow::PolygonWithHoles<Exact>(polygon);
}

void RegularEdgeLabeling::setValuesFromRegionMap(const RegionMap& map) {
    std::unordered_map<std::string, Vertex*> vertexByLabel;
    for (auto &v : m_initVertices) {
        vertexByLabel[v.label] = &v;
    }

    std::cout << "setting region values" << std::endl;

    for (const auto &[name, region] : map) {
        auto found = vertexByLabel.find(name);

        std::cout << name << std::endl;

        if (found == vertexByLabel.end()) {
            std::cout << "[WARNING]: region '" << region.name << "' not found in vertices\n";
            continue;
        }

        found->second->color = region.color;
        std::vector<PolygonWithHoles<Exact>> polygons;
        region.shape.polygons_with_holes(std::back_inserter(polygons));

        std::vector<std::pair<PolygonWithHoles<Exact>, Number<Exact>>> parts;

        for (const auto &p : polygons) {
            parts.push_back(pair(p, area(p)));
        }

        std::sort(parts.begin(), parts.end(), [](const auto &p1, const auto &p2) {
            return p1.second > p2.second;
        });

        auto bb = parts.front().first.bbox();

        found->second->preferred_aspect_ratio = bb.x_span() / bb.y_span();

    }
    adjustToBB();
}

static bool checkEdgeConsistency(const RegularEdgeLabeling& rel)
{
    const auto& H = rel.getHalfEdges();

    auto vertices = rel.getVertices();

    for (int i = 0; i < H.size(); ++i) {
        int t = H[i].twin;
        if (t < 0 || t >= H.size()) {
            std::cout << "twin edge " << t << " is invalid of vertices " << vertices[H[i].vertex].label << " and " << vertices[H[t].vertex].label << std::endl;

            return false;
        }

        // same color
        if (H[i].color != H[t].color) {
            std::cout << "color mismatch: " << std::endl;
            std::cout << "vertex: " << vertices[H[i].vertex].label << " color: " << H[i].color << "  vertex: " << vertices[H[t].vertex].label << " color: " << H[t].color << std::endl;

            return false;
        }

        // exactly one outgoing
        if (H[i].outgoing == H[t].outgoing) {
            std::cout << "edge direction mismatch: " << std::endl;

            std::cout << "vertex: " << vertices[H[i].vertex].label << " dir: " << H[i].outgoing << "  vertex: " << vertices[H[t].vertex].label << " color: " << H[t].outgoing << std::endl;

            return false;
        }
    }
    return true;
}

static bool checkVertexBlocks(const RegularEdgeLabeling& rel, bool debugging = false)
{
    const auto& V = rel.getVertices();
    const auto& H = rel.getHalfEdges();

    auto typeToString = [](int t)->std::string {
        switch(t) {
            case 0: return "IB";
            case 1: return "IR";
            case 2: return "OB";
            case 3: return "OR";
            default: return "?";
        }
    };

    for (int v = 0; v < (int)V.size(); ++v)
    {
        const auto& edges = V[v].edges;
        if (edges.empty()) continue;

        // build compressed sequence of types (no consecutive duplicates)
        std::vector<int> compressed;
        compressed.reserve(edges.size());
        for (int he : edges) {
            const auto &e = H[he];
            int t = -1;
            if (e.color == BLUE && !e.outgoing) t = 0; // IB
            else if (e.color == RED  && !e.outgoing) t = 1; // IR
            else if (e.color == BLUE &&  e.outgoing) t = 2; // OB
            else if (e.color == RED  &&  e.outgoing) t = 3; // OR
            else continue;

            if (compressed.empty() || compressed.back() != t) compressed.push_back(t);
        }

        if (compressed.empty()) continue;

        // MERGE wrap-around duplicate: if first==last and size>1, drop last
        if (compressed.size() > 1 && compressed.front() == compressed.back()) {
            compressed.pop_back();
        }

        // Core check:
        // Map each block to offset relative to first block, offsets must be strictly increasing and < 4.
        int base = compressed.front();
        int lastOffset = -1;
        bool ok = true;
        for (int x : compressed) {
            int offset = (x - base + 4) % 4; // 0..3
            if (offset <= lastOffset) { ok = false; break; } // not strictly increasing
            lastOffset = offset;
        }

        if (!ok) {
            if (debugging) {
                std::cout << "Vertex block-order violation at vertex " << v
                          << " label='" << V[v].label << "'; blocks = ";
                for (int b : compressed) std::cout << typeToString(b) << " ";
                std::cout << "\n";
            }
            return false;
        }
    }
    return true;
}

static bool isColorAcyclic(const RegularEdgeLabeling& rel, EdgeColor color)
{
    const auto& V = rel.getVertices();
    const auto& H = rel.getHalfEdges();

    std::vector<int> indeg(V.size(),0);
    std::vector<std::vector<int>> adj(V.size());

    for (int i=0;i<H.size();++i)
    {
        const auto& e = H[i];
        if (!e.outgoing || e.color!=color) continue;

        int u = e.vertex;
        int v = H[e.twin].vertex;

        adj[u].push_back(v);
        indeg[v]++;
    }

    queue<int> q;
    for (int i=0;i<V.size();++i)
        if (indeg[i]==0) q.push(i);

    int visited=0;
    while(!q.empty())
    {
        int u=q.front(); q.pop();
        visited++;
        for(int v:adj[u])
            if(--indeg[v]==0)
                q.push(v);
    }

    return visited==V.size();
}

bool RegularEdgeLabeling::isValidREL(bool debugging) const
{
    //return true;
    if (!checkEdgeConsistency(*this)) {
        if (debugging) std::cout<<"REL invalid: edge mismatch\n";
        return false;
    }

    if (!checkVertexBlocks(*this, debugging)) {
        if (debugging) std::cout<<"REL invalid: vertex block order\n";
        return false;
    }

    if (!isColorAcyclic(*this, BLUE)) {
        if (debugging) std::cout<<"REL invalid: blue cycle\n";
        return false;
    }

    if (!isColorAcyclic(*this, RED)) {
        if (debugging) std::cout<<"REL invalid: red cycle\n";
        return false;
    }

    if (debugging) std::cout<<"REL valid\n";
    return true;
}

void RegularEdgeLabeling::adjustToBB() {
    m_vertices = m_initVertices;
    m_halfEdges = m_initHalfEdges;

    normalizeVertexWeights();
    computePreferredSizes();

    if (!m_adaptiveLayoutEnabled) return;

    auto longestHorizontalPath = getLongestHorizontalPath();
    auto longestVerticalPath = getLongestVerticalPath();

    m_initLongestHorizontalPath = longestHorizontalPath.second.size();
    m_initLongestVerticalPath = longestVerticalPath.second.size();

    //std::cout << "Longest Vertical: " << longestVerticalPath.first << std::endl;

    //double threshHold = 0.0015 * m_boundingBox->area();
    double horizontalThreshHold = m_boundingBox->width() + m_threshHoldRelaxation * m_boundingBox->width();
    double verticalThreshHold = m_boundingBox->height() + m_threshHoldRelaxation * m_boundingBox->height();

    double horizontalStress = longestHorizontalPath.first - horizontalThreshHold;
    double verticalStress = longestVerticalPath.first - verticalThreshHold;

    bool mergingVertically = false;

    if (verticalStress >= horizontalStress) {
        mergingVertically = true;
        // Collapse horizontal segments on the longest vertical path
        while (verticalStress > 0) {
            // merge.first = edge id | merge.second = direction.
            auto merge = getLowestCostMerge(longestVerticalPath.second);

            if (merge.second) {
                mergeMaxHorizontalSegmentFromLeft(merge.first);
            } else
                mergeMaxHorizontalSegmentFromRight(merge.first);

            longestVerticalPath = getLongestVerticalPath();
            verticalStress = longestVerticalPath.first - verticalThreshHold;
        }

    }
    else {
        while (horizontalStress > 0) {
            auto merge = getLowestCostMerge(longestHorizontalPath.second);

            if (merge.second)
                mergeMaxVerticalSegmentFromBottom(merge.first);
            else mergeMaxVerticalSegmentFromTop(merge.first);

            longestHorizontalPath = getLongestHorizontalPath();
            horizontalStress = longestHorizontalPath.first - horizontalThreshHold;
        }
    }

    if (mergingVertically)
        adjustSeaRegionSizes(true, getLongestHorizontalPath().second.size());
    else adjustSeaRegionSizes(false, getLongestVerticalPath().second.size());

}

std::pair<int, bool> RegularEdgeLabeling::getLowestCostMerge(std::vector<int> const &path) const {
    std::pair lowestCostMerge(-1, false);
    double lowestCost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < path.size() - 1; i++) {
        int edge = -1;

        for (int he : m_vertices[path[i]].edges) {
            if (m_halfEdges[m_halfEdges[he].twin].vertex == path[i+1])
                edge = he;
        }
        double costFromSource = 0.0;
        double costFromTarget = 0.0;

        switch (m_mergeHeuristic) {
            case LOWEST_EDGE_COUNT:
                costFromSource = mergeEdgeCountCost(edge, true);
                costFromTarget = mergeEdgeCountCost(edge, false);
                break;
            case HIGHEST_SEGMENT_LOWEST_DIR_COUNT:
                //TODO: implement
                costFromSource = mergeEdgeCountCost(edge, true);
                costFromTarget = mergeEdgeCountCost(edge, false);
                break;
            case LOWEST_WEIGHT:
                costFromSource = mergeWeightCost(edge, true);
                costFromTarget = mergeWeightCost(edge, false);
                break;
            default: // default to LOWEST_EDGE_COUNT
                costFromSource = mergeEdgeCountCost(edge, true);
                costFromTarget = mergeEdgeCountCost(edge, false);
        }

        //costFromSource = computeLowestEdgeCountCost(edge, true);
        //costFromTarget = computeLowestEdgeCountCost(edge, false);

        bool fromSource = costFromSource <= costFromTarget;
        double lowestCostOfTwo = min(costFromSource, costFromTarget);
        if (lowestCostOfTwo < lowestCost) {
            lowestCost = lowestCostOfTwo;
            lowestCostMerge = std::pair(edge, fromSource);
        }
    }

    return lowestCostMerge;
}

double RegularEdgeLabeling::mergeEdgeCountCost(int edgeId, bool fromSource) const {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        cerr << "Invalid edgeId " << edgeId << endl;
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        cerr << "Invalid twinEdge id " << twinId << endl;
        return false;
    }

    int baseEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing)
        baseEdgeId = edgeId;
    else baseEdgeId = twinId;

    const HalfEdge &baseEdge = m_halfEdges[baseEdgeId];
    const Vertex &baseVertex = m_vertices[baseEdge.vertex];

    int edgeCount = 0;

    if ((fromSource && baseEdge.color == RED) || (!fromSource && baseEdge.color == BLUE)) {
        // next cyclic
        int currentEdgeId = baseEdgeId;
        int nextCyclicEdgeId = getNextCyclicEdge(currentEdgeId);

        do {
            edgeCount++;
            nextCyclicEdgeId = getNextCyclicEdge(currentEdgeId);

            if (m_halfEdges[nextCyclicEdgeId].color != baseEdge.color) {
                nextCyclicEdgeId = getNextCyclicEdge(m_halfEdges[nextCyclicEdgeId].twin);
            }
            currentEdgeId = nextCyclicEdgeId;
        }
        while (m_halfEdges[nextCyclicEdgeId].color == baseEdge.color);
    } else {
        // previous cyclic
        int currentEdgeId = baseEdgeId;
        int previousCyclicEdge = getPreviousCyclicEdge(currentEdgeId);

        do {
            edgeCount++;
            previousCyclicEdge = getPreviousCyclicEdge(currentEdgeId);

            if (m_halfEdges[previousCyclicEdge].color != baseEdge.color) {
                previousCyclicEdge = getPreviousCyclicEdge(m_halfEdges[previousCyclicEdge].twin);
            }
            currentEdgeId = previousCyclicEdge;
        } while (m_halfEdges[previousCyclicEdge].color == baseEdge.color);
    }

    return edgeCount;
}

double RegularEdgeLabeling::mergeWeightCost(int edgeId, bool fromSource) const {
    int baseEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing)
        baseEdgeId = edgeId;
    else baseEdgeId = m_halfEdges[edgeId].twin;

    const HalfEdge &baseEdge = m_halfEdges[baseEdgeId];
    const Vertex &baseVertex = m_vertices[baseEdge.vertex];

    unordered_set<int> merge_vertices;

    if ((fromSource && baseEdge.color == RED) || (!fromSource && baseEdge.color == BLUE)) {
        // next cyclic
        int currentEdgeId = baseEdgeId;
        int nextCyclicEdgeId = getNextCyclicEdge(currentEdgeId);

        do {
            HalfEdge he = m_halfEdges[currentEdgeId];
            merge_vertices.insert(he.vertex);
            merge_vertices.insert(m_halfEdges[he.twin].vertex);
            //edgeCount++;
            nextCyclicEdgeId = getNextCyclicEdge(currentEdgeId);

            if (m_halfEdges[nextCyclicEdgeId].color != baseEdge.color) {
                nextCyclicEdgeId = getNextCyclicEdge(m_halfEdges[nextCyclicEdgeId].twin);
            }
            currentEdgeId = nextCyclicEdgeId;
        }
        while (m_halfEdges[nextCyclicEdgeId].color == baseEdge.color);
    } else {
        // previous cyclic
        int currentEdgeId = baseEdgeId;
        int previousCyclicEdge = getPreviousCyclicEdge(currentEdgeId);

        do {
            HalfEdge he = m_halfEdges[currentEdgeId];
            merge_vertices.insert(he.vertex);
            merge_vertices.insert(m_halfEdges[he.twin].vertex);

            previousCyclicEdge = getPreviousCyclicEdge(currentEdgeId);

            if (m_halfEdges[previousCyclicEdge].color != baseEdge.color) {
                previousCyclicEdge = getPreviousCyclicEdge(m_halfEdges[previousCyclicEdge].twin);
            }
            currentEdgeId = previousCyclicEdge;
        } while (m_halfEdges[previousCyclicEdge].color == baseEdge.color);
    }

    double totalWeight = 0;
    for (const auto &v : merge_vertices) {
        totalWeight += m_vertices[v].weight;
    }

    return totalWeight;
}

void RegularEdgeLabeling::normalizeVertexWeights() {
    int total = 0;
    for (Vertex &v : m_vertices) {
        total += v.weight;
    }

    double ratio = m_boundingBox->area() / total;

    for (Vertex &v : m_vertices) {
        v.weight = v.weight * ratio;
    }
}

void RegularEdgeLabeling::computePreferredSizes() {
    for (Vertex &v : m_vertices) {
        double r = v.preferred_aspect_ratio;

        double w = sqrt(v.weight * r);
        double h = v.weight / w;

        v.preferred_width = w;
        v.preferred_height = h;
    }
}

void RegularEdgeLabeling::adjustSeaRegionSizes(bool vertically, int longestPath) {
    double t = 1;
    if (vertically) {
        t = clamp(static_cast<double>(longestPath - m_initLongestHorizontalPath) / ((m_vertices.size() - 4) - m_initLongestHorizontalPath), 0.00001, 1.0);
    } else {
        t = clamp(static_cast<double>(longestPath - m_initLongestVerticalPath) / ((m_vertices.size() - 4) - m_initLongestVerticalPath), 0.00001, 1.0);
    }

    for (Vertex &v : m_vertices) {
        if (!v.isLandRegion) {



            std::cout << "oldweight: " << v.weight << std::endl;
            v.weight = clamp(v.weight * (1-t), 1.0, static_cast<double>(v.weight));
            std::cout << "newweight: " << v.weight << std::endl;
            std::cout << "t: " << t << std::endl;
        }
    }

    normalizeVertexWeights();
}

static std::pair<double, std::vector<int>> REL_longestPathPred_generic(
    const RegularEdgeLabeling &rel,
    EdgeColor color,
    const std::string &sourceLabel,
    const std::string &sinkLabel,
    std::function<double(int)> vertexCost)
{
    const auto &V = rel.getVertices();
    const auto &H = rel.getHalfEdges();
    const int n = (int)V.size();
    if (n == 0) return { std::numeric_limits<double>::quiet_NaN(), {} };

    // find source and sink
    int source = -1, sink = -1;
    for (int i = 0; i < n; ++i) {
        if (V[i].label == sourceLabel) source = i;
        if (V[i].label == sinkLabel)   sink = i;
    }
    if (source == -1 || sink == -1) {
        std::cerr << "REL_longestPathPred_generic: missing source/sink label '"
                  << sourceLabel << "'/'" << sinkLabel << "'\n";
        return { std::numeric_limits<double>::quiet_NaN(), {} };
    }

    // helper: is this vertex an "inner" vertex (i.e. not one of the 4 outers)
    auto isInner = [&](int vid) -> bool {
        const std::string &lbl = V[vid].label;
        return !(lbl == "West" || lbl == "East" || lbl == "North" || lbl == "South");
    };

    // build adjacency over vertices using outgoing halfedges of the requested color
    std::vector<std::vector<int>> adj(n);
    std::vector<int> indeg(n,0);
    for (int hi = 0; hi < (int)H.size(); ++hi) {
        const auto &h = H[hi];
        if (!h.outgoing) continue;
        if (h.color != color) continue;
        int u = h.vertex;
        int t = h.twin;
        if (u < 0 || u >= n) continue;
        if (t < 0 || t >= (int)H.size()) continue;
        int v = H[t].vertex;
        if (v < 0 || v >= n) continue;
        adj[u].push_back(v);
        indeg[v] += 1;
    }

    // Kahn topo order
    std::deque<int> q;
    for (int i = 0; i < n; ++i) if (indeg[i] == 0) q.push_back(i);
    std::vector<int> topo;
    topo.reserve(n);
    while (!q.empty()) {
        int u = q.front(); q.pop_front();
        topo.push_back(u);
        for (int w : adj[u]) {
            if (--indeg[w] == 0) q.push_back(w);
        }
    }
    if (topo.size() != (size_t)n) {
        std::cerr << "REL_longestPathPred_generic: warning - color graph has cycle (n="
                  << n << ", topo=" << topo.size() << ")\n";
        // still continue with whatever topo we have
    }

    const double NEG_INF = -std::numeric_limits<double>::infinity();

    // We track for each vertex 3 states:
    // k = 0 : path to this vertex has 0 inner vertices (so far)
    // k = 1 : path has exactly 1 inner vertex (so far)
    // k = 2 : path has >= 2 inner vertices (cap)
    const int K = 3;
    std::vector<std::array<double, 3>> best(n);
    std::vector<std::array<int, 3>> prevVertex(n); // predecessor vertex index for (v,k)
    std::vector<std::array<int, 3>> prevK(n);      // predecessor k for (v,k)

    // init best to NEG_INF and prev to -1
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < K; ++k) {
            best[i][k] = NEG_INF;
            prevVertex[i][k] = -1;
            prevK[i][k] = -1;
        }
    }

    // initialize source state
    int initK = isInner(source) ? 1 : 0;
    best[source][initK] = isInner(source) ? vertexCost(source) : 0.0;
    prevVertex[source][initK] = -1;
    prevK[source][initK] = -1;

    // DP over topo order
    for (int u : topo) {
        for (int ku = 0; ku < K; ++ku) {
            if (best[u][ku] == NEG_INF) continue;
            for (int v : adj[u]) {
                int added = isInner(v) ? 1 : 0;
                int kv = ku + added;
                if (kv >= 2) kv = 2; // cap
                double addCost = isInner(v) ? vertexCost(v) : 0.0;
                double cand = best[u][ku] + addCost;
                if (cand > best[v][kv]) {
                    best[v][kv] = cand;
                    prevVertex[v][kv] = u;
                    prevK[v][kv] = ku;
                }
            }
        }
    }

    // We want the best path to sink with >= 2 inner vertices => state k = 2
    if (best[sink][2] == NEG_INF) {
        //std::cerr << "REL_longestPathPred_generic: sink '" << sinkLabel
        //          << "' unreachable from '" << sourceLabel << "' with >=2 inner vertices\n";
        return { std::numeric_limits<double>::quiet_NaN(), {} };
    }

    // Reconstruct the chosen path vertices by walking predecessor states
    std::vector<int> rev; // sink -> source (will reverse)
    int curV = sink;
    int curK = 2;
    while (curV != -1) {
        rev.push_back(curV);
        int pv = prevVertex[curV][curK];
        int pk = prevK[curV][curK];
        curV = pv;
        curK = pk;
    }
    std::reverse(rev.begin(), rev.end()); // now source -> sink

    // Build a pred vector compatible with existing callers:
    // pred[v] = predecessor vertex for vertices on chosen path, -1 otherwise.
    std::vector<int> pred_out(n, -1);
    for (size_t i = 1; i < rev.size(); ++i) {
        pred_out[rev[i]] = rev[i-1];
    }
    // source (rev[0]) remains pred -1

    return { best[sink][2], pred_out };
}

// Return cost + vertex path (excluding outer vertices)
std::pair<double, std::vector<int>> RegularEdgeLabeling::getLongestHorizontalPath() const
{
    // compute generic DP for blue edges West->East using preferred_width fallback to weight
    auto costAndPred = REL_longestPathPred_generic(
        *this,
        BLUE,
        "West",
        "East",
        [&](int vid)->double {
            const Vertex &v = this->m_vertices[vid];
            double w = v.preferred_width;
            if (!std::isfinite(w) || w <= 0.0) {
                return (std::isfinite(v.weight) ? v.weight : 0.0);
            }
            return w;
        }
    );

    double cost = costAndPred.first;
    if (!std::isfinite(cost)) return { cost, {} };
    const std::vector<int> &pred = costAndPred.second;

    // backtrack from sink to source using pred
    int sinkIdx = -1;
    for (int i = 0; i < (int)m_vertices.size(); ++i) if (m_vertices[i].label == "East") { sinkIdx = i; break; }
    if (sinkIdx == -1) return { std::numeric_limits<double>::quiet_NaN(), {} };

    std::vector<int> rev;
    int cur = sinkIdx;
    while (cur != -1) {
        rev.push_back(cur);
        cur = pred[cur];
    }
    std::reverse(rev.begin(), rev.end()); // now source -> sink

    // filter out outer vertices West,East,North,South
    std::vector<int> path;
    path.reserve(rev.size());
    for (int vid : rev) {
        const std::string &lbl = m_vertices[vid].label;
        if (lbl == "West" || lbl == "East" || lbl == "North" || lbl == "South") continue;
        path.push_back(vid);
    }

    // if result path has 1 or 0 nodes, treat as no valid path
    if (path.size() <= 1) {
        return { std::numeric_limits<double>::quiet_NaN(), {} };
    }

    // recompute cost for the filtered path using the same per-vertex metric
    double filteredCost = 0.0;
    for (int vid : path) {
        const Vertex &v = this->m_vertices[vid];
        double w = v.preferred_width;
        if (!std::isfinite(w) || w <= 0.0) {
            filteredCost += (std::isfinite(v.weight) ? v.weight : 0.0);
        } else {
            filteredCost += w;
        }
    }

    return { filteredCost, path };
}

std::pair<double, std::vector<int>> RegularEdgeLabeling::getLongestVerticalPath() const
{
    auto costAndPred = REL_longestPathPred_generic(
        *this,
        RED,
        "South",
        "North",
        [&](int vid)->double {
            const Vertex &v = this->m_vertices[vid];
            double h = v.preferred_height;
            if (!std::isfinite(h) || h <= 0.0) {
                return (std::isfinite(v.weight) ? v.weight : 0.0);
            }
            return h;
        }
    );

    double cost = costAndPred.first;
    if (!std::isfinite(cost)) return { cost, {} };
    const std::vector<int> &pred = costAndPred.second;

    // find sink "North"
    int sinkIdx = -1;
    for (int i = 0; i < (int)m_vertices.size(); ++i) if (m_vertices[i].label == "North") { sinkIdx = i; break; }
    if (sinkIdx == -1) return { 0, {} };

    std::vector<int> rev;
    int cur = sinkIdx;
    while (cur != -1) {
        rev.push_back(cur);
        cur = pred[cur];
    }
    std::reverse(rev.begin(), rev.end());

    // filter out outer vertices
    std::vector<int> path;
    path.reserve(rev.size());
    for (int vid : rev) {
        const std::string &lbl = m_vertices[vid].label;
        if (lbl == "West" || lbl == "East" || lbl == "North" || lbl == "South") continue;
        path.push_back(vid);
    }

    // if result path has 1 or 0 nodes, treat as no valid path
    if (path.size() <= 1) {
        return { std::numeric_limits<double>::quiet_NaN(), {} };
    }

    // recompute cost for the filtered path using the same per-vertex metric
    double filteredCost = 0.0;
    for (int vid : path) {
        const Vertex &v = this->m_vertices[vid];
        double h = v.preferred_height;
        if (!std::isfinite(h) || h <= 0.0) {
            filteredCost += (std::isfinite(v.weight) ? v.weight : 0.0);
        } else {
            filteredCost += h;
        }
    }

    return { filteredCost, path };
}

bool RegularEdgeLabeling::mergeMaxHorizontalSegmentFromLeft(int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        throw runtime_error("mergeMaxHorizontalSegment: Invalid edgeId: " + std::to_string(edgeId));
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        throw runtime_error("mergeMaxHorizontalSegment: Invalid twinEdgeId: " + std::to_string(edgeId));

        return false;
    }

    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge endEdge = m_halfEdges[endEdgeId];

    int leftMostRedEdge = getLastOutgoingRed(baseEdge.vertex); // the edge that we will collapse
    int nextEdgeId = getNextCyclicEdge(m_halfEdges[getNextCyclicEdge(leftMostRedEdge)].twin);

    // get initial leftmost red segment of the face
    while (m_halfEdges[nextEdgeId].color == RED) {
        leftMostRedEdge = getLastOutgoingRed(m_halfEdges[nextEdgeId].vertex);
        nextEdgeId = getNextCyclicEdge(m_halfEdges[getNextCyclicEdge(leftMostRedEdge)].twin);
    }

    // merge red edges from left to right
    int previousEdgeId = -1;
    while (m_halfEdges[baseEdgeId].color != BLUE && endEdge.vertex == m_halfEdges[baseEdge.twin].vertex) {

        // std::cout << "MERGING EDGE BETWEEN: " << m_vertices[m_halfEdges[leftMostRedEdge].vertex].label << " and "
        // << m_vertices[m_halfEdges[m_halfEdges[leftMostRedEdge].twin].vertex].label << std::endl;
        mergeLeftMostRedEdge(leftMostRedEdge);

        previousEdgeId = getPreviousCyclicEdge(leftMostRedEdge);

        if (m_halfEdges[previousEdgeId].color == BLUE) {
            leftMostRedEdge = getPreviousCyclicEdge(m_halfEdges[previousEdgeId].twin);
        }
        else {
            leftMostRedEdge = previousEdgeId;
        }
    }

    return true;
}

bool RegularEdgeLabeling::mergeMaxHorizontalSegmentFromRight(int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        throw runtime_error("mergeMaxHorizontalSegment: Invalid edgeId: " + std::to_string(edgeId));
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        throw runtime_error("mergeMaxHorizontalSegment: Invalid twinEdgeId: " + std::to_string(edgeId));
        return false;
    }

    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge endEdge = m_halfEdges[endEdgeId];

    int rightMostRedEdge = getFirstOutgoingRed(baseEdge.vertex); // the edge that we will collapse
    int previousEdgeId = getPreviousCyclicEdge(m_halfEdges[getPreviousCyclicEdge(rightMostRedEdge)].twin);

    // get initial rightmost red segment of the face
    while (m_halfEdges[previousEdgeId].color == RED) {
        rightMostRedEdge = getFirstOutgoingRed(m_halfEdges[previousEdgeId].vertex);
        previousEdgeId = getPreviousCyclicEdge(m_halfEdges[getPreviousCyclicEdge(rightMostRedEdge)].twin);
    }

    // merge red edges from right to left
    int nextEdgeId = -1;
    while (m_halfEdges[baseEdgeId].color != BLUE && endEdge.vertex == m_halfEdges[baseEdge.twin].vertex) {

        // std::cout << "MERGING EDGE BETWEEN: " << m_vertices[m_halfEdges[rightMostRedEdge].vertex].label << " and "
        // << m_vertices[m_halfEdges[m_halfEdges[rightMostRedEdge].twin].vertex].label << std::endl;
        mergeRightMostRedEdge(rightMostRedEdge);

        nextEdgeId = getNextCyclicEdge(rightMostRedEdge);

        if (m_halfEdges[nextEdgeId].color == BLUE) {
            rightMostRedEdge = getNextCyclicEdge(m_halfEdges[nextEdgeId].twin);
        }
        else {
            rightMostRedEdge = nextEdgeId;
        }
    }
    return true;
}

bool RegularEdgeLabeling::mergeMaxVerticalSegmentFromBottom(int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size())
        throw runtime_error("mergeMaxHorizontalSegment: Invalid edgeId: " + std::to_string(edgeId));
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size())
        throw runtime_error("mergeMaxHorizontalSegment: Invalid twinEdgeId: " + std::to_string(edgeId));

    int baseEdgeId = -1;
    int endEdgeId = -1;
    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge endEdge = m_halfEdges[endEdgeId];

    int lowestBLueEdge = getFirstOutgoingBlue(baseEdge.vertex); // the edge that we will collapse
    int previousEdgeId = getPreviousCyclicEdge(m_halfEdges[getPreviousCyclicEdge(lowestBLueEdge)].twin);

    // get initial lowest blue segment of the face
    while (m_halfEdges[previousEdgeId].color == BLUE) {
        lowestBLueEdge = getFirstOutgoingBlue(m_halfEdges[previousEdgeId].vertex);
        previousEdgeId = getPreviousCyclicEdge(m_halfEdges[getPreviousCyclicEdge(lowestBLueEdge)].twin);
    }

    int nextEdgeId = -1;
    while (m_halfEdges[baseEdgeId].color != RED && endEdge.vertex == m_halfEdges[baseEdge.twin].vertex) {

         //std::cout << "bottom: MERGING EDGE BETWEEN: " << m_vertices[m_halfEdges[lowestBLueEdge].vertex].label << " and "
         //<< m_vertices[m_halfEdges[m_halfEdges[lowestBLueEdge].twin].vertex].label << std::endl;
        mergeLowestBlueEdge(lowestBLueEdge);

        nextEdgeId = getNextCyclicEdge(lowestBLueEdge);

        if (m_halfEdges[nextEdgeId].color == RED) {
            lowestBLueEdge = getNextCyclicEdge(m_halfEdges[nextEdgeId].twin);
        }
        else {
            lowestBLueEdge = nextEdgeId;
        }
    }

    return true;
}

bool RegularEdgeLabeling::mergeMaxVerticalSegmentFromTop(int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size())
        throw runtime_error("mergeMaxHorizontalSegment: Invalid edgeId: " + std::to_string(edgeId));
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size())
        throw runtime_error("mergeMaxHorizontalSegment: Invalid twinEdgeId: " + std::to_string(edgeId));

    int baseEdgeId = -1;
    int endEdgeId = -1;
    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge endEdge = m_halfEdges[endEdgeId];

    int highestBLueEdge = getLastOutgoingBlue(baseEdge.vertex); // the edge that we will collapse
    int nextEdgeId = getNextCyclicEdge(m_halfEdges[getNextCyclicEdge(highestBLueEdge)].twin);

    // get initial highest blue segment of the face
    while (m_halfEdges[nextEdgeId].color == BLUE) {
        highestBLueEdge = getLastOutgoingBlue(m_halfEdges[nextEdgeId].vertex);
        nextEdgeId = getNextCyclicEdge(m_halfEdges[getNextCyclicEdge(highestBLueEdge)].twin);
    }

    int previousEdgeId = -1;
    while (m_halfEdges[baseEdgeId].color != RED && endEdge.vertex == m_halfEdges[baseEdge.twin].vertex) {

         //std::cout << "top: MERGING EDGE BETWEEN: " << m_vertices[m_halfEdges[highestBLueEdge].vertex].label << " and "
         //<< m_vertices[m_halfEdges[m_halfEdges[highestBLueEdge].twin].vertex].label << std::endl;
        mergeHighestBlueEdge(highestBLueEdge);

        previousEdgeId = getPreviousCyclicEdge(highestBLueEdge);

        if (m_halfEdges[previousEdgeId].color == RED) {
            highestBLueEdge = getPreviousCyclicEdge(m_halfEdges[previousEdgeId].twin);
        }
        else {
            highestBLueEdge = previousEdgeId;
        }
    }

    return true;
}

bool RegularEdgeLabeling::mergeLeftMostRedEdge(int edgeId) {

    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        cerr << "Invalid edgeId " << edgeId << endl;
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        cerr << "Invalid twinEdge id " << twinId << endl;
        return false;
    }

    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge &baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge &endEdge = m_halfEdges[endEdgeId];
    Vertex &baseVertex = m_vertices[baseEdge.vertex];
    Vertex &endVertex = m_vertices[endEdge.vertex];

    // If the bottom vertex needs to go to the left
    if (baseVertex.horizontal_order_index < endVertex.horizontal_order_index) {
        // If bottom vertex is last in the subsequence then we first want to flip all blue outgoing edge of the base node (highest to lowest order)
        {
            int lastBlueOutId = getLastOutgoingBlue(baseEdge.vertex);
            if (m_halfEdges[getPreviousCyclicEdge(m_halfEdges[lastBlueOutId].twin)].color == BLUE) {

                while (getFirstOutgoingBlue(baseEdge.vertex) != lastBlueOutId) {

                    flipEdgeDiagonally(lastBlueOutId, true);
                    lastBlueOutId = getLastOutgoingBlue(baseEdge.vertex);
                }
                // Last flipped edge recolor and flip in the other direction
                flipEdgeDiagonally(lastBlueOutId, false);
                flipEdgeColor(lastBlueOutId);
            }
        }

        // flip all outgoing red edges of the bottom vertex (right to left order)
        {
            int firstOutgoingEdgeId = getFirstOutgoingRed(baseEdge.vertex);
            while (firstOutgoingEdgeId != baseEdgeId) {
                flipEdgeDiagonally(firstOutgoingEdgeId, false);

                firstOutgoingEdgeId = getFirstOutgoingRed(baseEdge.vertex);
            }
        }

        // flip incoming blue edges on top vertex and recolor last one (lowest to highest order)
        {
            int lastIncomingEdgeId = getLastIncomingBlue(endEdge.vertex);
            while (getFirstIncomingBlue(endEdge.vertex) != lastIncomingEdgeId) {

                flipEdgeDiagonally(lastIncomingEdgeId, true);
                lastIncomingEdgeId = getLastIncomingBlue(endEdge.vertex);
            }
            // Last flipped edge recolor and flip in the other direction
            flipEdgeDiagonally(lastIncomingEdgeId, false);
            flipEdgeColor(lastIncomingEdgeId);

        }

    } // if the bottom vertex needs to go to the right
    else if (baseVertex.horizontal_order_index > endVertex.horizontal_order_index) {
        // If top vertex is last in the subsequence then we first want to flip all outgoing blue edge of the end node (lowest to highest order)
        {
            int firstBLueOutId = getFirstOutgoingBlue(endEdge.vertex);
            if (m_halfEdges[getNextCyclicEdge(m_halfEdges[firstBLueOutId].twin)].color == BLUE) {
                while (getLastOutgoingBlue(endEdge.vertex) != firstBLueOutId) {
                    flipEdgeDiagonally(firstBLueOutId, false);
                    firstBLueOutId = getFirstOutgoingBlue(endEdge.vertex);
                }
                // Last flipped edge recolor and flip in the other direction
                flipEdgeDiagonally(firstBLueOutId, false);
                flipEdgeColor(firstBLueOutId);
            }
        }

        // flip all incoming red edges of the top vertex (right to left order)
        {
            int lastOutgoingEdgeId = getLastIncomingRed(endEdge.vertex);
            while (lastOutgoingEdgeId != endEdgeId) {
                flipEdgeDiagonally(lastOutgoingEdgeId, true);
                lastOutgoingEdgeId = getLastIncomingRed(endEdge.vertex);
            }
        }

        // flip incoming blue edges on bottom vertex and recolor last one (highest to lowest order)
        {
            int firstIncomingEdgeId = getFirstIncomingBlue(baseEdge.vertex);
            while (getLastIncomingBlue(baseEdge.vertex) != firstIncomingEdgeId) {
                flipEdgeDiagonally(firstIncomingEdgeId, false);
                firstIncomingEdgeId = getFirstIncomingBlue(baseEdge.vertex);
            }
            // Last flipped edge recolor and flip in the other direction
            flipEdgeDiagonally(firstIncomingEdgeId, false);
            flipEdgeColor(firstIncomingEdgeId);
        }

        // when the bottom vertex goes to the right of the top, the edge dir needs to be reverted
        revertEdgeDirection(baseEdgeId);
    }

    // flip color of edge that needs to merge
    bool flippedColor = flipEdgeColor(baseEdgeId);

    return true;
}

bool RegularEdgeLabeling::mergeRightMostRedEdge(int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        cerr << "Invalid edgeId " << edgeId << endl;
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        cerr << "Invalid twinEdge id " << twinId << endl;
        return false;
    }

    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge &baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge &endEdge = m_halfEdges[endEdgeId];
    Vertex &baseVertex = m_vertices[baseEdge.vertex];
    Vertex &endVertex = m_vertices[endEdge.vertex];

    if (baseVertex.horizontal_order_index > endVertex.horizontal_order_index) {
        // If bottom vertex is first in the subsequence then we first want to flip all blue incoming edge of the base node (highest to lowest order)
        {
            int firstBlueInId = getFirstIncomingBlue(baseEdge.vertex);
            if (m_halfEdges[getNextCyclicEdge(m_halfEdges[firstBlueInId].twin)].color == BLUE) {
                while (getLastIncomingBlue(baseEdge.vertex) != firstBlueInId) {

                    flipEdgeDiagonally(firstBlueInId, false);
                    firstBlueInId = getFirstIncomingBlue(baseEdge.vertex);
                }
                // Last flipped edge recolor and flip in the other direction
                flipEdgeDiagonally(firstBlueInId, false);
                flipEdgeColor(firstBlueInId);
            }
        }

        // flip all outgoing red edges of the bottom vertex (left to right order)
        {
            int lastOutgoingEdgeId = getLastOutgoingRed(baseEdge.vertex);
            while (lastOutgoingEdgeId != baseEdgeId) {
                flipEdgeDiagonally(lastOutgoingEdgeId, true);

                lastOutgoingEdgeId = getLastOutgoingRed(baseEdge.vertex);
            }
        }

        // flip outgoing blue edges on top vertex and recolor last one (lowest to highest order)
        {
            int firstOutgoingEdgeId = getFirstOutgoingBlue(endEdge.vertex);
            while (getLastOutgoingBlue(endEdge.vertex) != firstOutgoingEdgeId) {

                flipEdgeDiagonally(firstOutgoingEdgeId, false);
                firstOutgoingEdgeId = getFirstOutgoingBlue(endEdge.vertex);
            }
            // Last flipped edge recolor and flip
            flipEdgeDiagonally(firstOutgoingEdgeId, false);
            flipEdgeColor(firstOutgoingEdgeId);
        }
        revertEdgeDirection(baseEdgeId);
    }
    else if (baseVertex.horizontal_order_index < endVertex.horizontal_order_index) {
        // If top vertex is first in the subsequence then we first want to flip all incoming blue edge of the end node (lowest to highest order)
        {
            int lastBlueIn = getLastIncomingBlue(endEdge.vertex);
            if (m_halfEdges[getPreviousCyclicEdge(m_halfEdges[lastBlueIn].twin)].color == BLUE) {
                while (getFirstIncomingBlue(endEdge.vertex) != lastBlueIn) {
                    flipEdgeDiagonally(lastBlueIn, true);
                    lastBlueIn = getLastIncomingBlue(endEdge.vertex);
                }
                // Last flipped edge recolor and flip in the other direction
                flipEdgeDiagonally(lastBlueIn, false);
                flipEdgeColor(lastBlueIn);
            }
        }

        // flip all incoming red edges of the top vertex (left to right order)
        {
            int firstIncomingEdgeId = getFirstIncomingRed(endEdge.vertex);
            while (firstIncomingEdgeId != endEdgeId) {
                flipEdgeDiagonally(firstIncomingEdgeId, false);
                firstIncomingEdgeId = getFirstIncomingRed(endEdge.vertex);
            }
        }

        // flip outgoing blue edges on bottom vertex and recolor last one (highest to lowest order)
        {
            int lastOutgoingEdgeId = getLastOutgoingBlue(baseEdge.vertex);
            while (getFirstOutgoingBlue(baseEdge.vertex) != lastOutgoingEdgeId) {
                flipEdgeDiagonally(lastOutgoingEdgeId, true);
                lastOutgoingEdgeId = getLastOutgoingBlue(baseEdge.vertex);
            }
            // Last flipped edge recolor and flip in the other direction
            flipEdgeDiagonally(lastOutgoingEdgeId, false);
            flipEdgeColor(lastOutgoingEdgeId);
        }
    }

    flipEdgeColor(baseEdgeId);
    return true;
}


bool RegularEdgeLabeling::mergeLowestBlueEdge(int edgeId) {

    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        cerr << "Invalid edgeId " << edgeId << endl;
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        cerr << "Invalid twinEdge id " << twinId << endl;
        return false;
    }

    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge &baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge &endEdge = m_halfEdges[endEdgeId];
    Vertex &baseVertex = m_vertices[baseEdge.vertex];
    Vertex &endVertex = m_vertices[endEdge.vertex];

    // If the left vertex needs to go to the bottom
    if (baseVertex.vertical_order_index < endVertex.vertical_order_index) {

        // flip all outgoing red edges of the left vertex (right to left order)
        {
            int firstOutgoingEdgeId = getFirstOutgoingRed(baseEdge.vertex);
            if (m_halfEdges[getNextCyclicEdge(m_halfEdges[firstOutgoingEdgeId].twin)].color == RED) {
                while (getLastOutgoingRed(baseEdge.vertex) != firstOutgoingEdgeId) {
                    flipEdgeDiagonally(firstOutgoingEdgeId, false);
                    firstOutgoingEdgeId = getFirstOutgoingRed(baseEdge.vertex);
                }
                flipEdgeDiagonally(firstOutgoingEdgeId, true);
                flipEdgeColor(firstOutgoingEdgeId);
            }
        }

        {
            int lastOutgoingEdgeId = getLastOutgoingBlue(baseEdge.vertex);
            while (lastOutgoingEdgeId != baseEdgeId) {
                flipEdgeDiagonally(lastOutgoingEdgeId, true);
                lastOutgoingEdgeId = getLastOutgoingBlue(baseEdge.vertex);
            }
        }

        // flip incoming red edges on right vertex and recolor last one (lowest to highest order)
        {
            int firstIncomingEdgeId = getFirstIncomingRed(endEdge.vertex);
            while (getLastIncomingRed(endEdge.vertex) != firstIncomingEdgeId) {

                flipEdgeDiagonally(firstIncomingEdgeId, false);
                firstIncomingEdgeId = getFirstIncomingRed(endEdge.vertex);
            }
            // Last flipped edge recolor and flip in the other direction
            flipEdgeDiagonally(firstIncomingEdgeId, true);
            flipEdgeColor(firstIncomingEdgeId);

        }

    } // if the left vertex needs to go to the top
    else if (baseVertex.vertical_order_index > endVertex.vertical_order_index) {
        // flip outgoing red edges on right vertex and recolor last one (highest to lowest order)
        {
            int lastOutgoingEdgeId = getLastOutgoingRed(endEdge.vertex);
            if (m_halfEdges[getPreviousCyclicEdge(m_halfEdges[lastOutgoingEdgeId].twin)].color == RED) {
                while (getFirstOutgoingRed(endEdge.vertex) != lastOutgoingEdgeId) {
                    //std::cout << "flip outred of right vertex" << std::endl;
                    flipEdgeDiagonally(lastOutgoingEdgeId, true);
                    lastOutgoingEdgeId = getLastOutgoingRed(endEdge.vertex);
                }
                // Last flipped edge recolor and flip in the other direction
                //std::cout << "flip outred of right vertex and recolor (last one)" << std::endl;

                flipEdgeDiagonally(lastOutgoingEdgeId, true);
                flipEdgeColor(lastOutgoingEdgeId);
            }
        }

        int firstIncomingEdgeId = getFirstIncomingBlue(endEdge.vertex);
        while (firstIncomingEdgeId != endEdgeId) {
            //std::cout << "should not happen" << endl;
            flipEdgeDiagonally(firstIncomingEdgeId, false);
            firstIncomingEdgeId = getFirstIncomingBlue(endEdge.vertex);
        }

        // flip all incoming red edges of the left vertex
        {
            int lastOutgoingId = getLastIncomingRed(baseEdge.vertex);
            while (getFirstIncomingRed(baseEdge.vertex) != lastOutgoingId) {
                //std::cout << "flip inred of left vertex" << std::endl;

                flipEdgeDiagonally(lastOutgoingId, true);
                lastOutgoingId = getLastIncomingRed(baseEdge.vertex);
            }
            //std::cout << "flip inred of left vertex and recolor (last one)" << std::endl;
            flipEdgeDiagonally(lastOutgoingId, true);
            flipEdgeColor(lastOutgoingId);
        }

        // when the bottom vertex goes to the right of the top, the edge dir needs to be reverted
        revertEdgeDirection(baseEdgeId);
    }

    // flip color of edge that needs to merge
    flipEdgeColor(baseEdgeId);

    return true;
}

bool RegularEdgeLabeling::mergeHighestBlueEdge(int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        cerr << "Invalid edgeId " << edgeId << endl;
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        cerr << "Invalid twinEdge id " << twinId << endl;
        return false;
    }

    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }

    HalfEdge &baseEdge = m_halfEdges[baseEdgeId];
    HalfEdge &endEdge = m_halfEdges[endEdgeId];
    Vertex &baseVertex = m_vertices[baseEdge.vertex];
    Vertex &endVertex = m_vertices[endEdge.vertex];

    if (baseVertex.vertical_order_index > endVertex.vertical_order_index) {
        // flip all incoming red edges of the left vertex (right to left order)
        {
            int lastIncomingEdgeId = getLastIncomingRed(baseEdge.vertex);
            if (m_halfEdges[getPreviousCyclicEdge(m_halfEdges[lastIncomingEdgeId].twin)].color == RED) {
                while (getFirstIncomingRed(baseEdge.vertex) != lastIncomingEdgeId) {
                    flipEdgeDiagonally(lastIncomingEdgeId, true);
                    lastIncomingEdgeId = getLastIncomingRed(baseEdge.vertex);
                }
                flipEdgeDiagonally(lastIncomingEdgeId, true);
                flipEdgeColor(lastIncomingEdgeId);
            }
        }

        {
            int firstOutgoingEdgeId = getFirstOutgoingBlue(baseEdge.vertex);
            while (firstOutgoingEdgeId != baseEdgeId) {
                flipEdgeDiagonally(firstOutgoingEdgeId, false);

                firstOutgoingEdgeId = getFirstOutgoingBlue(baseEdge.vertex);
            }
        }

        // flip outgoing red edges on right vertex and recolor last one (lowest to highest order)s
        {
            int lastOutgoingEdgeId = getLastOutgoingRed(endEdge.vertex);
            while (getFirstOutgoingRed(endEdge.vertex) != lastOutgoingEdgeId) {
                flipEdgeDiagonally(lastOutgoingEdgeId, true);
                lastOutgoingEdgeId = getLastOutgoingRed(endEdge.vertex);
            }
            // Last flipped edge recolor and flip in the other direction
            flipEdgeDiagonally(lastOutgoingEdgeId, true);
            flipEdgeColor(lastOutgoingEdgeId);
        }

        revertEdgeDirection(baseEdgeId);

    } else if (baseVertex.vertical_order_index < endVertex.vertical_order_index) {
        // flip incoming red edges on right vertex and recolor last one (highest to lowest order)
        {
            int firstIncomingEdgeId = getFirstIncomingRed(endEdge.vertex);
            if (m_halfEdges[getNextCyclicEdge(m_halfEdges[firstIncomingEdgeId].twin)].color == RED) {

                while (getLastIncomingRed(endEdge.vertex) != firstIncomingEdgeId) {
                    flipEdgeDiagonally(firstIncomingEdgeId, false);
                    firstIncomingEdgeId = getFirstIncomingRed(endEdge.vertex);
                }
                // Last flipped edge recolor and flip in the other direction
                flipEdgeDiagonally(firstIncomingEdgeId, true);
                flipEdgeColor(firstIncomingEdgeId);
            }
        }

        {
            int lastIncomingEdgeId = getLastIncomingBlue(endEdge.vertex);
            while (lastIncomingEdgeId != endEdgeId) {
                flipEdgeDiagonally(lastIncomingEdgeId, true);
                lastIncomingEdgeId = getLastIncomingBlue(endEdge.vertex);
            }
        }

        // flip all incoming red edges of the left vertex
        {
            int FirstOutgoingId = getFirstOutgoingRed(baseEdge.vertex);
            while (getLastOutgoingRed(baseEdge.vertex) != FirstOutgoingId) {
                flipEdgeDiagonally(FirstOutgoingId, false);
                FirstOutgoingId = getFirstOutgoingRed(baseEdge.vertex);
            }
            flipEdgeDiagonally(FirstOutgoingId, true);
            flipEdgeColor(FirstOutgoingId);
        }
    }

    flipEdgeColor(baseEdgeId);
    return true;
}

// ---------------- otherLabelOfHalfEdge ----------------
string RegularEdgeLabeling::otherLabelOfHalfEdge(int h) const {
    if (h < 0 || h >= (int)m_halfEdges.size()) return string();
    int t = m_halfEdges[h].twin;
    if (t == -1) return string();
    int other = m_halfEdges[t].vertex;
    if (other < 0 || other >= (int)m_vertices.size()) return string();
    return m_vertices[other].label;
}

static int find_position_in_vertex_incident(const std::vector<Vertex> &verts, int vIdx, int heIdx) {
    if (vIdx < 0 || vIdx >= verts.size()) return -1;
    const auto &edges = verts[vIdx].edges;
    for (int i = 0; i < edges.size(); ++i) if (edges[i] == heIdx) return i;
    return -1;
}

int RegularEdgeLabeling::getPreviousCyclicEdge(const int edgeId) const {
    const int vertexID = m_halfEdges[edgeId].vertex;
    const Vertex &v = m_vertices[vertexID];
    const int vDegree = getVertexDegree(vertexID);
    const int i = find_position_in_vertex_incident(m_vertices, vertexID, edgeId);

    return v.edges[(i + vDegree - 1) % vDegree];
}

int RegularEdgeLabeling::getNextCyclicEdge(const int edgeId) const {
    const int vertexID = m_halfEdges[edgeId].vertex;
    const Vertex &v = m_vertices[vertexID];
    const int vDegree = getVertexDegree(vertexID);
    const int i = find_position_in_vertex_incident(m_vertices, vertexID, edgeId);

    return v.edges[(i+1) % vDegree];
}

int RegularEdgeLabeling::findFirstEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const {
    const Vertex &v = m_vertices[vertexId];

    int vDegree = getVertexDegree(vertexId);

    for (int i = 0; i < vDegree; i++) {
        const HalfEdge &edge = m_halfEdges[v.edges[i]];
        const HalfEdge &prevEdge = m_halfEdges[v.edges[(i + vDegree - 1) % vDegree]];

        if (edge.color == edge_color && edge.outgoing == outgoing && (prevEdge.color != edge_color || prevEdge.outgoing != outgoing)) {
            return v.edges[i];
        }
    }
    return -1;
}

int RegularEdgeLabeling::findLastEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const {
    const Vertex &v = m_vertices[vertexId];

    int vDegree = getVertexDegree(vertexId);

    for (int i = 0; i < vDegree; i++) {
        const HalfEdge &edge = m_halfEdges[v.edges[i]];
        const HalfEdge &nextEdge = m_halfEdges[v.edges[(i+1) % vDegree]];

        if (edge.color == edge_color && edge.outgoing == outgoing && (nextEdge.color != edge_color || nextEdge.outgoing != outgoing)) {
            return v.edges[i];
        }
    }
    return -1;
}


int RegularEdgeLabeling::getFirstOutgoingBlue(const int vertexId) const {
    return findFirstEdgeOfType(vertexId, BLUE, true);
}
int RegularEdgeLabeling::getFirstIncomingBlue(const int vertexId) const {
    return findFirstEdgeOfType(vertexId, BLUE, false);
}
int RegularEdgeLabeling::getFirstOutgoingRed(const int vertexId) const {
    return findFirstEdgeOfType(vertexId, RED, true);
}
int RegularEdgeLabeling::getFirstIncomingRed(const int vertexId) const {
    return findFirstEdgeOfType(vertexId, RED, false);
}
int RegularEdgeLabeling::getLastOutgoingBlue(const int vertexId) const {
    return findLastEdgeOfType(vertexId, BLUE, true);
}
int RegularEdgeLabeling::getLastIncomingBlue(const int vertexId) const {
    return findLastEdgeOfType(vertexId, BLUE, false);
}
int RegularEdgeLabeling::getLastOutgoingRed(const int vertexId) const {
    return findLastEdgeOfType(vertexId, RED, true);
}
int RegularEdgeLabeling::getLastIncomingRed(const int vertexId) const {
    return findLastEdgeOfType(vertexId, RED, false);
}

int RegularEdgeLabeling::canonicalHalfEdge(int he) const {
    if (he < 0 || he >= (int)m_halfEdges.size()) return -1;
    const HalfEdge &h = m_halfEdges[he];
    if (h.outgoing) return he;
    int t = h.twin;
    if (t >= 0 && t < (int)m_halfEdges.size() && m_halfEdges[t].outgoing) return t;
    return he;
}

bool RegularEdgeLabeling::flipEdgeColor(const int edgeId) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) return false;
    HalfEdge &halfEdge = m_halfEdges[edgeId];
    if (halfEdge.twin < 0 || halfEdge.twin >= m_halfEdges.size()) return false;
    HalfEdge &twin = m_halfEdges[halfEdge.twin];

    if (halfEdge.color == BLUE) {
        halfEdge.color = RED;
        twin.color = RED;

        return true;
    } else if (halfEdge.color == RED) {
        halfEdge.color = BLUE;
        twin.color = BLUE;

        return true;
    }

    return false;
}

bool RegularEdgeLabeling::flipEdgeDiagonally(const int edgeId, bool clockwise) {
    if (edgeId < 0 || edgeId >= m_halfEdges.size()) {
        throw runtime_error("flipEdgeDiagonally: Invalid edgeId: " + std::to_string(edgeId));
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        throw runtime_error("flipEdgeDiagonally: Invalid twinEdgeId: " + std::to_string(twinId));
        return false;
    }

    int a = -1;
    int b = -1;
    int baseEdgeId = -1;
    int endEdgeId = -1;

    if (m_halfEdges[edgeId].outgoing) {
        a = m_halfEdges[edgeId].vertex; // origin of half-edge id
        b = m_halfEdges[twinId].vertex;
        baseEdgeId = edgeId;
        endEdgeId = twinId;
    }
    else {
        a = m_halfEdges[twinId].vertex; // origin of half-edge id
        b = m_halfEdges[edgeId].vertex;
        baseEdgeId = twinId;
        endEdgeId = edgeId;
    }
    //
    // int a = m_halfEdges[edgeId].vertex; // origin of half-edge id
    // int b = m_halfEdges[twinId].vertex; // other edge id (of twin)
    if (a < 0 || b < 0 || a >= m_vertices.size() || b >= m_vertices.size()) return false;

    const int posA = find_position_in_vertex_incident(m_vertices, a, baseEdgeId);
    const int posB = find_position_in_vertex_incident(m_vertices, b, endEdgeId);
    if (posA == -1) return false;// || posB == -1) return false;

    // previous or next half-edge around a and b in ccw order
    int cEdge = -1;
    int dEdge = -1;

    if (clockwise) {
        cEdge = getNextCyclicEdge(baseEdgeId);
        dEdge = getPreviousCyclicEdge(baseEdgeId);
    }
    else {
        cEdge = getPreviousCyclicEdge(baseEdgeId);
        dEdge = getNextCyclicEdge(baseEdgeId);
    }

    int cVertex = m_halfEdges[m_halfEdges[cEdge].twin].vertex;
    int dVertex = m_halfEdges[m_halfEdges[dEdge].twin].vertex;

    // cout << "c edges: " << endl;
    //
    // for (auto edge : m_vertices[cVertex].edges) {
    //     cout << edge << endl;
    // }
    // cout << "d edges: " << endl;
    //
    // for (auto edge : m_vertices[dVertex].edges) {
    //     cout << edge << endl;
    // }
    // cout << m_vertices[a].label << " " << m_vertices[b].label << endl;
    // cout << baseEdgeId << " " << endEdgeId << endl;
    // cout << m_vertices[cVertex].label << " " << m_vertices[dVertex].label << endl;
    //
    // cout << cEdge << " " << dEdge << endl;

    // PERFORM FLIP
    // 1) erase a and b from lists
    {
        auto &elistA = m_vertices[a].edges;
        elistA.erase(elistA.begin() + posA);
    }
    {
        auto &elistB = m_vertices[b].edges;
        elistB.erase(elistB.begin() + posB);
    }

    // 2) find insertion location in the c vertex: a->c and b->d
    int cCyclicPos = find_position_in_vertex_incident(m_vertices, cVertex, m_halfEdges[cEdge].twin);
    if (cCyclicPos == -1) return false;
    {
        auto &elistC = m_vertices[cVertex].edges;
        int insertPos = clockwise ? cCyclicPos + 1 : cCyclicPos;
        if (insertPos < 0) insertPos = 0;
        if (insertPos > elistC.size()) insertPos = elistC.size();
        elistC.insert(elistC.begin() + insertPos, baseEdgeId);
    }
    // 3) find insertion location in the d vertex: b->d
    int dCyclicPos = find_position_in_vertex_incident(m_vertices, dVertex, m_halfEdges[dEdge].twin);
    if (dCyclicPos == -1) return false;
    {
        auto &elistD = m_vertices[dVertex].edges;
        int insertPos = clockwise ? dCyclicPos : dCyclicPos + 1;
        if (insertPos < 0) insertPos = 0;
        if (insertPos > elistD.size()) insertPos = elistD.size();
        elistD.insert(elistD.begin() + insertPos, endEdgeId);
    }
    // 4) update half edge vertex references
    m_halfEdges[baseEdgeId].vertex = cVertex;
    m_halfEdges[endEdgeId].vertex = dVertex;

    // cout << "c edges: " << endl;
    //
    // for (auto edge : m_vertices[cVertex].edges) {
    //     cout << edge << endl;
    // }
    // cout << "d edges: " << endl;
    //
    // for (auto edge : m_vertices[dVertex].edges) {
    //     cout << edge << endl;
    // }

    // 5) update id string
    string originC = m_vertices [cVertex].label;
    string destD  = m_vertices[ m_halfEdges[endEdgeId].vertex ].label;
    m_halfEdges[baseEdgeId].id_str = originC + "->" + destD;

    string originD = m_vertices[ dVertex ].label;
    string destC  = m_vertices[ m_halfEdges[baseEdgeId].vertex ].label; // cVert
    m_halfEdges[endEdgeId].id_str = originD + "->" + destC;

    return true;
}

void RegularEdgeLabeling::revertEdgeDirection(int edgeId) {
    HalfEdge &edge = m_halfEdges[edgeId];
    edge.outgoing = !edge.outgoing;
    m_halfEdges[edge.twin].outgoing = !m_halfEdges[edge.twin].outgoing;
}

void RegularEdgeLabeling::debugCheckAfterFlip(int edgeId) const {
    if (edgeId < 0 || edgeId >= (int)m_halfEdges.size()) return;
    int twin = m_halfEdges[edgeId].twin;
    if (twin < 0 || twin >= (int)m_halfEdges.size()) return;
    int a = m_halfEdges[edgeId].vertex;
    int b = m_halfEdges[twin].vertex;
    std::cout << "After flip: halfEdge " << edgeId << " at vertex " << a << ", twin " << twin << " at vertex " << b << "\n";
    std::cout << "V["<<a<<"] edges:";
    for (int he : m_vertices[a].edges) std::cout << " " << he;
    std::cout << "\nV["<<b<<"] edges:";
    for (int he : m_vertices[b].edges) std::cout << " " << he;
    std::cout << "\n";
}

// ---------------- printSummary ----------------
void RegularEdgeLabeling::printSummary() const {
    cout << "Vertices: " << m_vertices.size() << ", HalfEdges: " << m_halfEdges.size() << "\n";
    for (int i = 0; i < (int)m_vertices.size(); ++i) {
        const Vertex &v = m_vertices[i];
        cout << "V["<<i<<"] '"<<v.label<< "' | weight: " << v.weight << " | incident (CCW):";
        for (int heIdx : v.edges) {
            const HalfEdge &h = m_halfEdges[heIdx];
            cout << " {#" << heIdx << " " << h.id_str
                 << (h.outgoing ? " OUT" : " IN")
                 << (h.color==RED ? " RED" : (h.color==BLUE ? " BLUE" : " BLACK"))
                 << " other=";
            string other = otherLabelOfHalfEdge(heIdx);
            if (other.empty()) cout << "nil";
            else cout << "'" << other << "'";
            cout << "}";
        }
        cout << "\n";
    }
}


void RegularEdgeLabeling::setVertexSegmentIndices(int vertexId, int leftSeg, int rightSeg, int bottomSeg, int topSeg) {
    if (vertexId < 0 || vertexId >= (int)m_vertices.size()) {
        std::cerr << "setVertexSegmentIndices: invalid vertex index " << vertexId << "\n";
        return;
    }
    Vertex &v = m_vertices[vertexId];
    v.left_segment = leftSeg;
    v.right_segment = rightSeg;
    v.bottom_segment = bottomSeg;
    v.top_segment = topSeg;
}

int RegularEdgeLabeling::getVertexLeftSegment(int v) const {
    if (v < 0 || v >= (int)m_vertices.size()) return -1;
    return m_vertices[v].left_segment;
}
int RegularEdgeLabeling::getVertexRightSegment(int v) const {
    if (v < 0 || v >= (int)m_vertices.size()) return -1;
    return m_vertices[v].right_segment;
}
int RegularEdgeLabeling::getVertexBottomSegment(int v) const {
    if (v < 0 || v >= (int)m_vertices.size()) return -1;
    return m_vertices[v].bottom_segment;
}
int RegularEdgeLabeling::getVertexTopSegment(int v) const {
    if (v < 0 || v >= (int)m_vertices.size()) return -1;
    return m_vertices[v].top_segment;
}