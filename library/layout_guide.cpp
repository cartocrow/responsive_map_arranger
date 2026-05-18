/*
Copyright (C) 2026  TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "layout_guide.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace cartocrow::layout_guide {

// ---------------- static helpers ----------------
static string dirKey(const string &a, const string &b) {
    return a + "->" + b;
}
static string undirKey(const string &a, const string &b) {
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
}

LayoutGuide::LayoutGuide(vector<Vertex> vertices,
                         vector<HalfEdge> halfEdges)
                        : m_vertices(vertices), m_halfEdges(halfEdges) {
}

LayoutGuide::LayoutGuide(const json &j) {
    // clear existing
    m_vertices.clear();
    m_halfEdges.clear();
    m_labelToIndex.clear();

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
        if (!m_labelToIndex.contains(lbl)) { // skip over duplicate label entries
            int idx = (int)m_vertices.size();
            Vertex v;
            v.m_label = lbl;
            v.m_area = r["weight"].get<int>();

            if (lbl.starts_with("sea_")) {
                //v.m_color = cartocrow::Color{230, 230, 230};
                v.m_seaRegion = true;
            } else {
                //v.color = m_vertColors[i % m_vertColors.size()];
                v.m_seaRegion = false;
            }
            i++;

            v.m_aspectRatio = r["preferred_aspect"].get<double>();
            m_vertices.push_back(std::move(v));
            m_labelToIndex[lbl] = idx;
        }
        else std::cout << "[WARNING] Duplicate label: " << lbl << std::endl;
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
                m_vertices[vid].m_horizontal_order_index = idx;
            } else {
                std::cerr << "[WARNING] horizontal_order contains unknown label '" << lbl << "'\n";
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
                m_vertices[vid].m_vertical_order_index = idx;
            } else {
                std::cerr << "[WARNING] vertical_order contains unknown label '" << lbl << "'\n";
            }
            ++idx;
        }
    }


    // 2) create explicit half-edges for outgoing lists, preserving per-vertex input order
    unordered_map<int, vector<int>> explicitOutPerVertex; // vertexIdx -> list of explicit halfedge indices (order)
    unordered_map<string,int> directedMap; // "A->B" -> halfedge index

    auto addOut = [&](const json &region, const string &field, EdgeLabel label) {
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
                v.m_label = toLabel;
                m_vertices.push_back(std::move(v));
                m_labelToIndex[toLabel] = newIdx;
            }
            int toIdx = m_labelToIndex.at(toLabel);

            HalfEdge he;
            he.m_vertex = fromIdx;
            he.m_twin = -1; // link later
            he.m_edgeLabel = label;
            he.m_outgoing = true;
            //he.id_str = fromLabel + "->" + toLabel;

            int heIdx = (int)m_halfEdges.size();
            m_halfEdges.push_back(he);
            directedMap[ dirKey(fromLabel, toLabel) ] = heIdx;
            explicitOutPerVertex[fromIdx].push_back(heIdx);
        }
    };

    for (const auto &r : j["regions"]) {
        addOut(r, "red_out", VERTICAL);
        addOut(r, "blue_out", HORIZONTAL);
    }

    // 3) build undirected map and link/create twins (same as before)
    unordered_map<string, pair<int,int>> undirMap; // undirKey -> (leftSlot,rightSlot) where left corresponds to smaller label
    for (const auto &kv : directedMap) {
        string key = kv.first; // "A->B"
        size_t p = key.find("->");
        string a = key.substr(0,p);
        string b = key.substr(p+2);
        string ukey = undirKey(a,b);
        if (!undirMap.contains(ukey)) undirMap[ukey] = {-1,-1};
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
            m_halfEdges[left].m_twin = right;
            m_halfEdges[right].m_twin = left;
        } else if (left != -1 && right == -1) {
            // create implicit twin attached to b
            HalfEdge twin;
            twin.m_vertex = m_labelToIndex.at(b);
            twin.m_twin = left;
            twin.m_edgeLabel = m_halfEdges[left].m_edgeLabel;
            twin.m_outgoing = false;
            //twin.id_str = b + "<-" + a;
            int twinIdx = (int)m_halfEdges.size();
            m_halfEdges.push_back(twin);
            m_halfEdges[left].m_twin = twinIdx;
        } else if (left == -1 && right != -1) {
            // create implicit twin attached to a
            HalfEdge twin;
            twin.m_vertex = m_labelToIndex.at(a);
            twin.m_twin = right;
            twin.m_edgeLabel = m_halfEdges[right].m_edgeLabel;
            twin.m_outgoing = false;
            //twin.id_str = a + "<-" + b;
            int twinIdx = (int)m_halfEdges.size();
            m_halfEdges.push_back(twin);
            m_halfEdges[right].m_twin = twinIdx;
        }
    }

    // 4) build incomingMap: for vertex v, incomingMap[v][uLabel] = halfedge idx at v representing u->v
    vector<unordered_map<string,int>> incomingMap(m_vertices.size());
    for (int hi = 0; hi < (int)m_halfEdges.size(); ++hi) {
        const HalfEdge &he = m_halfEdges[hi];
        if (!he.m_outgoing) {
            // h is an incoming-styled halfedge attached at h.vertex, coming from twin
            if (he.m_twin != -1) {
                int otherV = m_halfEdges[he.m_twin].m_vertex;
                incomingMap[he.m_vertex][ m_vertices[otherV].m_label ] = hi;
            }
        } else {
            // outgoing: h.twin corresponds to incoming at other vertex
            if (he.m_twin != -1) {
                int otherV = m_halfEdges[he.m_twin].m_vertex;
                incomingMap[ otherV ][ m_vertices[he.m_vertex].m_label ] = he.m_twin;
            }
        }
    }

    // Helper: ensure (and create if needed) an incoming half-edge at dest from src of given color.
    // Returns the index of the incoming half-edge at dest (attached to dest) representing src->dest.
    auto ensureIncomingHalfEdgeAt = [&](int srcIdx, int destIdx, EdgeLabel edgeLabel) -> int {
        const string &srcLabel = m_vertices[srcIdx].m_label;
        // if incoming already exists, return it
        auto it = incomingMap[destIdx].find(srcLabel);
        if (it != incomingMap[destIdx].end()) return it->second;

        // attempt to link to an explicit outgoing if present
        string dir = dirKey(m_vertices[srcIdx].m_label, m_vertices[destIdx].m_label);
        auto dit = directedMap.find(dir);
        if (dit != directedMap.end()) {
            int outHe = dit->second;
            // if outHe exists but hasn't a twin (should have from undir step), create twin to attach here
            if (outHe >= 0 && outHe < (int)m_halfEdges.size() && m_halfEdges[outHe].m_twin != -1) {
                int twinIdx = m_halfEdges[outHe].m_twin;
                incomingMap[destIdx][srcLabel] = twinIdx;
                return twinIdx;
            }
            // else if outHe exists but no twin, create one now and link
            if (outHe >= 0 && outHe < (int)m_halfEdges.size() && m_halfEdges[outHe].m_twin == -1) {
                HalfEdge twin;
                twin.m_vertex = destIdx;
                twin.m_twin = outHe;
                twin.m_edgeLabel = edgeLabel;
                twin.m_outgoing = false;
                //twin.id_str = m_vertices[destIdx].label + "<-" + srcLabel;
                int twinIdx = (int)m_halfEdges.size();
                m_halfEdges.push_back(twin);
                m_halfEdges[outHe].m_twin = twinIdx;
                incomingMap[destIdx][srcLabel] = twinIdx;
                return twinIdx;
            }
        }

        // last resort: create an incoming half-edge at dest (no explicit outgoing exists)
        HalfEdge twin;
        twin.m_vertex = destIdx;
        twin.m_twin = -1;
        twin.m_edgeLabel = edgeLabel;
        twin.m_outgoing = false;
        //twin.id_str = m_vertices[destIdx].label + "<-" + srcLabel;
        int twinIdx = (int)m_halfEdges.size();
        m_halfEdges.push_back(twin);
        incomingMap[destIdx][srcLabel] = twinIdx;
        return twinIdx;
    };

    // // 5) build outNeighbors list per vertex preserving input order
    // vector<vector<string>> outNeighbors(m_vertices.size());
    // for (const auto &kv : explicitOutPerVertex) {
    //     int vIdx = kv.first;
    //     for (int heIdx : kv.second) {
    //         //const string &id = m_halfEdges[heIdx].id_str; // "A->B"
    //         size_t p = id.find("->");
    //         string toLabel = id.substr(p+2);
    //         outNeighbors[vIdx].push_back(toLabel);
    //     }
    // }

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
                ensureIncomingHalfEdgeAt(srcIdx, vid, HORIZONTAL);
                prescribedBlueIn[vid].push_back(src);
            }
        }

        if (r.contains("red_in") && r["red_in"].is_array()) {
            for (const auto &entry : r["red_in"]) {
                if (!entry.is_string()) continue;
                string src = entry.get<string>();
                if (!m_labelToIndex.contains(src)) {
                    std::cerr << "buildFromJson: red_in contains unknown label '" << src << "'\n";
                    continue;
                }
                int srcIdx = m_labelToIndex.at(src);
                ensureIncomingHalfEdgeAt(srcIdx, vid, VERTICAL);
                prescribedRedIn[vid].push_back(src);
            }
        }
    }

    // 6) assemble final incident list per vertex into 4 REL blocks (prescribed order preferred)
    // Desired CCW order (user request): [ incoming blue ] [ incoming red ] [ outgoing blue ] [ outgoing red ]
    for (int vIdx = 0; vIdx < (int)m_vertices.size(); ++vIdx) {
        vector<int> incomingHorizontal;
        vector<int> incomingVertical;
        vector<int> outgoingHorizontal;
        vector<int> outgoingVertical;

        // 6a) incoming blue: prefer prescribed order if present, otherwise derive deterministically
        if (!prescribedBlueIn[vIdx].empty()) {
            for (const string &src : prescribedBlueIn[vIdx]) {
                auto it = incomingMap[vIdx].find(src);
                if (it != incomingMap[vIdx].end()) {
                    int inHe = it->second;
                    if (inHe >= 0 && inHe < (int)m_halfEdges.size() && m_halfEdges[inHe].m_edgeLabel == HORIZONTAL)
                        incomingHorizontal.push_back(inHe);
                }
            }
        } else {
            // fallback: collect incoming blue edges deterministically by source vertex index order
            for (const auto &kv : incomingMap[vIdx]) {
                int he = kv.second;
                if (he >= 0 && he < (int)m_halfEdges.size() && m_halfEdges[he].m_edgeLabel == HORIZONTAL) incomingHorizontal.push_back(he);
            }
            sort(incomingHorizontal.begin(), incomingHorizontal.end(), [&](int a, int b) {
                int sa = (m_halfEdges[a].m_twin >= 0) ? m_halfEdges[m_halfEdges[a].m_twin].m_vertex : -1;
                int sb = (m_halfEdges[b].m_twin >= 0) ? m_halfEdges[m_halfEdges[b].m_twin].m_vertex : -1;
                return sa < sb;
            });
        }

        // 6b) incoming red
        if (!prescribedRedIn[vIdx].empty()) {
            for (const string &src : prescribedRedIn[vIdx]) {
                auto it = incomingMap[vIdx].find(src);
                if (it != incomingMap[vIdx].end()) {
                    int inHe = it->second;
                    if (inHe >= 0 && inHe < (int)m_halfEdges.size() && m_halfEdges[inHe].m_edgeLabel == VERTICAL)
                        incomingVertical.push_back(inHe);
                }
            }
        } else {
            for (const auto &kv : incomingMap[vIdx]) {
                int he = kv.second;
                if (he >= 0 && he < (int)m_halfEdges.size() && m_halfEdges[he].m_edgeLabel == VERTICAL) incomingVertical.push_back(he);
            }
            sort(incomingVertical.begin(), incomingVertical.end(), [&](int a, int b) {
                int sa = (m_halfEdges[a].m_twin >= 0) ? m_halfEdges[m_halfEdges[a].m_twin].m_vertex : -1;
                int sb = (m_halfEdges[b].m_twin >= 0) ? m_halfEdges[m_halfEdges[b].m_twin].m_vertex : -1;
                return sa < sb;
            });
        }

        // 6c) outgoing lists: preserve earlier explicit order you built (explicitOutPerVertex)
        auto itExp = explicitOutPerVertex.find(vIdx);
        if (itExp != explicitOutPerVertex.end()) {
            for (int heIdx : itExp->second) {
                if (heIdx < 0 || heIdx >= (int)m_halfEdges.size()) continue;
                const HalfEdge &hOut = m_halfEdges[heIdx];
                if (hOut.m_edgeLabel == HORIZONTAL) outgoingHorizontal.push_back(heIdx);
                else if (hOut.m_edgeLabel == VERTICAL) outgoingVertical.push_back(heIdx);
            }
        }

        // final concatenation in requested order: IB | IR | OB | OR
        vector<int> incident;
        incident.reserve(incomingHorizontal.size() + incomingVertical.size() + outgoingHorizontal.size() + outgoingVertical.size());
        incident.insert(incident.end(), incomingHorizontal.begin(), incomingHorizontal.end());
        incident.insert(incident.end(), incomingVertical.begin(), incomingVertical.end());
        incident.insert(incident.end(), outgoingHorizontal.begin(), outgoingHorizontal.end());
        incident.insert(incident.end(), outgoingVertical.begin(), outgoingVertical.end());

        m_vertices[vIdx].m_edges.swap(incident);
    }
}

bool LayoutGuide::isValidREL(const bool debugging) const {
    if (!checkRelHalfEdgesConsistency()) {
        if (debugging) std::cout<<"REL invalid: half edge mismatch" << endl;;
        return false;
    }

    if (!checkCyclicEdgeTypeOrder()) {
        if (debugging) std::cout<<"REL invalid: cyclic edge type order not valid" << endl;;
        return false;
    }

    if (!isEdgeTypeAcyclic(HORIZONTAL)) {
        if (debugging) std::cout<<"REL invalid: cycle found in the HORIZONTAL edges" << endl;
        return false;
    }
    if (!isEdgeTypeAcyclic(VERTICAL)) {
        if (debugging) std::cout<<"REL invalid: cycle found in the VERTICAL edges" << endl;
        return false;
    }

    if (debugging) cout << "REL valid" << endl;

    return true;
}

// returns heID if heID is outgoing and returns its twin if heID is not outgoing
int LayoutGuide::getCanonicalHalfEdge(int const &heId) const {
    if (!isValidHalfEdge(heId)) return -1;

    const HalfEdge &he = m_halfEdges[heId];
    if (he.m_outgoing) return heId;
    const int twinID = he.m_twin;
    if (!isValidHalfEdge(twinID)) return -1;
    return twinID;

}

int LayoutGuide::getNextCyclicEdge(int const &heId) const {
    const int pos = getCanonicalHalfEdge(heId);
    const Vertex v = m_vertices[m_halfEdges[heId].m_vertex];

    return v.m_edges[(pos + v.degree() - 1) % v.degree()];
}

int LayoutGuide::getPreviousCyclicEdge(int const &heId) const {
    const int pos = getCanonicalHalfEdge(heId);
    const Vertex v = m_vertices[m_halfEdges[heId].m_vertex];

    return v.m_edges[(pos+1) % v.degree()];
}

int LayoutGuide::getCyclicPositionOfHalfEdge(int const &heId) const {
    if (!isValidHalfEdge(heId)) return -1;

    const Vertex v = m_vertices[m_halfEdges[heId].m_vertex];
    for (int i = 0; i < v.degree(); ++i)
        if (v.m_edges[i] == heId) return i;

    return -1;
}

int LayoutGuide::getFirstEdgeOfType(const int vId, const EdgeLabel edgeLabel, const bool outgoing) const {
    const Vertex &v = m_vertices[vId];

    for (int i = 0; i < v.degree(); i++) {
        const int edgeId = v.m_edges[i];
        const int prevEdgeId = getPreviousCyclicEdge(edgeId);

        const HalfEdge &edge = m_halfEdges[edgeId];
        const HalfEdge &prevEdge = m_halfEdges[prevEdgeId];

        if (edge.m_edgeLabel == edgeLabel && edge.m_outgoing == outgoing && (prevEdge.m_edgeLabel != edgeLabel || prevEdge.m_outgoing != outgoing)) {
            return v.m_edges[i];
        }
    }
    return -1;
}

int LayoutGuide::getLastEdgeOfType(const int vId, const EdgeLabel edgeLabel, const bool outgoing) const {
    const Vertex &v = m_vertices[vId];

    for (int i = 0; i < v.degree(); i++) {
        const int edgeId = v.m_edges[i];
        const int nextEdgeId = getNextCyclicEdge(edgeId);

        const HalfEdge &edge = m_halfEdges[edgeId];
        const HalfEdge &nextEdge = m_halfEdges[nextEdgeId];

        if (edge.m_edgeLabel == edgeLabel && edge.m_outgoing == outgoing && (nextEdge.m_edgeLabel != edgeLabel || nextEdge.m_outgoing != outgoing)) {
            return v.m_edges[i];
        }
    }
    return -1;
}

bool LayoutGuide::flipEdgeColor(int const &heID) {
    if (!isValidHalfEdge(heID)) return false;
    HalfEdge &he = m_halfEdges[heID];
    if (!isValidHalfEdge(he.m_twin)) return false;
    HalfEdge &twin = m_halfEdges[he.m_twin];

    if (he.m_edgeLabel == HORIZONTAL) {
        he.m_edgeLabel = VERTICAL;
        twin.m_edgeLabel = HORIZONTAL;
        return true;
    }
    if (he.m_edgeLabel == VERTICAL) {
        he.m_edgeLabel = HORIZONTAL;
        twin.m_edgeLabel = VERTICAL;
        return true;
    }
    return false;
}

bool LayoutGuide::flipEdgeDiagonally(int const &heId, bool clockwise) {
    const int baseHeId = getCanonicalHalfEdge(heId);
    if (!isValidHalfEdge(baseHeId))
        throw runtime_error("flipEdgeDiagonally: Invalid edgeID: " + to_string(heId));

    HalfEdge &baseHe = m_halfEdges[baseHeId];
    int endHeId = baseHe.m_twin;

    if (!isValidHalfEdge(endHeId))
        throw runtime_error("flipEdgeDiagonally: Invalid edgeID: " + to_string(heId));

    HalfEdge &endHe = m_halfEdges[endHeId];

    const int baseVertexId = baseHe.m_vertex;
    const int endVertexId = endHe.m_vertex;
    Vertex& baseVertex = m_vertices[baseVertexId];
    Vertex& endVertex = m_vertices[endVertexId];

    if (!isValidVertex(baseVertexId) || !isValidVertex(endVertexId)) return false;

    const int basePos = getCyclicPositionOfHalfEdge(baseHeId);
    const int endPos = getCyclicPositionOfHalfEdge(endVertexId);

    // Get new base and en
    int targetBaseHe = -1;
    int targetEndHe = -1;
    if (clockwise) {
        targetBaseHe = getNextCyclicEdge(baseHeId);
        targetEndHe = getPreviousCyclicEdge(baseHeId);

    } else {
        targetBaseHe = getPreviousCyclicEdge(baseHeId);
        targetEndHe = getNextCyclicEdge(baseHeId);
    }

    int targetBaseVertexId = m_halfEdges[m_halfEdges[targetBaseHe].m_twin].m_vertex;
    int targetEndVertexId = m_halfEdges[m_halfEdges[targetEndHe].m_twin].m_vertex;
    Vertex& targetBaseVertex = m_vertices[targetBaseVertexId];
    Vertex& targetEndVertex = m_vertices[targetEndVertexId];

    // erase edges from original vertices edge list
    baseVertex.m_edges.erase(baseVertex.m_edges.begin() + basePos);
    endVertex.m_edges.erase(endVertex.m_edges.begin() + endPos);

    // insert he into target base/end vertices
    int baseInsertPos = getCyclicPositionOfHalfEdge(targetBaseHe);
    if (clockwise) baseInsertPos++;
    baseInsertPos = clamp(baseInsertPos, 0, targetBaseVertex.degree()); // -1 means we insert at the beginning
    targetBaseVertex.m_edges.insert(targetBaseVertex.m_edges.begin() + baseInsertPos, endHeId);

    int endInsertPos = getCyclicPositionOfHalfEdge(targetEndHe);
    if (!clockwise) endInsertPos++;
    endInsertPos = clamp(endInsertPos, 0, targetEndVertex.degree()); // -1 means we insert at the beginning
    targetEndVertex.m_edges.insert(targetEndVertex.m_edges.begin() + endInsertPos, endHeId);

    // update half edge vertex references
    m_halfEdges[baseHeId].m_vertex = targetBaseVertexId;
    m_halfEdges[endHeId].m_vertex = targetEndVertexId;

    return true;
}

bool LayoutGuide::redirectEdge(int const &heID) {
    if (!isValidHalfEdge(heID)) return false;

    HalfEdge &he = m_halfEdges[heID];
    he.m_outgoing = !he.m_outgoing;

    HalfEdge &twin = m_halfEdges[he.m_twin];
    twin.m_outgoing = !twin.m_outgoing;

    return true;
}

bool LayoutGuide::checkRelHalfEdgesConsistency() const {
    for (HalfEdge he : m_halfEdges) {
        const Vertex baseVertex = m_vertices[he.m_vertex];
        if (!isValidHalfEdge(he.m_twin)) {
            std::cerr << "Invalid REL: twin edge " << he.m_twin
            << " is an invalid twin-he id of the he incident to vertex " << baseVertex.m_label << std::endl;
            return false;
        }

        const HalfEdge twin = m_halfEdges[he.m_twin];
        const Vertex endVertex = m_vertices[twin.m_vertex];

        if (he.m_edgeLabel != twin.m_edgeLabel) {
            std::cerr << "Invalid REL: inconsistent edge labels for half edges between "
            << baseVertex.m_label << " and " << endVertex.m_label << endl;
            return false;
        }

        if (he.m_outgoing == twin.m_outgoing) {
            std::cerr << "Invalid REL: inconsistent edge direction for half edges between "
            << baseVertex.m_label << " and " << endVertex.m_label << endl;
            return false;
        }
    }

    return true;
}

bool LayoutGuide::checkCyclicEdgeTypeOrder() const {
    for (Vertex v : m_vertices) {
        if (v.m_edges.empty()) {
            std::cout << "[Warning] vertex " << v.m_label << " has degree 0." << std::endl;
            continue;
        }

        vector<EdgeType> blocks;
        blocks.reserve(v.m_edges.size());

        for (const int heId : v.m_edges) {
            const HalfEdge edge = m_halfEdges[heId];
            const EdgeType type = edge.edgeType();
            if (type == NONE) {
                std::cerr << "Invalid REL: vertex " << v.m_label << " has an incident edge of EdgeType NONE" << std::endl;
                return false;
            }

            // add type to block if the last block type is different
            if (blocks.empty() || blocks.back() != type)
                blocks.push_back(type);

        }

        // remove last block if first and last block are the same
        if (blocks.size() > 1 && blocks.front() == blocks.back())
            blocks.pop_back();

        if (blocks.size() != 4) {
            std::cerr << "Invalid REL: cyclic EdgeType order of " << v.m_label << " is not correct: ";
            for (const auto b : blocks)
                std::cerr << b << " ";
            std::cerr << std::endl;
            return false;
        }

        constexpr std::array<EdgeType, 4> expected = {
            OUTGOING_VERTICAL,
            OUTGOING_HORIZONTAL,
            INCOMING_VERTICAL,
            INCOMING_HORIZONTAL
        };

        bool matches = false;

        // check whether blocks match expected order up to cyclic rotation
        for (int shift = 0; shift < 4; ++shift) {
            matches = true;
            for (int i = 0; i < 4; ++i) {
                if (blocks[i] != expected[(i + shift) % 4]) {
                    matches = false;
                    break;
                }
            }
            if (matches) {
                break;
            }
        }

        if (!matches) {
            std::cerr << "Invalid REL: cyclic EdgeType order of " << v.m_label << " is not correct: ";
            for (const auto b : blocks)
                std::cerr << b << " ";
            std::cerr << std::endl;

            return false;
        }
    }

    return true;

}

bool LayoutGuide::isEdgeTypeAcyclic(EdgeLabel edgeLabel) const {
    vector inDeg(m_vertices.size(), 0);
    vector<vector<int>> adj(m_vertices.size());

    for (int i = 0; i < m_halfEdges.size(); ++i) {
        const auto& he = m_halfEdges[i];
        if (!he.m_outgoing || he.m_edgeLabel != edgeLabel) continue;

        int u = he.m_vertex;
        int v = m_halfEdges[he.m_twin].m_vertex;

        adj[u].push_back(v);
        inDeg[v]++;
    }

    queue<int> q;
    for (int i = 0; i < m_vertices.size(); ++i) {
        if (inDeg[i] == 0) q.push(i);
    }

    int visited = 0;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        visited++;
        for (int v: adj[u]) {
            if (--inDeg[v] == 0) q.push(v);
        }
    }

    return visited == m_vertices.size();
}
} // namespace cartocrow::layout_guide
