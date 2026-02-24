// RegularEdgeLabeling.cpp
// Implements RegularEdgeLabeling declared in the header you provided.
// Requires nlohmann::json single-header available in include path.

#include "regular_edge_labeling.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <stdexcept>
#include <set>
#include <algorithm>

using json = nlohmann::json;
using namespace std;

// ---------------- static helpers ----------------
string RegularEdgeLabeling::dirKey(const string &a, const string &b) {
    return a + "->" + b;
}
string RegularEdgeLabeling::undirKey(const string &a, const string &b) {
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
}

// ---------------- buildFromJson ----------------
void RegularEdgeLabeling::buildFromJson(const json &j) {
    // clear existing
    m_vertices.clear();
    m_labelToIndex.clear();
    m_halfEdges.clear();

    if (!j.contains("regions") || !j["regions"].is_array()) {
        throw runtime_error("JSON must contain 'regions' array");
    }

    // 1) create vertices for all provided region labels
    for (const auto &r : j["regions"]) {
        if (!r.contains("label")) throw runtime_error("Each region must have a 'label'");
        string lbl = r["label"].get<string>();
        if (m_labelToIndex.find(lbl) == m_labelToIndex.end()) {
            int idx = (int)m_vertices.size();
            Vertex v;
            v.label = lbl;
            m_vertices.push_back(std::move(v));
            m_labelToIndex[lbl] = idx;
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

    // 3) build undirected map and link/create twins
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

    // 6) incoming-only neighbors set for each vertex (neighbors that send to v, but v did not list them)
    vector< set<string> > incomingOnly(m_vertices.size());
    for (const auto &dkv : directedMap) {
        string key = dkv.first;
        size_t p = key.find("->");
        string a = key.substr(0,p);
        string b = key.substr(p+2);
        int bIdx = m_labelToIndex.at(b);
        bool found = false;
        for (const string &s : outNeighbors[bIdx]) if (s == a) { found = true; break; }
        if (!found) incomingOnly[bIdx].insert(a);
    }

    // 7) assemble final incident list per vertex: for each neighbor in outNeighbors (in that order),
    //    append incoming from neighbor->v (if present), then outgoing v->neighbor (if present).
    //    Then append incoming-only neighbors (sorted) at the end.
    for (int vIdx = 0; vIdx < (int)m_vertices.size(); ++vIdx) {
        vector<int> incident;
        // primary sequence: outNeighbors
        for (const string &nbr : outNeighbors[vIdx]) {
            auto itIn = incomingMap[vIdx].find(nbr);
            if (itIn != incomingMap[vIdx].end()) incident.push_back(itIn->second);
            auto itOut = directedMap.find( dirKey(m_vertices[vIdx].label, nbr) );
            if (itOut != directedMap.end()) incident.push_back(itOut->second);
        }
        // incoming-only neighbors
        for (const string &nbr : incomingOnly[vIdx]) {
            auto itIn = incomingMap[vIdx].find(nbr);
            if (itIn != incomingMap[vIdx].end()) incident.push_back(itIn->second);
        }
        m_vertices[vIdx].edges.swap(incident);
    }
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

// ---------------- printSummary ----------------
void RegularEdgeLabeling::printSummary() const {
    cout << "Vertices: " << m_vertices.size() << ", HalfEdges: " << m_halfEdges.size() << "\n";
    for (int i = 0; i < (int)m_vertices.size(); ++i) {
        const Vertex &v = m_vertices[i];
        cout << "V["<<i<<"] '"<<v.label<<"' incident:";
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