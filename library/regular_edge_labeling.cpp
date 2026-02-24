// regular_edge_labeling.cpp
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

    // 7) assemble final incident list per vertex into 4 REL blocks (preserve neighbor ordering within each block)
    //
    // Assemble incident edges into REL blocks (preserve neighbor order inside each block)
// Desired CCW order (user request): [ incoming blue ] [ incoming red ] [ outgoing blue ] [ outgoing red ]
for (int vIdx = 0; vIdx < (int)m_vertices.size(); ++vIdx) {
    vector<int> incomingBlue;
    vector<int> incomingRed;
    vector<int> outgoingBlue;
    vector<int> outgoingRed;

    // primary sequence: iterate outNeighbors in the recorded order so outgoing order is preserved
    for (const string &nbr : outNeighbors[vIdx]) {
        // incoming from neighbor -> v (if present)
        auto itIn = incomingMap[vIdx].find(nbr);
        if (itIn != incomingMap[vIdx].end()) {
            int inHe = itIn->second;
            if (inHe >= 0 && inHe < (int)m_halfEdges.size()) {
                const HalfEdge &hIn = m_halfEdges[inHe];
                if (hIn.color == BLUE) incomingBlue.push_back(inHe);
                else if (hIn.color == RED) incomingRed.push_back(inHe);
                else incomingRed.push_back(inHe); // fallback for unknown color
            }
        }

        // outgoing v -> neighbor (if present)
        auto itOut = directedMap.find(dirKey(m_vertices[vIdx].label, nbr));
        if (itOut != directedMap.end()) {
            int outHe = itOut->second;
            if (outHe >= 0 && outHe < (int)m_halfEdges.size()) {
                const HalfEdge &hOut = m_halfEdges[outHe];
                if (hOut.color == BLUE) outgoingBlue.push_back(outHe);
                else if (hOut.color == RED) outgoingRed.push_back(outHe);
                else outgoingRed.push_back(outHe); // fallback
            }
        }
    }

    // incoming-only neighbors (neighbors that send to v but v did not list them)
    // incomingOnly[vIdx] is a set<string> (sorted deterministic)
    for (const string &nbr : incomingOnly[vIdx]) {
        auto itIn = incomingMap[vIdx].find(nbr);
        if (itIn != incomingMap[vIdx].end()) {
            int inHe = itIn->second;
            if (inHe >= 0 && inHe < (int)m_halfEdges.size()) {
                const HalfEdge &hIn = m_halfEdges[inHe];
                if (hIn.color == BLUE) incomingBlue.push_back(inHe);
                else if (hIn.color == RED) incomingRed.push_back(inHe);
                else incomingRed.push_back(inHe);
            }
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

        if (edge.color == edge_color && edge.outgoing == outgoing && prevEdge.color != edge_color) {
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

        if (edge.color == edge_color && edge.outgoing == outgoing && nextEdge.color != edge_color) {
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
int RegularEdgeLabeling::getlastOutgoingBlue(const int vertexId) const {
    return findLastEdgeOfType(vertexId, BLUE, true);
}
int RegularEdgeLabeling::getlastIncomingBlue(const int vertexId) const {
    return findLastEdgeOfType(vertexId, BLUE, false);
}
int RegularEdgeLabeling::getlastOutgoingRed(const int vertexId) const {
    return findLastEdgeOfType(vertexId, RED, true);
}
int RegularEdgeLabeling::getlastIncomingRed(const int vertexId) const {
    return findLastEdgeOfType(vertexId, RED, false);
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
        cerr << "Invalid edgeId " << edgeId << endl;
        return false;
    }
    const int twinId = m_halfEdges[edgeId].twin;
    if (twinId < 0 || twinId >= m_halfEdges.size()){
        cerr << "Invalid twinEdge id " << twinId << endl;
        return false;
    }

    int a = m_halfEdges[edgeId].vertex; // origin of half-edge id
    int b = m_halfEdges[twinId].vertex; // other edge id (of twin)
    if (a < 0 || b < 0 || a >= m_vertices.size() || b >= m_vertices.size()) return false;

    const int posA = find_position_in_vertex_incident(m_vertices, a, edgeId);
    const int posB = find_position_in_vertex_incident(m_vertices, b, twinId);
    if (posA == -1 || posB == -1) return false;

    // previous or next half-edge around a and b in ccw order
    int cEdge = -1;
    int dEdge = -1;
    if (clockwise) {
        cEdge = getNextCyclicEdge(edgeId);
        dEdge = getNextCyclicEdge(twinId);
    } else {
        cEdge = getPreviousCyclicEdge(edgeId);
        dEdge = getPreviousCyclicEdge(twinId);
    }

    int cVertex = m_halfEdges[m_halfEdges[cEdge].twin].vertex;
    int dVertex = m_halfEdges[m_halfEdges[dEdge].twin].vertex;

    cout << "c edges: " << endl;

    for (auto edge : m_vertices[cVertex].edges) {
        cout << edge << endl;
    }
    cout << "d edges: " << endl;

    for (auto edge : m_vertices[dVertex].edges) {
        cout << edge << endl;
    }
    cout << m_vertices[a].label << " " << m_vertices[b].label << endl;
    cout << edgeId << " " << twinId << endl;
    cout << m_vertices[cVertex].label << " " << m_vertices[dVertex].label << endl;

    cout << cEdge << " " << dEdge << endl;

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
        int insertPos = clockwise ? cCyclicPos + 1 : cCyclicPos - 1;
        if (insertPos < 0) insertPos = 0;
        if (insertPos > elistC.size()) insertPos = elistC.size();
        elistC.insert(elistC.begin() + insertPos, edgeId);
    }
    // 3) find insertion location in the d vertex: b->d
    int dCyclicPos = find_position_in_vertex_incident(m_vertices, dVertex, m_halfEdges[dEdge].twin);
    if (dCyclicPos == -1) return false;
    {
        auto &elistD = m_vertices[dVertex].edges;
        int insertPos = clockwise ? dCyclicPos + 1 : dCyclicPos - 1;
        if (insertPos < 0) insertPos = 0;
        if (insertPos > elistD.size()) insertPos = elistD.size();
        elistD.insert(elistD.begin() + insertPos, twinId);
    }
    // 4) update half edge vertex references
    m_halfEdges[edgeId].vertex = cVertex;
    m_halfEdges[twinId].vertex = dVertex;


    cout << "c edges: " << endl;

    for (auto edge : m_vertices[cVertex].edges) {
        cout << edge << endl;
    }
    cout << "d edges: " << endl;

    for (auto edge : m_vertices[dVertex].edges) {
        cout << edge << endl;
    }

    // 5) update id string
    string originC = m_vertices [cVertex].label;
    string destD  = m_vertices[ m_halfEdges[twinId].vertex ].label;
    m_halfEdges[edgeId].id_str = originC + "->" + destD;

    string originD = m_vertices[ dVertex ].label;
    string destC  = m_vertices[ m_halfEdges[edgeId].vertex ].label; // cVert
    m_halfEdges[twinId].id_str = originD + "->" + destC;

    return true;
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
        cout << "V["<<i<<"] '"<<v.label<<"' incident (CCW):";
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

