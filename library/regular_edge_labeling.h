#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <nlohmann/json_fwd.hpp>

using json = nlohmann::json;

using namespace std;

enum EdgeColor {
    RED = 0,
    BLUE = 1,
    BLACK = 2
};

struct HalfEdge {
    int32_t vertex = -1; //vertex index
    int32_t twin = -1; // opposite half-edge
    //int32_t next = -1; // next half edge around face
    //int32_t prev = -1; // previous half edge around face
    EdgeColor color = BLACK;
    bool outgoing;
    string id_str;
};

struct Vertex {
    string label;
    // edges of the vertex in COUNTERCLOCKWISE order (combinatorial / cyclic)
    vector<int> edges;
};

class RegularEdgeLabeling {
public:
    RegularEdgeLabeling() = default;
    ~RegularEdgeLabeling() = default;

    void buildFromJson(const json &j);

    const vector<Vertex> &getVertices() const { return m_vertices; }
    const vector<HalfEdge> &getHalfEdges() const { return m_halfEdges; }

    string otherLabelOfHalfEdge(int h) const;

    void printSummary() const;

    // New API: return the first half-edge index at vertex 'vertexIdx' (scanning
    // edges in counterclockwise order) that matches the given color & direction.
    // Returns -1 if no matching half-edge is found.
    int getFirstOutgoingBlue(int vertexIdx) const;
    int getFirstIncomingBlue(int vertexIdx) const;
    int getFirstOutgoingRed(int vertexIdx) const;
    int getFirstIncomingRed(int vertexIdx) const;

private:
    vector<Vertex> m_vertices;
    unordered_map<string, int> m_labelToIndex;
    vector<HalfEdge> m_halfEdges;

    static string dirKey(const std::string &a, const std::string &b);
    static string undirKey(const std::string &a, const std::string &b);

};