#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <cartocrow/core/core.h>
#include <cartocrow/core/region_map.h>
#include <nlohmann/json_fwd.hpp>
#include "geometry_types.h"

using json = nlohmann::json;

using namespace std;

enum MergeHeuristic {
    MIN_EDGE, // For all max-segments computes the direction when the least edges
    //HIGHEST_SEGMENT_LOWEST_DIR_COUNT, // For all max-segments takes the segment with most edges, and then the dir with least edges
    MIN_WEIGHT, // for all max-segments computes the direction where the sum of vertex weights is smallest
    MIN_EDGE_MIN_WEIGHT, // MIN_EDGE, but ties are resolved using MIN_WEIGHT
    MIN_MAX_PATH
};

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

    bool isDeleted = false;
};

struct Vertex {
    string label;
    double oldWeight;
    double weight;

    bool isLandRegion = true;
    bool isDeleted = false;
    // edges of the vertex in COUNTERCLOCKWISE order (combinatorial / cyclic)
    vector<int> edges;

    int left_segment = -1;
    int right_segment = -1;
    int bottom_segment = -1;
    int top_segment = -1;

    double preferred_aspect_ratio = 1.0;
    double preferred_width = 0.0;
    double preferred_height = 0.0;

    int horizontal_order_index = -1;
    int vertical_order_index = -1;

    cartocrow::Color color{255, 255, 255};
};

class RegularEdgeLabeling {
public:
    RegularEdgeLabeling() = default;
    ~RegularEdgeLabeling() = default;

    void buildFromJson(const json &j, bool useSquareAspectRatios);

    void setDataValuesFromJson(const json &j);
    void setValuesFromRegionMap(const cartocrow::RegionMap& map);
    void setMergeHeuristic(const MergeHeuristic merge_heuristic) { m_mergeHeuristic = merge_heuristic; }
    void enableAdaptiveLayout(bool enable) { m_adaptiveLayoutEnabled = enable; };
    void setThreshHoldRelaxation(double slack) { m_threshHoldRelaxation = slack; };

    bool isValidREL(bool debugging = false) const;

    void adjustToBB();

    bool lowestOfTwoIsFirst(const std::pair<double, double> &costOne, const std::pair<double, double> &costTwo) const;
    // Returns the edgeID of the lowest cost to collapse and the direction. False = from source | True = from target (e.g., false (from left), true (from right))
    std::pair<int, bool> getLowestCostMerge(std::vector<int> const &path) const;
    std::pair<double, double> mergeEdgeCountCost(int edgeId, bool fromSource) const;
    std::pair<double, double> mergeWeightCost(int edgeId, bool fromSource) const;
    std::pair<double, double> mergeEdgeWeightCost(int edgeId, bool fromSource) const;
    std::pair<double, double> mergePathCost(int edgeId, bool fromSource) const;


    const vector<Vertex> &getVertices()  const { return m_vertices; }
    void updateVertexWeight(int id, int weight) { m_vertices[id].weight = weight; }
    void normalizeVertexWeights();
    void computePreferredSizes();
    void adjustSeaRegionSizes(bool vertically, int longestPath);
    bool deleteSeaRegionIfPossible(int seaVertexID);

    bool isValidVertex(const int v) const {
        return 0 <= v && v < static_cast<int>(m_vertices.size()) && !m_vertices[v].isDeleted;
    }
    bool isValidHalfEdge(const int he) const {
        return 0 <= he && he < static_cast<int>(m_halfEdges.size()) && !m_halfEdges[he].isDeleted;
    }
    int neighborOfHalfEdge(int he) const;
    void removeIncidentEdgesToNeighbor(int vertexID, int neighborID);
    int findHalfEdgeToNeighbor(int vertexID, int neighborID) const;

    bool isOuterVertexLabel(const int v) const {return v < 4; }
    bool isInnerVertex(const int v) const { return v >= 4; }


    std::pair<double, std::vector<int>> getLongestHorizontalPath() const;
    std::pair<double, std::vector<int>> getLongestVerticalPath() const;

    bool mergeMaxHorizontalSegmentFromLeft(int edgeId);
    bool mergeMaxHorizontalSegmentFromRight(int edgeId);
    bool mergeMaxVerticalSegmentFromBottom(int edgeId);
    bool mergeMaxVerticalSegmentFromTop(int edgeId);

    bool mergeLeftMostRedEdge(int edgeId);
    bool mergeRightMostRedEdge(int edgeId);
    bool mergeLowestBlueEdge(int edgeId);
    bool mergeHighestBlueEdge(int edgeId);

    const vector<HalfEdge> &getHalfEdges() const { return m_halfEdges; }

    string otherLabelOfHalfEdge(int h) const;

    // bb box
    void setBoundingBox(const BoundingBox &bb) { m_boundingBox = bb; adjustToBB(); }
    optional<BoundingBox> getBoundingBox() const { return m_boundingBox; }
    bool hasBoundingBox() const noexcept { return static_cast<bool>(m_boundingBox); }
    void clearBoundingBox() { m_boundingBox.reset(); }

    void printSummary() const;

    void setVertexSegmentIndices(int vertexId, int leftSeg, int rightSeg, int bottomSeg, int topSeg);

    int getVertexLeftSegment(int v) const;
    int getVertexRightSegment(int v) const;
    int getVertexBottomSegment(int v) const;
    int getVertexTopSegment(int v) const;

    int getPreviousCyclicEdge(const int edgeId) const;
    int getNextCyclicEdge(const int edgeId) const;

    int getVertexDegree(const int vertexId) const { return m_vertices[vertexId].edges.size(); }
    int getVertexDegree(const Vertex& vertex) const {return vertex.edges.size(); }
    int getFirstOutgoingBlue(int vertexId) const;
    int getFirstIncomingBlue(int vertexId) const;
    int getFirstOutgoingRed(int vertexId) const;
    int getFirstIncomingRed(int vertexId) const;
    int getLastOutgoingBlue(int vertexId) const;
    int getLastIncomingBlue(int vertexId) const;
    int getLastOutgoingRed(int vertexId) const;
    int getLastIncomingRed(int vertexId) const;

    int canonicalHalfEdge(int he) const;

    bool flipEdgeColor(int edgeId);
    bool flipEdgeDiagonally(int edgeId, bool clockwise);
    void revertEdgeDirection(int edgeId);
    void debugCheckAfterFlip(int edgeId) const;

private:
    vector<Vertex> m_vertices;
    unordered_map<string, int> m_labelToIndex;
    vector<HalfEdge> m_halfEdges;

    vector<Vertex> m_initVertices;
    vector<HalfEdge> m_initHalfEdges;

    int m_initLongestVerticalPath;
    int m_initLongestHorizontalPath;

    optional<BoundingBox> m_boundingBox;

    bool m_adaptiveLayoutEnabled = true;
    double m_threshHoldRelaxation = 0.5;

    MergeHeuristic m_mergeHeuristic = MIN_EDGE;

    static string dirKey(const std::string &a, const std::string &b);
    static string undirKey(const std::string &a, const std::string &b);

    int findFirstEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const;
    int findFirstEdgeOfType(const Vertex& vertex, EdgeColor edge_color, bool outgoing) const;
    int findLastEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const;

    vector<cartocrow::Color> m_vertColors{ {166, 205, 226}, {255, 255 , 153}, {252, 190, 110}, {250, 153, 153}, {201, 177, 213}, {177, 222, 137}};

};