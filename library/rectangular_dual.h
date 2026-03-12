// rectangular_dual.h
#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <optional>
#include <unordered_set>

#include "regular_edge_labeling.h"


enum SegmentType {
    SEGMENT_UNKNOWN = 0,
    SEGMENT_HORIZONTAL = 1, // blue
    SEGMENT_VERTICAL = 2    // red
};

struct Segment {
    SegmentType type = SEGMENT_UNKNOWN; // blue or red
    std::vector<int> halfedges;          // half-edge indices belonging to this maximal segment
    std::vector<int> incoming_vertices;  // for horizontal: below rects, for vertical: left rects
    std::vector<int> outgoing_vertices;  // for horizontal: above rects, for vertical: right rects

    bool fixedSegment = false;
    double coord = 0.0;
    double gradientValue = 0.0;
    //coord
    //gradient value
};

class RELmap;

class RegularEdgeLabeling;
enum EdgeColor;

class RectangularDual {
public:
    struct Rect {
        double left;
        double right;
        double bottom;
        double top;

        cartocrow::Color color{255, 255, 255};

        double computeArea() {
            return (right - left) * (top - bottom);
        }
    };

    struct STGraph {
        int source = -1;
        int sink = -1;
        std::vector<std::vector<int>> out; // adjacency out: u -> list of v
        std::vector<std::vector<int>> in; // adjacency in: v -> list of u
    };

    RectangularDual() = default;

    void setFromREL(RegularEdgeLabeling& rel);

    bool hasValidSegmentCoords(RegularEdgeLabeling& rel) const;

    double computeAreaDeviation(RegularEdgeLabeling &rel);

    void fixRectangleAreas(RegularEdgeLabeling &rel);

    bool computeMaximalSegments(RegularEdgeLabeling &rel);
    std::vector<Segment> getMaximalSegments() const { return maximalSegments; };
    bool computeSegmentPositions(const RegularEdgeLabeling &rel, double cell_size = 1.0);
    bool computeRectanglesFromSegments(const RegularEdgeLabeling &rel, double cell_size = 0.0);

    void debugDumpSegment(int segId, const RegularEdgeLabeling &rel) const;
    void debugDumpVertexSegments(const RegularEdgeLabeling &rel, int v) const;

    bool buildSTandDUal(const RegularEdgeLabeling &rel) ;

    bool buildSTGraphsFromREL(const RegularEdgeLabeling &rel);
    void debugListUnassignedHalfEdges(const RegularEdgeLabeling &rel, EdgeColor color) const;

    // Build rectangular dual from RELmap (legacy)
    // cell_size: scale for each grid cell (visual size).
    bool initializeFromREL(const RELmap &rel, double cell_size = 40.0);

    // NEW: Build rectangular dual directly from RegularEdgeLabeling
    // This reads explicit outgoing half-edges and interprets:
    //   BLUE  => horizontal constraint (left -> right)
    //   RED   => vertical constraint (bottom -> top)
    bool initializeFromREL(const RegularEdgeLabeling &rel, double cell_size = 40.0);

    std::size_t size() const noexcept { return rects.size(); }
    const Rect &getRect(std::uint32_t id) const;
    const std::vector<Rect> &rectangles() const noexcept { return rects; }

private:

    // helper functions (implemented in cpp)
    bool buildDAGsFromRELmap(const RELmap &rel);
    bool buildDAGsFromRegularEdgeLabeling(const RegularEdgeLabeling &rel);

    bool topoSort(const std::vector<std::vector<std::uint32_t>> &adj,
                  std::vector<std::uint32_t> &order) const;

    // packHorizontal / packVertical compute integer indices;
    // the semantics here: leftIndex[v] = integer column index for v;
    // bottomIndex[v] = integer row index for v.
    // These are simple longest-path packers; they do not currently produce multi-column spans.
    void packHorizontal(const std::vector<std::vector<std::uint32_t>> &adj,
                        const std::vector<std::uint32_t> &topo,
                        std::vector<int> &leftIndex, int &maxRight) const;

    void packVertical(const std::vector<std::vector<std::uint32_t>> &adj,
                      const std::vector<std::uint32_t> &topo,
                      std::vector<int> &bottomIndex, int &maxTop) const;


    std::vector<Segment> maximalSegments;

    STGraph G1;
    STGraph G2;
    // dual graphs of G1 and G2: each face is represented as a vertex, and two faces with common edge have an edge in the dual.
    STGraph G1dual;
    STGraph G2Dual;

    std::vector<int> faceOfHalfEdge_G1; // size = #halfedges (or 0), -1 for not-part-of-G1
    std::vector<int> faceOfHalfEdge_G2; // same for G2
    int F1 = 0; // number of faces in G1
    int F2 = 0; // number of faces in G2

    // adjacency lists (size = number of rectangles/vertices)
    // horAdj: left -> right edges (blue)
    // verAdj: bottom -> top edges (red)
    std::vector<std::vector<std::uint32_t>> horAdj;
    std::vector<std::vector<std::uint32_t>> verAdj;

    std::vector<Rect> rects;
};