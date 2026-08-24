#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>

#include "regular_edge_labeling.h"

class DorlingPainting;

struct DorlingPosition {
    using Pt = cartocrow::Point<cartocrow::Inexact>;

    cartocrow::Color color;
    std::string label;
    Pt center;
    double radius = 0.0;

    DorlingPosition(const Vertex &vertex, double x, double y, double r)
        : color(vertex.color), label(vertex.label), center(x, y), radius(r) {}
};

class DorlingCartogram {
public:
    using Pt = cartocrow::Point<cartocrow::Inexact>;

    DorlingCartogram() = default;

    void setFromREL(RegularEdgeLabeling &rel);
    void setInitializationFromMapCentroids(bool enable) { initializationFromMapCentroids = enable; }
    void setSourceMapCentroids(const std::unordered_map<std::string, Pt> &centroids,
                               const BoundingBox &boundingBox);

    int forceIterationCount = 250;
    double targetAreaFraction = 0.67;
    double adjacencyForce = 0.08;
    double maxAdjacencyForce = 2.0;
    double overlapForce = 0.35;
    double anchorForce = 0.015;
    double adjacencyPadding = 6.0;
    double boundaryPadding = 1e-6;
    double maxStepRadiusFraction = 0.35;
    double initialStepScale = 1.0;
    double minimumStepScale = 0.2;
    bool initializationFromMapCentroids = true;

    const DorlingPosition &getPosition(int index) const { return m_positions.at(index); }
    const std::vector<DorlingPosition> &positions() const { return m_positions; }

private:
    using Rect = cartocrow::Rectangle<cartocrow::Inexact>;

    struct NodeState {
        int vertexIndex = -1;
        double x = 0.0;
        double y = 0.0;
        double anchorX = 0.0;
        double anchorY = 0.0;
        double radius = 0.0;
    };

    double baseRadiusOf(const Vertex &vertex) const;
    void normalizedDirection(const NodeState &from, const NodeState &to, double &dx, double &dy, double &distance) const;
    void applyAdjacencyForces(const std::vector<std::pair<int, int>> &adjacencyPairs,
                              const std::vector<NodeState> &nodes,
                              std::vector<double> &deltaX,
                              std::vector<double> &deltaY) const;
    void applyOverlapForces(const std::vector<NodeState> &nodes,
                            std::vector<double> &deltaX,
                            std::vector<double> &deltaY) const;
    void applyAnchorForces(const std::vector<NodeState> &nodes,
                           std::vector<double> &deltaX,
                           std::vector<double> &deltaY) const;
    double averageRadius(const std::vector<NodeState> &nodes) const;
    double iterationStepScale(int iteration) const;
    void applyForcesAndClamp(std::vector<NodeState> &nodes,
                             const std::vector<double> &deltaX,
                             const std::vector<double> &deltaY,
                             const BoundingBox &bb,
                             double maxStep,
                             double stepScale) const;

    Rect m_box;
    std::vector<DorlingPosition> m_positions;
    std::unordered_map<std::string, Pt> m_sourceMapCentroids;
    BoundingBox m_sourceMapBoundingBox{};
    bool m_hasSourceMapBoundingBox = false;

    friend class DorlingPainting;
};

class DorlingPainting : public cartocrow::renderer::GeometryPainting {
public:
    using Renderer = cartocrow::renderer::GeometryRenderer;

    DorlingPainting(std::shared_ptr<DorlingCartogram> cartogram, std::shared_ptr<RegularEdgeLabeling> rel)
        : m_cartogram(std::move(cartogram)), m_rel(std::move(rel)) {}

    void paint(Renderer &renderer) const override;

    void drawLabels(bool draw) { m_drawLabels = draw; }

private:
    std::shared_ptr<DorlingCartogram> m_cartogram;
    std::shared_ptr<RegularEdgeLabeling> m_rel;
    bool m_drawLabels = true;
};
