#pragma once

#include <memory>
#include <string>
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
    DorlingCartogram() = default;

    void setFromREL(RegularEdgeLabeling &rel);

    int forceIterationCount = 250;
    double targetAreaFraction = 0.67;
    double adjacencyForce = 0.08;
    double overlapForce = 0.35;
    double anchorForce = 0.015;
    double adjacencyPadding = 6.0;
    double boundaryPadding = 1e-6;
    double maxStepRadiusFraction = 0.35;
    double initialStepScale = 1.0;
    double minimumStepScale = 0.2;

    const DorlingPosition &getPosition(int index) const { return m_positions.at(index); }
    const std::vector<DorlingPosition> &positions() const { return m_positions; }

private:
    using Rect = cartocrow::Rectangle<cartocrow::Inexact>;

    Rect m_box;
    std::vector<DorlingPosition> m_positions;

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
