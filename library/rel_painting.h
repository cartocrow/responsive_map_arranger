#ifndef CARTOCROW_REL_PAINTING
#define CARTOCROW_REL_PAINTING

#include <functional>
#include <memory>
#include <array>
#include <string>

#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>

#include "rel_map.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace cartocrow::rel_vis {

using Inexact = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointI  = Point<Inexact>;
using PolygonI = Polygon<Inexact>;
using Renderer = renderer::GeometryRenderer;

/// Geometry painting for visualizing an REL graph.
class RELPainting : public renderer::GeometryPainting {
public:
    struct Options {
        bool drawREL = true;

        // layout
        Number<Inexact> horizontalSpacing = 140.0;
        Number<Inexact> verticalSpacing   = 120.0;

        // appearance
        Number<Inexact> nodeRadius = 12.0;
        Number<Inexact> arrowSize  = 12.0;

        bool drawLabels = true;
        bool drawNodeFill = true;
        bool drawNodeStroke = true;
        bool drawEdgeArrowheads = true;

        // colors (RGB 0..255)
        std::array<int,3> colorRedEdge   = {220, 0, 0};
        std::array<int,3> colorBlueEdge  = {0, 100, 220};
        std::array<int,3> colorNodeFill  = {255, 255, 200};
        std::array<int,3> colorNodeStroke= { 60,  60,  60};
    };

    //RELPainting(std::shared_ptr<RELmap> relmap);
    RELPainting(std::shared_ptr<RELmap> relmap, Options &&opts);

    void paint(Renderer &renderer) const override;

    void setOptions(Options o) { m_options = std::move(o); }
    const Options& options() const { return m_options; }
    void drawREL(bool draw) { m_options.drawREL = draw; }

private:
    std::shared_ptr<RELmap> m_relmap;
    Options m_options;

    // helpers
    PolygonI makeCirclePolygon(const PointI &p, Number<Inexact> r, int n = 24) const;
};

} // namespace cartocrow::rel_vis

#endif // CARTOCROW_REL_PAINTING