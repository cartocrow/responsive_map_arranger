#ifndef CARTOCROW_RECTANGULAR_CARTOGRAM_PAINTING
#define CARTOCROW_RECTANGULAR_CARTOGRAM_PAINTING

#include <memory>
#include <string>
#include <array>

#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>

#include "rectangular_dual.h" // your RectangularDual class
#include "rel_map.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "demers.h"


using Inexact = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointI  = cartocrow::Point<Inexact>;
using PolygonI = cartocrow::Polygon<Inexact>;
using Renderer = cartocrow::renderer::GeometryRenderer;

/// Paint a rectangular dual / initial rectangular cartogram.
class RectangularCartogramPainting : public cartocrow::renderer::GeometryPainting {
public:
    struct Options {
        // colors (RGB 0..255)
        std::array<int,3> colorFill = {240, 240, 240};
        std::array<int,3> colorStroke = { 32,  32,  32};
        std::array<int,3> colorText = { 32,  32,  32};

        double strokeWidth = 1.0;    // stroke width for rectangle borders
        bool drawLabels = true;      // draw region labels in center
        bool drawLinearOrders = false;
        bool drawFill = true;        // fill rectangles
        bool drawStroke = true;      // stroke rectangle borders
    };

    RectangularCartogramPainting(std::shared_ptr<RectangularDual> dual,
                             std::shared_ptr<RegularEdgeLabeling> relmap = nullptr);

    RectangularCartogramPainting(std::shared_ptr<RectangularDual> dual,
                                 std::shared_ptr<RegularEdgeLabeling> relmap,
                                 Options opts);

    void paint(Renderer &renderer) const override;

    void setOptions(Options o) { m_options = std::move(o); }
    void drawLinearOrders(bool draw) { m_options.drawLinearOrders = draw; }

private:
    std::shared_ptr<RectangularDual> m_dual;
    std::shared_ptr<RegularEdgeLabeling> m_relmap; // optional - used to fetch labels, colors, weights
    Options m_options;
};


#endif // CARTOCROW_RECTANGULAR_CARTOGRAM_PAINTING