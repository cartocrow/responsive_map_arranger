#ifndef CARTOCROW_REL_PAINTING
#define CARTOCROW_REL_PAINTING

#include <functional>
#include <memory>
#include <array>
#include <string>
#include <optional>

#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>

#include "rel_map.h"
#include "rectangular_dual.h" // optional dependency for centroids

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace cartocrow::rel_vis {

using Inexact = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointI  = cartocrow::Point<Inexact>;
using PolygonI = cartocrow::Polygon<Inexact>;
using Renderer = renderer::GeometryRenderer;

/// Geometry painting for visualizing an REL graph using rectangle centroids (optional).
class RELPainting : public renderer::GeometryPainting {
public:
    struct Options {
        bool drawREL = true;

        // layout spacing used when RectangularDual is not provided
        Number<Inexact> horizontalSpacing = 140.0;
        Number<Inexact> verticalSpacing   = 120.0;

        // appearance
        Number<Inexact> arrowSize  = 12.0;
        Number<Inexact> strokeWidth = 1.0;

        bool drawLabels = true;
        bool drawEdgeArrowheads = true;

        // colors (RGB)
        std::array<int,3> colorRedEdge   = {220, 0, 0};
        std::array<int,3> colorBlueEdge  = {0, 100, 220};
        std::array<int,3> colorEdgeFrame = {0, 0, 0};
        std::array<int,3> colorText      = { 32, 32, 32 };
    };

    // Constructors: overload pattern to avoid default-arg with nested type
    RELPainting(std::shared_ptr<RELmap> relmap,
                std::shared_ptr<RectangularDual> dual = nullptr);

    RELPainting(std::shared_ptr<RELmap> relmap,
                std::shared_ptr<RectangularDual> dual,
                Options opts);

    void paint(Renderer &renderer) const override;

    // setters/getters
    void setOptions(Options o) { m_options = std::move(o); }
    void drawREL(bool draw) { m_options.drawREL = draw; }
    void setRectangularDual(std::shared_ptr<RectangularDual> dual) { m_dual = std::move(dual); }

private:
    std::shared_ptr<RELmap> m_relmap;
    std::shared_ptr<RectangularDual> m_dual; // optional
    Options m_options;

    // helpers
    static bool isFrameLabel(const std::string &s) {
        return s == "North" || s == "South" || s == "West" || s == "East";
    }
};

} // namespace cartocrow::rel_vis

#endif // CARTOCROW_REL_PAINTING