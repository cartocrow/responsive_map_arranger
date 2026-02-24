#include <functional>
#include <memory>
#include <array>
#include <string>
#include <optional>

#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>
#include "rectangular_dual.h" // optional dependency for centroids

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

using Inexact = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointI  = cartocrow::Point<Inexact>;
using PolygonI = cartocrow::Polygon<Inexact>;
using Renderer = cartocrow::renderer::GeometryRenderer;
#include "regular_edge_labeling.h" // new dependency


class RELPainting : public cartocrow::renderer::GeometryPainting {
public:
    struct Options {
        bool drawREL = true;
        bool drawEdgeArrowheads = true;
        bool drawLabels = true;
        double horizontalSpacing = 120.0;
        double verticalSpacing = 120.0;
        double arrowSize = 8.0;
        double strokeWidth = 1.0;
        double colorRedEdge[3] = { 200, 30, 45 };
        double colorBlueEdge[3] = { 41, 128, 185 };
        double colorEdgeFrame[3] = { 44, 62, 80 };
        double colorText[3] = { 34, 34, 34 };
    };


    // New constructors: takes RegularEdgeLabeling (required) and optional dual + options
    RELPainting(std::shared_ptr<RegularEdgeLabeling> rel,
                std::shared_ptr<RectangularDual> dual = nullptr);

    RELPainting(std::shared_ptr<RegularEdgeLabeling> rel,
                std::shared_ptr<RectangularDual> dual,
                Options opts);

    // setter if you prefer to construct first and set later
    void setRegularEdgeLabeling(std::shared_ptr<RegularEdgeLabeling> rel);

    void drawRel(bool draw) { m_options.drawREL = draw; }

    // main paint entry
    void paint(Renderer &renderer) const;

private:
    std::shared_ptr<RegularEdgeLabeling> m_rel;
    std::shared_ptr<RectangularDual> m_dual;
    Options m_options;

    // helpers
    static bool isFrameLabel(const std::string &s) {
        return s == "North" || s == "South" || s == "West" || s == "East";
    }
};