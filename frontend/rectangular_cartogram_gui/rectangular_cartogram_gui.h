#ifndef RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H
#define RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H


#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QMainWindow>
#include <cartocrow/renderer/geometry_widget.h>


#include <nlohmann/json.hpp>

#include "library/rectangular_dual.h"
#include "library/rectangular_cartogram_painting.h"
#include "library/regular_edge_labeling.h"
#include "library/rel_map.h"
#include "library/rel_painting.h"
#include "library/geometry_types.h"

using json = nlohmann::json;

using namespace cartocrow;
using namespace cartocrow::renderer;

class RectangularCartogramDemo : public QMainWindow {
    Q_OBJECT

    json m_projectData;
    RegularEdgeLabeling m_rel;
    std::shared_ptr<RegularEdgeLabeling> m_relPtr;
    std::shared_ptr<RectangularDual> m_rectangularDual;

    GeometryWidget* m_renderer;
    std::shared_ptr<RELPainting> m_relPainting;
    std::shared_ptr<RectangularCartogramPainting> m_rectPainting;


    bool m_bboxDragging = false;
    Point<Inexact> m_dragStartWorld;
    BoundingBox m_bboxBeforeDrag;
    double m_bboxHandleTolerance = 40.0; // world units tolerance to hit corner (adjust)
    double m_bboxMinWidth  = 20.0;
    double m_bboxMinHeight = 20.0;

    QCheckBox* m_useSquareAspectRatios = nullptr;
    QCheckBox* m_showREL = nullptr;

    void loadData(const std::filesystem::path &dataPath);
    void processData();
public:
    RectangularCartogramDemo();
};


#endif //RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H