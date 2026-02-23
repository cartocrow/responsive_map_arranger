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
#include "library/rel_map.h"
#include "library/rel_painting.h"

using json = nlohmann::json;

using namespace cartocrow;
using namespace cartocrow::renderer;

class RectangularCartogramDemo : public QMainWindow {
    Q_OBJECT

    json m_projectData;
    RELmap m_RELmap;
    std::shared_ptr<RELmap> m_RELmap_ptr;
    std::shared_ptr<RectangularDual> m_rectangularDual;

    GeometryWidget* m_renderer;
    std::shared_ptr<rel_vis::RELPainting> m_relPainting;
    std::shared_ptr<rectangular_cartogram::RectangularCartogramPainting> m_rectPainting;

    QCheckBox* m_showREL = nullptr;

    void loadData(const std::filesystem::path &dataPath);
    void processData();
public:
    RectangularCartogramDemo();
};


#endif //RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H