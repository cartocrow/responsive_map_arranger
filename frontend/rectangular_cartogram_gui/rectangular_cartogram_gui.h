#ifndef RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H
#define RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H


#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QMainWindow>
#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_widget.h>


#include <nlohmann/json.hpp>

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

    GeometryWidget* m_renderer;
    std::shared_ptr<rel_vis::RELPainting> m_relPainting;

    QCheckBox* m_showREL = nullptr;

    void loadData(const std::filesystem::path &dataPath);
    void processData();
public:
    RectangularCartogramDemo();
};


#endif //RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H