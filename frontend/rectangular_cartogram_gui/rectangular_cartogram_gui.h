#ifndef RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H
#define RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H


#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QMainWindow>
#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_widget.h>


using namespace cartocrow;
using namespace cartocrow::renderer;



class RectangularCartogramDemo : public QMainWindow {
    Q_OBJECT

    GeometryWidget* m_renderer;


public:
    RectangularCartogramDemo();
};


#endif //RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H