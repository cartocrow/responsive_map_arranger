#ifndef RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H
#define RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H


#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QMainWindow>
#include <QComboBox>
#include <cartocrow/renderer/geometry_widget.h>


#include <nlohmann/json.hpp>

#include "library/rectangular_dual.h"
#include "library/rectangular_cartogram_painting.h"
#include "library/regular_edge_labeling.h"
#include "library/rel_map.h"
#include "library/rel_painting.h"
#include "library/geometry_types.h"

#include "library/demers.h"

#include "persistent_settings.h"

using json = nlohmann::json;

using namespace cartocrow;
using namespace cartocrow::renderer;

enum CartogramType {
    RECTANGULAR_CARTOGRAM,
    DEMERS_CARTOGRAM
};

class RectangularCartogramDemo : public QMainWindow {
    Q_OBJECT

    json m_RELData;
    json m_weightData;
    RegionMap m_regionMap;
    RegularEdgeLabeling m_rel;
    std::shared_ptr<RegularEdgeLabeling> m_relPtr;
    std::shared_ptr<RectangularDual> m_rectangularDual;
    std::shared_ptr<DemersCartogram> m_demers;

    GeometryWidget* m_renderer;
    std::shared_ptr<RELPainting> m_relPainting;
    std::shared_ptr<RectangularCartogramPainting> m_rectPainting;
    std::shared_ptr<DemersPainting> m_demersPainting;

    CartogramType m_cartogramType;

    bool m_bboxDragging = false;
    Point<Inexact> m_dragStartWorld;
    BoundingBox m_bboxBeforeDrag;
    double m_bboxHandleTolerance = 40.0; // world units tolerance to hit corner (adjust)
    double m_bboxMinWidth  = 5.0;
    double m_bboxMinHeight = 5.0;

    QCheckBox* m_useAdaptiveLayout = nullptr;
    QDoubleSpinBox* m_threshHoldRelaxation = nullptr;
    QDoubleSpinBox* m_frameSizeX = nullptr;
    QDoubleSpinBox* m_frameSizeY = nullptr;
    QCheckBox* m_useSquareAspectRatios = nullptr;
    QCheckBox* m_showREL = nullptr;
    QCheckBox* m_drawLabels = nullptr;
    QCheckBox* m_showLinearOrders = nullptr;
    QComboBox* m_cartogramTypeComboBox = nullptr;
    QComboBox* m_mergeHeuristicComboBox = nullptr;

    PersistentSettings m_settings = PersistentSettings("settings");

    void loadRELData(const std::filesystem::path &dataPath);
    void loadWeightData(const std::filesystem::path &dataPath);
    void loadMap(const std::filesystem::path &mapPath);
    void processData();
    void setCartogramFromREL() const;
public:
    RectangularCartogramDemo();
};


#endif //RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H