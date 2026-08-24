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
#include "library/rel_painting.h"
#include "library/geometry_types.h"

#include "library/demers.h"
#include "library/dorling.h"
#include "library/choropleth_map.h"
#include "library/centroid_vector_distortion.h"


#include "persistent_settings.h"
#include "library/layout_guide.h"

using json = nlohmann::json;

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace layout_guide;

enum CartogramType {
    RECTANGULAR_CARTOGRAM,
    DEMERS_CARTOGRAM,
    DORLING_CARTOGRAM,
    CHOROPLETH_MAP
};

class RectangularCartogramDemo : public QMainWindow {
    Q_OBJECT

    json m_RELData;
    json m_weightData;
    RegionMap m_regionMap;
    RegionCentroidMap m_regionCentroids;
    BoundingBox m_regionMapBoundingBox;
    RegularEdgeLabeling m_rel;
    std::shared_ptr<RegularEdgeLabeling> m_relPtr;
    std::shared_ptr<LayoutGuide> m_layoutGuide;
    std::shared_ptr<RectangularDual> m_rectangularDual;
    std::shared_ptr<DemersCartogram> m_demers;
    std::shared_ptr<DorlingCartogram> m_dorling;
    std::shared_ptr<ChoroplethMap> m_choroplethMap;

    GeometryWidget* m_renderer;
    std::shared_ptr<RELPainting> m_relPainting;
    std::shared_ptr<RectangularCartogramPainting> m_rectPainting;
    std::shared_ptr<DemersPainting> m_demersPainting;
    std::shared_ptr<DorlingPainting> m_dorlingPainting;
    std::shared_ptr<ChoroplethPainting> m_choroplethPainting;

    CartogramType m_cartogramType;

    bool m_bboxDragging = false;
    Point<Inexact> m_dragStartWorld;
    BoundingBox m_bboxBeforeDrag;
    double m_bboxHandleTolerance = 40.0; // world units tolerance to hit corner (adjust)
    double m_bboxMinWidth  = 5.0;
    double m_bboxMinHeight = 5.0;

    QTabWidget* m_tabs;

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
    QSpinBox* m_localMetricNeighborCount = nullptr;

    // choropleth
    QSpinBox* choroForceIterSpinBox = nullptr;
    QDoubleSpinBox* forceStepSpinBox = nullptr;
    QDoubleSpinBox* forceMaxMovementSpinBox = nullptr;
    QDoubleSpinBox* originalPosForceSpinBox = nullptr;
    QDoubleSpinBox* cartogramPosForceSpinBox = nullptr;
    QDoubleSpinBox* RELForceSpinBox = nullptr;
    QDoubleSpinBox* overlapForceSpinBox = nullptr;
    QDoubleSpinBox* boundaryForceSpinBox = nullptr;
    QCheckBox* m_useValueColorRamp = nullptr;

    // dorling
    QSpinBox* m_dorlingForceIterSpinBox = nullptr;
    QDoubleSpinBox* m_dorlingAreaFractionSpinBox = nullptr;
    QDoubleSpinBox* m_dorlingAdjacencyForceSpinBox = nullptr;
    QDoubleSpinBox* m_dorlingOverlapForceSpinBox = nullptr;
    QDoubleSpinBox* m_dorlingAnchorForceSpinBox = nullptr;
    QDoubleSpinBox* m_dorlingAdjacencyPaddingSpinBox = nullptr;
    QDoubleSpinBox* m_dorlingBoundaryPaddingSpinBox = nullptr;
    QSpinBox* m_dorlingSeparationIterationsSpinBox = nullptr;
    QCheckBox* m_dorlingUseMapCentroidInitializationCheckBox = nullptr;

    //video
    QDoubleSpinBox* m_cycleDuration = nullptr;
    QSpinBox* m_cycleCount = nullptr;
    QDoubleSpinBox* m_vidMinAspectSize = nullptr;
    QSpinBox* m_vidFPS = nullptr;
    int m_globalFrame = 0;

    PersistentSettings m_settings = PersistentSettings("settings");

    void loadRELData(const std::filesystem::path &dataPath);
    void loadWeightData(const std::filesystem::path &dataPath);
    void loadMap(const std::filesystem::path &mapPath);
    void processData();
    void setCartogramFromREL() const;
    void exportAspectRatioDeviationSweep() const;
    void exportCentroidVectorDistortionSweep() const;
    static std::string mergeHeuristicLabel(MergeHeuristic heuristic);

    void addGeneralTab();
    void addDorlingTab();
    void addChoroplethTab();
    void addVideoTab();
public:
    RectangularCartogramDemo();
};


#endif //RECTANGULAR_CARTOGRAM_RECTANGULAR_CARTOGRAM_GUI_H
