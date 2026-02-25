#include "rectangular_cartogram_gui.h"

#include <QApplication>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>

#include <nlohmann/json.hpp>

#include "library/regular_edge_labeling.h"

using json = nlohmann::json;


void RectangularCartogramDemo::loadData(const std::filesystem::path &dataPath) {
    std::cout << "loading data from " << dataPath << std::endl;

    auto ext = dataPath.extension();

    if (ext != ".json") {
        std::cerr << "Cannot load data from file type " << ext << std::endl;
        return;
    }
    std::ifstream f(dataPath);
    m_projectData = json::parse(f);
    processData();

    // std::cout << m_relPtr->flipEdgeDiagonally(12, true) << std::endl;
    // m_relPtr->debugCheckAfterFlip(12);
    //
    // m_relPtr->flipEdgeColor(12);
    //
    // m_relPtr->printSummary();



    m_rectangularDual = std::make_shared<RectangularDual>();

    // CREATE RECTANGULAR DUAL
    if (!m_rectangularDual->initializeFromREL(m_rel)) {
        std::cerr << "Failed to compute rectangular dual. You might want to check for cycles" << std::endl;
    }

    // RENDERING
    RELPainting::Options relDrawingOptions;
    relDrawingOptions.drawLabels = true;
    relDrawingOptions.drawREL = m_showREL->isChecked();

    RectangularCartogramPainting::Options rectCartogramOptions;

    m_rectPainting = std::make_shared<RectangularCartogramPainting>(m_rectangularDual, m_relPtr, rectCartogramOptions);
    m_relPainting = std::make_shared<RELPainting>(m_relPtr, m_rectangularDual);

    auto constructedSTgraphs = m_rectangularDual->buildSTGraphsFromREL(*m_relPtr);

    std::cout << "ST graph construction status: " << constructedSTgraphs << std::endl;


    //    m_renderer->addPainting(m_debugPainting, "Debugging");

    m_renderer->addPainting(m_rectPainting, "RectangularCartogram");
    m_renderer->addPainting(m_relPainting, "REL");

}

void RectangularCartogramDemo::processData() {
    std::cout << "processing data" << std::endl;

    try {
        m_rel.buildFromJson(m_projectData); // will validate & throw if errors found
    } catch (const std::exception& e) {
        std::cerr << "Failed to load REL: " << e.what() << std::endl;
    }

    m_rel.printSummary();

    m_relPtr = std::make_shared<RegularEdgeLabeling>(m_rel);

}

RectangularCartogramDemo::RectangularCartogramDemo() {
    setWindowTitle("RectangularCartogramDemo");

    m_renderer = new GeometryWidget();
    m_renderer->setDrawAxes(false);
    setCentralWidget(m_renderer);

    auto* dockWidget = new QDockWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, dockWidget);
    auto* vWidget = new QWidget();
    auto* vLayout = new QVBoxLayout(vWidget);
    vLayout->setAlignment(Qt::AlignTop);
    dockWidget->setWidget(vWidget);

    auto* inputSettings = new QLabel("<h3>Input</h3>", vWidget);
    auto loadDataButton = new QPushButton("Load Data (json)");
    vLayout->addWidget(inputSettings);
    vLayout->addWidget(loadDataButton);

    auto* debugSettings = new QLabel("<h3>Debug settings</h3>", vWidget);
    m_showREL = new QCheckBox("Show REL");
    m_showREL->setChecked(true);
    vLayout->addWidget(debugSettings);
    vLayout->addWidget(m_showREL);


    connect(loadDataButton, &QPushButton::clicked, [this, loadDataButton]() {
        QString startDir = "data";
        std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select region data file"), startDir).toStdString();
        if (filePath.empty()) return;
        loadData(filePath);
        loadDataButton->setText(QString::fromStdString(filePath.filename().string()));
    });

    connect(m_showREL, &QCheckBox::toggled, [this, loadDataButton]() {
        m_relPainting->drawRel(m_showREL->isChecked());
        m_renderer->update();
    });
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    RectangularCartogramDemo demo;
    demo.show();
    return app.exec();
}