#include "rectangular_cartogram_gui.h"

#include <QApplication>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QLabel>



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
    vLayout->addWidget(inputSettings);


}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    RectangularCartogramDemo demo;
    demo.show();
    return app.exec();
}