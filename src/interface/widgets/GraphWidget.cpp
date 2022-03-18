//
// Created by mrlukasbos on 22-9-19.
//

#include "interface/widgets/GraphWidget.h"

#include "interface/api/Input.h"

namespace rtt::ai::interface {

GraphWidget::GraphWidget(QWidget *parent) {
    auto verticalLayout = new QVBoxLayout(this);

    fpsView = new QChartView();

    fpsSeries = new QSplineSeries();
    fpsSeries->setUseOpenGL();
    fpsSeries->setColor(Qt::blue);
    fpsSeries->setName("FPS");

    fpsView->chart()->addSeries(fpsSeries);
    fpsView->chart()->createDefaultAxes();
    fpsView->chart()->setMinimumHeight(300);
    fpsView->chart()->setTheme(QChart::ChartThemeDark);
    fpsView->chart()->setBackgroundBrush(QColor(53, 53, 53));
    fpsView->chart()->axes(Qt::Horizontal).back()->setMinorGridLineColor(Qt::gray);
    fpsView->chart()->axes(Qt::Vertical).back()->setGridLineVisible(true);

    connect(dynamic_cast<const QSplineSeries *>(fpsSeries), &QSplineSeries::pointAdded, [=, this](int index) {
        qreal y = fpsSeries->at(index).y();
        qreal x = fpsSeries->at(index).x();

        if (y > fpsGraphYMax) {
            if (y > fpsGraphYMax) fpsGraphYMax = y;
            fpsView->chart()->axes(Qt::Horizontal).back()->setRange(0, fpsGraphYMax + 20);
        }

        if (x < fpsGraphXMin || x > fpsGraphXMax) {
            if (x < fpsGraphXMin) fpsGraphXMin = x;
            if (x > fpsGraphXMax) fpsGraphXMax = x;

            if (fpsGraphXMax - fpsGraphXMin > 30) {
                fpsGraphXMin = fpsGraphXMax - 30;
            }
            fpsView->chart()->axes(Qt::Vertical).back()->setRange(fpsGraphXMin, fpsGraphXMax);
        }
    });

    verticalLayout->addWidget(fpsView);
    this->setLayout(verticalLayout);
}

void GraphWidget::updateContents() {
    fpsSeries->append(seriesIndex, Input::getFps());
    seriesIndex += 0.2;
    fpsView->chart()->createDefaultAxes();
}

}  // namespace rtt::ai::interface
