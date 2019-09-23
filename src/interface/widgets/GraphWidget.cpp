//
// Created by mrlukasbos on 22-9-19.
//

#include <include/roboteam_ai/interface/api/Input.h>
#include "interface/widgets/GraphWidget.h"

namespace rtt {
    namespace ai {

        namespace interface {


GraphWidget::GraphWidget(QWidget * parent) {
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
    fpsView->chart()->setBackgroundBrush(QColor(53,53,53));
    fpsView->chart()->axisY()->setMinorGridLineColor(Qt::gray);
    fpsView->chart()->axisY()->setGridLineVisible(true);

    connect(fpsSeries, &QSplineSeries::pointAdded, [=](int index){
        qreal y = fpsSeries->at(index).y();
        qreal x = fpsSeries->at(index).x();

        if(y > fpsyMax){
            if(y> fpsyMax) fpsyMax = y;
            fpsView->chart()->axisY()->setRange(0, fpsyMax+20);
        }

        if(x< fpsxMin || x > fpsxMax){
            if(x < fpsxMin) fpsxMin = x;
            if(x> fpsxMax) fpsxMax = x;

            if (fpsxMax - fpsxMin > 30) {
                fpsxMin = fpsxMax - 30;
            }
            fpsView->chart()->axisX()->setRange(fpsxMin, fpsxMax);
        }
    });

    verticalLayout->addWidget(fpsView);


    timeView = new QChartView();
    timeSeries = Input::getCycleTimes();
    timeSeries->setName("Time per tick");

    timeView->chart()->addSeries(timeSeries);
    timeView->chart()->createDefaultAxes();
    timeView->chart()->setMinimumHeight(300);
    timeView->chart()->setTheme(QChart::ChartThemeDark);
    timeView->chart()->setBackgroundBrush(QColor(53,53,53));
    timeView->chart()->axisY()->setMinorGridLineColor(Qt::gray);
    timeView->chart()->axisY()->setGridLineVisible(true);

    QObject::connect(timeSeries, &QLineSeries::pointAdded, [=](int index){
        qreal y = timeSeries->at(index).y();
        qreal x = timeSeries->at(index).x();

        if(y > timeyMax) {
            if(y> timeyMax) timeyMax = y;
            timeSeries->chart()->axisY()->setRange(0, 30);
        }

        if(x< timexMin || x > timexMax) {
            if(x < timexMin) timexMin = x;
            if(x> timexMax) timexMax = x;

            if (timexMax - timexMin > 30) {
                timexMin = timexMax - 30;
            }
            timeView->chart()->axisX()->setRange(timexMin, timexMax);
        }

    });
    verticalLayout->addWidget(timeView);
    this->setLayout(verticalLayout);
}

void GraphWidget::updateContents() {
    std::cout << "i got " << seriesIndex << std::endl;
    fpsSeries->append(seriesIndex, Input::getFps());
    seriesIndex+=0.2;
    fpsView->chart()->createDefaultAxes();

}



}
}
}