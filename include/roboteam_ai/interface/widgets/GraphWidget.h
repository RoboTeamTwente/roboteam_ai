//
// Created by mrlukasbos on 22-9-19.
//
#include <QtCharts/QtCharts>

#ifndef RTT_GRAPHWIDGET_H
#define RTT_GRAPHWIDGET_H

namespace rtt {
namespace ai {
namespace interface {

class GraphWidget : public QWidget {
    Q_OBJECT
private:
    float seriesIndex = 0;
    float timeyMax = 0;
    float timexMin =0;
    float timexMax = 0;
    float fpsyMax = 0;
    float fpsxMin =0;
    float fpsxMax = 0;
    QChartView * timeView;
    QChartView * fpsView;
    QLineSeries * fpsSeries;
    QLineSeries * timeSeries;
public:
    explicit GraphWidget(QWidget * parent = nullptr);
public slots:
    void updateContents();
};

}
}
}

#endif //RTT_GRAPHWIDGET_H
