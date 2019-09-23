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
    float fpsyMax = 0;
    float fpsxMin =0;
    float fpsxMax = 0;
    QChartView * fpsView;
    QLineSeries * fpsSeries;
public:
    explicit GraphWidget(QWidget * parent = nullptr);
public slots:
    void updateContents();
};

}
}
}

#endif //RTT_GRAPHWIDGET_H
