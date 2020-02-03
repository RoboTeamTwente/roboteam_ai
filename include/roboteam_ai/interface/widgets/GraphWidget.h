//
// Created by mrlukasbos on 22-9-19.
//
#include <QtCharts/QtCharts>

#ifndef RTT_GRAPHWIDGET_H
#define RTT_GRAPHWIDGET_H

namespace rtt::ai::interface {

class GraphWidget : public QWidget {
    Q_OBJECT
   private:
    float seriesIndex = 0;
    float fpsGraphYMax = 0;
    float fpsGraphXMin = 0;
    float fpsGraphXMax = 0;
    QChartView *fpsView;
    QLineSeries *fpsSeries;

   public:
    explicit GraphWidget(QWidget *parent = nullptr);
   public slots:
    void updateContents();
};

}  // namespace rtt::ai::interface

#endif  // RTT_GRAPHWIDGET_H
