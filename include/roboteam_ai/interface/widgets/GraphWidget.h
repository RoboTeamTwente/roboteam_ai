//
// Created by mrlukasbos on 22-9-19.
//

#ifndef RTT_GRAPHWIDGET_H
#define RTT_GRAPHWIDGET_H

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

namespace rtt::ai::interface {

class GraphWidget : public QWidget {
    Q_OBJECT
   private:
    float seriesIndex = 0;
    float fpsGraphYMax = 0;
    float fpsGraphXMin = 0;
    float fpsGraphXMax = 0;
    QtCharts::QChartView *fpsView;
    QtCharts::QLineSeries *fpsSeries;

   public:
    explicit GraphWidget(QWidget *parent = nullptr);
   public slots:
    void updateContents();
};

}  // namespace rtt::ai::interface

#endif  // RTT_GRAPHWIDGET_H
