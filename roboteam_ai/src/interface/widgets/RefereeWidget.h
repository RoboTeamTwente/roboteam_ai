//
// Created by mrlukasbos on 15-5-19.
//

#ifndef ROBOTEAM_AI_REFEREEWIDGET_H
#define ROBOTEAM_AI_REFEREEWIDGET_H

#include <QtWidgets/QWidget>

namespace rtt {
namespace ai {
namespace interface {

class RefereeWidget : public QWidget {
    Q_OBJECT

private:
    QVBoxLayout * vLayout;
    void updateLabels();
    QWidget * contentsWidget;
public:
    explicit RefereeWidget(QWidget * parent = nullptr);

public slots:
    void updateContents();

};

} // interface
} // ai
} // rtt


#endif //ROBOTEAM_AI_REFEREEWIDGET_H
