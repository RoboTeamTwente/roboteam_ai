//
// Created by mrlukasbos on 15-5-19.
//

#ifndef ROBOTEAM_AI_RULESETWIDGET_H
#define ROBOTEAM_AI_RULESETWIDGET_H

#include <QtWidgets/QWidget>

class QVBoxLayout;

namespace rtt {
namespace ai {
namespace interface {

class RuleSetWidget : public QWidget {
    Q_OBJECT

private:
    QVBoxLayout * vLayout;
    void updateLabels();
    QWidget * contentsWidget;
public:
    explicit RuleSetWidget(QWidget * parent = nullptr);

public slots:
    void updateContents();

};

} // interface
} // ai
} // rtt


#endif //ROBOTEAM_AI_RULESETWIDGET_H
