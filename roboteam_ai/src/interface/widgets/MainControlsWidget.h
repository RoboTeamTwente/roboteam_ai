//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_MAINCONTROLSWIDGET_H
#define ROBOTEAM_AI_MAINCONTROLSWIDGET_H

#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QShortcut>
#include "QLayout"
#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

class MainControlsWidget : public QWidget {
Q_OBJECT
public:
    explicit MainControlsWidget(QWidget * parent = nullptr);
private:
    QVBoxLayout* vLayout;
    QPushButton* pauseBtn;
    QPushButton* refreshBtn;
    QPushButton* refreshJsonBtn;
    QPushButton* toggleColorBtn;
    QPushButton* toggleSideBtn;
    QPushButton* haltBtn;
    QShortcut* spaceClick;

    QComboBox* select_strategy;
    QComboBox* select_keeper_strategy;
    QComboBox* select_goalie;
    QComboBox* select_ruleset;

    bool isHalted = false;

    void setToggleColorBtnLayout() const;
    void setToggleSideBtnLayout() const;

public slots:
    void toggleOurColorParam();
    void toggleOurSideParam();

};
} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINCONTROLSWIDGET_H
