//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_MAINCONTROLSWIDGET_H
#define ROBOTEAM_AI_MAINCONTROLSWIDGET_H

#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include "QLayout"
#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

class MainControlsWidget : public QWidget {
Q_OBJECT
public:
    explicit MainControlsWidget(QWidget * parent = nullptr);
    QString getSelectStrategyText() const;
    void setSelectStrategyText(QString text);

signals:
    void treeHasChanged();

private:
    QVBoxLayout* vLayout;
    QPushButton* haltBtn;
    QPushButton* refreshBtn;
    QPushButton* toggleColorBtn;
    QPushButton* toggleSideBtn;

    QComboBox* select_strategy;
    QComboBox* select_keeper_strategy;
    QComboBox* select_goalie;

    void setToggleColorBtnLayout() const;
    void setToggleSideBtnLayout() const;

public slots:
    void setTimeOutTop(bool top);
    void setUsesKeeper(bool usekeeper);
    void toggleOurColorParam();
    void toggleOurSideParam();
    void sendHaltSignal();
    void updatePause();
    void setUseReferee(bool useRef);
    void refreshSignal();
};
} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINCONTROLSWIDGET_H
