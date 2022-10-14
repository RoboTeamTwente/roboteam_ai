//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_MAINCONTROLSWIDGET_H
#define ROBOTEAM_AI_MAINCONTROLSWIDGET_H

#include <QLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QShortcut>

#include "STPManager.h"
#include "widget.h"

namespace rtt::ai::interface {

class MainControlsWidget : public QWidget {
    Q_OBJECT

   public:
    void updatePlays();
    explicit MainControlsWidget(QWidget *parent = nullptr, STPManager *manager = nullptr);

    inline static std::atomic<bool> ignoreInvariants;

   private:
    QVBoxLayout *vLayout;
    QPushButton *pauseBtn;
    QPushButton *toggleColorBtn;
    QPushButton *toggleSideBtn;
    QPushButton *toggleRobotHubModeBtn;
    QShortcut *spaceClick;

    QComboBox *select_play;
    QComboBox *select_goalie;
    QComboBox *select_ruleset;

    STPManager *manager;

    void setToggleColorBtnLayout() const;
    void setToggleSideBtnLayout() const;
    void setToggleRobotHubModeBtnLayout() const;

   public slots:
    void toggleOurColorParam();
    void toggleOurSideParam();
    void toggleRobotHubModeParam();
    void sendPauseSignal();
    void setUseReferee(bool useRef);
    void setIgnoreInvariants(bool ignore);
    void updateContents();
};
}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_MAINCONTROLSWIDGET_H
