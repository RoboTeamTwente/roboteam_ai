//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_MAINCONTROLSWIDGET_H
#define ROBOTEAM_AI_MAINCONTROLSWIDGET_H

#include "ApplicationManager.h"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QShortcut>

#include <QLayout>
#include "widget.h"

namespace rtt::ai::interface {

class MainControlsWidget : public QWidget {
    Q_OBJECT

public:
    void updatePlays();
    explicit MainControlsWidget(QWidget *parent = nullptr, ApplicationManager *manager = nullptr);

    inline static std::atomic<bool> ignoreInvariants;

   private:
    QVBoxLayout *vLayout;
    QPushButton *pauseBtn;
    QPushButton *toggleColorBtn;
    QPushButton *toggleSideBtn;
    QPushButton *toggleSerialBtn;
    QShortcut *spaceClick;

    QComboBox *select_play;
    QComboBox *select_goalie;
    QComboBox *select_ruleset;

    ApplicationManager *manager;

    void setToggleColorBtnLayout() const;
    void setToggleSideBtnLayout() const;
    void setToggleSerialBtnLayout() const;

   public slots:
    void toggleOurColorParam();
    void toggleOurSideParam();
    void toggleSerialParam();
    void sendPauseSignal();
    void updatePause();
    void setUseReferee(bool useRef);
    void setIgnoreInvariants(bool ignore);
    void updateContents();
};
}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_MAINCONTROLSWIDGET_H
