//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_MAINCONTROLSWIDGET_H
#define ROBOTEAM_AI_MAINCONTROLSWIDGET_H

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QShortcut>
#include "QLayout"
#include "widget.h"

namespace rtt::ai::interface {

class MainControlsWidget : public QWidget {
 Q_OBJECT
 public:
  explicit MainControlsWidget(QWidget *parent = nullptr);

 signals:
  void treeHasChanged();

 private:
  //  QLineSeries* lineSeries;
  QVBoxLayout *vLayout;
  QPushButton *pauseBtn;
  QPushButton *toggleColorBtn;
  QPushButton *toggleSideBtn;
  QPushButton *toggleSerialBtn;
  QShortcut *spaceClick;

  QComboBox *select_strategy;
  QComboBox *select_keeper_strategy;
  QComboBox *select_goalie;
  QComboBox *select_ruleset;

  void setToggleColorBtnLayout() const;
  void setToggleSideBtnLayout() const;
  void setToggleSerialBtnLayout() const;

 public slots:
  void setTimeOutTop(bool top);
  void toggleOurColorParam();
  void toggleOurSideParam();
  void toggleSerialParam();
  void sendPauseSignal();
  void updatePause();
  void setUseReferee(bool useRef);
  void updateContents();
};
}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_MAINCONTROLSWIDGET_H
