//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H

#include <QMainWindow>
#include "../utilities/Constants.h"
#include <iostream>
#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
    Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = 0);
    void updateWidget();

    private slots:

    signals:
        void btnclicked(bool click);
        void rolescheckboxClicked(bool click);
        void toggleTacticsCheckboxClicked(bool click);
    private:
        bool click = false;
    Widget wi;

};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
