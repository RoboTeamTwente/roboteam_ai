//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H

#include "ui_mainwindow.h"
#include <QMainWindow>
#include "../utilities/Constants.h"

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
    Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = 0);
        ~MainWindow() override;

    private slots:
        void on_pushButton_clicked();
        void on_pushButton_2_clicked();
        void on_toggleRolesCheckbox_clicked(bool checked);
        void on_toggleTacticsCheckbox_clicked(bool checked);

    signals:
        void btnclicked(bool click);
        void rolescheckboxClicked(bool click);
        void toggleTacticsCheckboxClicked(bool click);
    private:
        bool click = false;
        Ui::MainWindow* ui;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
