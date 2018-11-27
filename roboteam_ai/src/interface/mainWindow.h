//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H

#include "ui_mainwindow.h"
#include <QMainWindow>

namespace ui {
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
        void on_pushButton_clicked();
        void on_pushButton_2_clicked();

private:
    Ui::MainWindow * ui;
};
}

#endif //ROBOTEAM_AI_MAINWINDOW_H
