//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include <iostream>
namespace ui {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}

} // ui