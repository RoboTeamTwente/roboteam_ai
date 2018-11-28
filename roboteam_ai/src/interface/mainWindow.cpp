//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include <iostream>

namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    ui->toggleRolesCheckbox->setChecked(constants::STD_SHOW_ROLES);
    ui->toggleTacticsCheckbox->setChecked(constants::STD_SHOW_ROLES);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButton_clicked() {
    std::cout << "========================================== click" << std::endl;
    click = !click;
    emit btnclicked(click);
}

void MainWindow::on_toggleRolesCheckbox_clicked(bool checked) {
    emit rolescheckboxClicked(checked);
}

void MainWindow::on_toggleTacticsCheckbox_clicked(bool checked) {
    emit toggleTacticsCheckboxClicked(checked);
}

}
}
} // interface