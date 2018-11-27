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

void MainWindow::on_pushButton_clicked() {
std::cout << "========================================== WOWWERAIOWUERPIOAWUERPIWER" << std::endl;
}
void MainWindow::on_pushButton_2_clicked() {
    std::cout << " WOWWERAIOWUERPIOAWUERPIWER" << std::endl;

}

} // ui