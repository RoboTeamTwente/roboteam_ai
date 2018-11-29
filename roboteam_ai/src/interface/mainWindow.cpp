//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include "QHBoxLayout"
#include "QPushButton"


namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {

    setMinimumWidth(800);
    setMinimumHeight(600);

    auto * horizontalLayout = new QHBoxLayout;
    auto * verticalLayout = new QVBoxLayout;

    QPushButton * button1 = new QPushButton("button1");
    QPushButton * button2 = new QPushButton("button2");

    QSpacerItem * vSpacer = new QSpacerItem(0,10, QSizePolicy::Expanding, QSizePolicy::Expanding);

    // main layout
    horizontalLayout->addWidget(&wi, 2);

    // secondary layout (sidebar)
    verticalLayout->addWidget(button1);
    verticalLayout->addWidget(button2);
    verticalLayout->addItem(vSpacer);

    horizontalLayout->addLayout(verticalLayout, 1);


    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(horizontalLayout);
}

void MainWindow::updateWidget() {
    wi.update();
}

}
}
} // interface