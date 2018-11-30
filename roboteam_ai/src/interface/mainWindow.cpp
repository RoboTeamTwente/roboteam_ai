//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent) {
    setMinimumWidth(800);
    setMinimumHeight(600);

    visualizer = std::make_shared<Widget>(this);
    horizontalLayout = std::make_shared<QHBoxLayout>();
    verticalLayout = std::make_shared<QVBoxLayout>();

    // button for X
    button1 = std::make_shared<QPushButton>("button1");
    verticalLayout->addWidget(button1.get());

    // checkbox for toggling Role text
    cb_rolenames = std::make_shared<QCheckBox>("show rolenames");
    cb_rolenames->setChecked(constants::STD_SHOW_ROLES);
    verticalLayout->addWidget(cb_rolenames.get());
    QObject::connect(cb_rolenames.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowRoles(bool)));

    // checkbox for toggling Tactics text
    cb_tacticnames = std::make_shared<QCheckBox>("show tacticnames");
    cb_tacticnames->setChecked(constants::STD_SHOW_TACTICS);
    verticalLayout->addWidget(cb_tacticnames.get());
    QObject::connect(cb_tacticnames.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowTactics(bool)));

    // checkbox for toggling Tactics colors
    cb_tacticcolors = std::make_shared<QCheckBox>("show tacticColors");
    cb_tacticcolors->setChecked(constants::STD_SHOW_TACTICS_COLORS);
    verticalLayout->addWidget(cb_tacticcolors.get());
    QObject::connect(cb_tacticcolors.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowTacticColors(bool)));


    // checkbox for toggling angle indicators
    cb_angles = std::make_shared<QCheckBox>("show angles");
    cb_angles->setChecked(constants::STD_SHOW_ANGLES);
    verticalLayout->addWidget(cb_angles.get());
    QObject::connect(cb_angles.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowAngles(bool)));

    // checkbox for toggling velocity indicators
    cb_velocities = std::make_shared<QCheckBox>("show velocities");
    cb_velocities->setChecked(constants::STD_SHOW_VELOCITIES);
    verticalLayout->addWidget(cb_velocities.get());
    QObject::connect(cb_velocities.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowVelocities(bool)));

    // Spacer to nicely align buttons at the top
    vSpacer = std::make_shared<QSpacerItem>(0,10, QSizePolicy::Expanding, QSizePolicy::Expanding);
    verticalLayout->addItem(vSpacer.get());

    // main layout: left the visualizer and right the vertical layout
    horizontalLayout->addWidget(visualizer.get(), 2); // width stretch 2/3
    horizontalLayout->addLayout(verticalLayout.get(), 1); // width stretch 1/3

    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(horizontalLayout.get());
}

void MainWindow::updateWidget() {
    visualizer->update();
    button1->setText(QString::number(visualizer->getSelectedRobot().id));
}

}
}
} // interface