//
// Created by thijs on 17-6-19.
//

#include "CheckboxWidget.h"
#include "mainWindow.h"
#include <roboteam_ai/src/utilities/RobotDealer.h>


namespace rtt {
namespace ai {
namespace interface {

CheckboxWidget::CheckboxWidget(Visualizer* visualizer, QWidget* parent, CheckboxType checkboxType) {
    switch (checkboxType) {
    default:break;
    case visualize:
        visualizeCheckbox(visualizer, parent);
        break;
    case manualRobots:
        manualRobotsCheckbox(visualizer, parent);
        break;
    }

}

void CheckboxWidget::visualizeCheckbox(Visualizer* visualizer, QWidget* parent) {

    auto cbVLayout = new QVBoxLayout();

    MainWindow::configureCheckBox("show rolenames", cbVLayout, visualizer, SLOT(setShowRoles(bool)),
            Constants::STD_SHOW_ROLES());
    MainWindow::configureCheckBox("show tacticnames", cbVLayout, visualizer, SLOT(setShowTactics(bool)),
            Constants::STD_SHOW_TACTICS());
    MainWindow::configureCheckBox("show tacticColors", cbVLayout, visualizer, SLOT(setShowTacticColors(bool)),
            Constants::STD_SHOW_TACTICS_COLORS());
    MainWindow::configureCheckBox("show angles", cbVLayout, visualizer, SLOT(setShowAngles(bool)),
            Constants::STD_SHOW_ANGLES());
    MainWindow::configureCheckBox("show velocities", cbVLayout, visualizer, SLOT(setShowVelocities(bool)),
            Constants::STD_SHOW_VELOCITIES());
    MainWindow::configureCheckBox("show robot shortcomings", cbVLayout, visualizer, SLOT(setShowRobotInvalids(bool)),
            Constants::STD_SHOW_ROBOT_INVALIDS());
    MainWindow::configureCheckBox("Show marker for BallPtr Placement", cbVLayout, visualizer,
            SLOT(setShowBallPlacementMarker(bool)), Constants::STD_SHOW_BALL_PLACEMENT_MARKER());
    MainWindow::configureCheckBox("show debug values in terminal", cbVLayout, visualizer,
            SLOT(setShowDebugValueInTerminal(bool)), Constants::STD_SHOW_DEBUG_VALUES());
    MainWindow::configureCheckBox("Inverse interface", cbVLayout, visualizer, SLOT(setToggleFieldDirection(bool)),
            false);

    auto cbVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    cbVLayout->addSpacerItem(cbVSpacer);
    this->setLayout(cbVLayout);
}

void CheckboxWidget::manualRobotsCheckbox(Visualizer* visualizer, QWidget* parent) {

    auto cbVLayout = new QVBoxLayout();

    MainWindow::configureCheckBox("Enable All Robots", cbVLayout, visualizer, SLOT(setEnableAllManualRobots(bool)),
            true);

    for (int id = 0; id < 16; id++) {
        QString checkboxName = QString::fromStdString("Enable Robot " + std::to_string(id));

        auto checkbox = new QCheckBox(checkboxName);
        checkbox->setChecked(Constants::STD_SHOW_MANUAL_ROBOTS()[id]);
        cbVLayout->addWidget(checkbox);

        QObject::connect(checkbox, &QCheckBox::clicked, [=](bool state){

          Output::setManualRobotId(id, state);
          robotDealer::RobotDealer::refresh();
        });
    }

    auto cbVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    cbVLayout->addSpacerItem(cbVSpacer);
    this->setLayout(cbVLayout);
}

} //interface
} //ai
} //rtt