//
// Created by mrlukasbos on 7-5-19.
//

#include "VisualizationSettingsWidget.h"
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {


VisualizationSettingsWidget::VisualizationSettingsWidget(Visualizer * visualizer, QWidget * parent) {
    auto cbVLayout = new QVBoxLayout();
    MainWindow::configureCheckBox("show rolenames", cbVLayout, visualizer, SLOT(setShowRoles(bool)), Constants::STD_SHOW_ROLES());
    MainWindow::configureCheckBox("show tacticnames", cbVLayout, visualizer, SLOT(setShowTactics(bool)), Constants::STD_SHOW_TACTICS());
    MainWindow::configureCheckBox("show tacticColors", cbVLayout, visualizer, SLOT(setShowTacticColors(bool)), Constants::STD_SHOW_TACTICS_COLORS());
    MainWindow::configureCheckBox("show angles", cbVLayout, visualizer, SLOT(setShowAngles(bool)), Constants::STD_SHOW_ANGLES());
    MainWindow::configureCheckBox("show velocities", cbVLayout, visualizer, SLOT(setShowVelocities(bool)), Constants::STD_SHOW_VELOCITIES());
    MainWindow::configureCheckBox("show path for selected robots", cbVLayout, visualizer, SLOT(setShowPath(bool)), Constants::STD_SHOW_PATHS_CURRENT());
    MainWindow::configureCheckBox("show path for all robots", cbVLayout, visualizer, SLOT(setShowPathAll(bool)), Constants::STD_SHOW_PATHS_ALL());
    MainWindow::configureCheckBox("Show marker for Ball Placement", cbVLayout, visualizer, SLOT(setShowBallPlacementMarker(bool)), Constants::STD_SHOW_BALL_PLACEMENT_MARKER());
    MainWindow::configureCheckBox("show debug values in terminal", cbVLayout, visualizer, SLOT(setShowDebugValueInTerminal(bool)), Constants::STD_SHOW_DEBUG_VALUES());
    MainWindow::configureCheckBox("show passes for selected robots", cbVLayout, visualizer, SLOT(setShowAvailablePasses(bool)), Constants::STD_SHOW_AVAILABLE_PASSES());
    MainWindow::configureCheckBox("Inverse interface", cbVLayout, visualizer, SLOT(setToggleFieldDirection(bool)), false);

    auto cbVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    cbVLayout->addSpacerItem(cbVSpacer);
    this->setLayout(cbVLayout);
}



} // interface
} // ai
} // rtt