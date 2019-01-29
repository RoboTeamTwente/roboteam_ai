//
// Created by mrlukasbos on 28-1-19.
//
#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/interface/widget.h>
#include <QtWidgets/QApplication>
#include <roboteam_ai/src/interface/mainWindow.h>

namespace rtt {
namespace ai {
namespace interface {

TEST(MainWindowTest, it_shows_the_visualizer_properly) {
    auto argc = 0;
    char **argv = 0;
    auto app = new QApplication(argc, argv); // initialize qt5

    auto window = std::make_shared<MainWindow>();
    std::shared_ptr<Visualizer> vis =  window->visualizer;

    // Initialize
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;
    rtt::ai::Field::set_field(field);

    // check if the conversion factor is calculated properly
    vis->calculateFieldSizeFactor();
    EXPECT_FLOAT_EQ(vis->factor, -6.666667); // the standard size().width() width is 100

    // check if coordinate conversions are handled properly
    Vector2 fieldPos = Vector2(0, 0);  // the middle dot on the field, in field coordinates
    Vector2 screenPos = Vector2(55, 20); // the middle dot in screen coordinates

    Vector2 convertedFieldPos = vis->toScreenPosition(fieldPos);
    EXPECT_FLOAT_EQ(convertedFieldPos.x, screenPos.x);
    EXPECT_FLOAT_EQ(convertedFieldPos.y, screenPos.y);

    Vector2 convertedScreenPos = vis->toFieldPosition(screenPos);
    EXPECT_FLOAT_EQ(convertedScreenPos.x, fieldPos.x);
    EXPECT_FLOAT_EQ(convertedScreenPos.y, fieldPos.y);

    vis->setShowAngles(true);
    vis->setShowRoles(true);
    vis->setShowBallPlacementMarker(true);
    vis->setShowPath(true);
    vis->setShowPathAll(true);
    vis->setShowTacticColors(true);
    vis->setShowTactics(true);
    vis->setShowVelocities(true);

    ASSERT_TRUE(vis->showAngles
            && vis->showRoles
            && vis->showBallPlacementMarker
            && vis->showPath
            && vis->showAllPaths
            && vis->showTacticColors
            && vis->showTactics
            && vis->showVelocities);

    vis->setShowAngles(false);
    vis->setShowRoles(false);
    vis->setShowBallPlacementMarker(false);
    vis->setShowPath(false);
    vis->setShowPathAll(false);
    vis->setShowTacticColors(false);
    vis->setShowTactics(false);
    vis->setShowVelocities(false);

    ASSERT_FALSE(vis->showAngles
            || vis->showRoles
            || vis->showBallPlacementMarker
            || vis->showPath
            || vis->showAllPaths
            || vis->showTacticColors
            || vis->showTactics
            || vis->showVelocities);
}

TEST(MainWindowTest, it_displays_main_window) {
    
}

}
}
}