//
// Created by mrlukasbos on 28-1-19.
//
#include <gtest/gtest.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/interface/widget.h>
#include <QtWidgets/QApplication>
#include <roboteam_ai/src/interface/mainWindow.h>
#include <ros/node_handle.h>

namespace rtt {
namespace ai {
namespace interface {

TEST(MainWindowTest, it_shows_the_visualizer_properly) {
    auto window = std::make_shared<MainWindow>();

    std::shared_ptr<Visualizer> vis =  std::make_shared<Visualizer>(window.get());

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
    vis->setShowDebugValueInTerminal(true);
    vis->setShowPath(true);
    vis->setShowPathAll(true);
    vis->setShowTacticColors(true);
    vis->setShowTactics(true);
    vis->setShowVelocities(true);

    EXPECT_TRUE(vis->showAngles);
    EXPECT_TRUE(vis->showRoles);
    EXPECT_TRUE(vis->showBallPlacementMarker);
    EXPECT_TRUE(vis->showPath);
    EXPECT_TRUE(vis->showAllPaths);
    EXPECT_TRUE(vis->showTacticColors);
    EXPECT_TRUE(vis->showTactics);
    EXPECT_TRUE(vis->showVelocities);

    vis->setShowAngles(false);
    vis->setShowRoles(false);
    vis->setShowBallPlacementMarker(false);
    vis->setShowDebugValueInTerminal(false);
    vis->setShowPath(false);
    vis->setShowPathAll(false);
    vis->setShowTacticColors(false);
    vis->setShowTactics(false);
    vis->setShowVelocities(false);

    EXPECT_FALSE(vis->showAngles);
    EXPECT_FALSE(vis->showRoles);
    EXPECT_FALSE(vis->showBallPlacementMarker);
    EXPECT_FALSE(vis->showPath);
    EXPECT_FALSE(vis->showAllPaths);
    EXPECT_FALSE(vis->showTacticColors);
    EXPECT_FALSE(vis->showTactics);
    EXPECT_FALSE(vis->showVelocities);
    
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    robot.id = 1;
    worldMsg.us.push_back(robot);
    World::set_world(worldMsg);
    
    EXPECT_EQ(vis->getSelectedRobots().size(), 0);
    EXPECT_FALSE(vis->robotIsSelected(robot));
    
    vis->toggleSelectedRobot(1);
    EXPECT_EQ(vis->getSelectedRobots().size(), 1);
    EXPECT_TRUE(vis->robotIsSelected(robot));
    
    vis->toggleSelectedRobot(1);
    EXPECT_EQ(vis->getSelectedRobots().size(), 0);
    EXPECT_FALSE(vis->robotIsSelected(robot));
}

TEST(MainWindowTest, it_toggles_our_color_param) {
    auto window = std::make_shared<MainWindow>();

    ros::NodeHandle nh;
    std::string ourColorParam;
    nh.setParam("our_color", "yellow");

    window->toggleOurColorParam();
    nh.getParam("our_color", ourColorParam);
    EXPECT_EQ(ourColorParam, "blue");

    // reverse it again
    window->toggleOurColorParam();
    nh.getParam("our_color", ourColorParam);
    EXPECT_EQ(ourColorParam, "yellow");
}

}
}
}
