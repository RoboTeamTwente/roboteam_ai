//
// Created by mrlukasbos on 7-5-19.
//
#include <gtest/gtest.h>
#include <interface/api/Output.h>
#include <interface/widgets/MainControlsWidget.h>
#include <interface/widgets/mainWindow.h>
#include <interface/widgets/widget.h>

#include <QtWidgets/QApplication>

namespace w = rtt::world;

namespace rtt {
namespace ai {
namespace interface {

TEST(MainControlsWidgetTest, it_toggles_our_color_param) {
    auto widget = std::make_shared<MainControlsWidget>();

    ros::NodeHandle nh;
    std::string ourColorParam;
    nh.setParam("our_color", "yellow");

    widget->toggleOurColorParam();
    nh.getParam("our_color", ourColorParam);
    EXPECT_EQ(ourColorParam, "blue");

    // reverse it again
    widget->toggleOurColorParam();
    nh.getParam("our_color", ourColorParam);
    EXPECT_EQ(ourColorParam, "yellow");
}

TEST(MainControlsWidgetTest, it_toggles_our_side_param) {
    auto widget = std::make_shared<MainControlsWidget>();

    ros::NodeHandle nh;
    std::string ourColorParam;
    nh.setParam("our_color", "yellow");

    widget->toggleOurColorParam();
    nh.getParam("our_color", ourColorParam);
    EXPECT_EQ(ourColorParam, "blue");

    // reverse it again
    widget->toggleOurColorParam();
    nh.getParam("our_color", ourColorParam);
    EXPECT_EQ(ourColorParam, "yellow");
}

}  // namespace interface
}  // namespace ai
}  // namespace rtt