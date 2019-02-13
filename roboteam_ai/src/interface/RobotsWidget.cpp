//
// Created by mrlukasbos on 1-2-19.
//

#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <roboteam_msgs/WorldRobot.h>
#include "RobotsWidget.h"
#include <QScrollArea>

namespace rtt {
namespace ai {
namespace interface {

RobotsWidget::RobotsWidget(QWidget* parent) : QWidget(parent){

    // make sure it is scrollable
    auto container = new QVBoxLayout();
    VLayout = new QVBoxLayout();
    auto scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    auto inner = new QFrame(scroll);
    inner->setLayout(VLayout);
    scroll->setWidget(inner);
    container->addWidget(scroll);
    this->setLayout(container);
}

void RobotsWidget::updateContents(Visualizer* visualizer) {
    auto us = rtt::ai::World::get_world().us;

    // reload the widgets completely if a robot is added or removed
    // or if the amount of selected robots is not accurate
    if (VLayout->count()!=static_cast<int>(us.size())
            || amountOfSelectedRobots!=static_cast<int>(visualizer->getSelectedRobots().size())) {
        amountOfSelectedRobots = visualizer->getSelectedRobots().size();
        clearLayout(VLayout);

        for (auto robot : us) {
            QGroupBox* groupBox = new QGroupBox("Robot "+QString::number(robot.id));
            groupBox->setCheckable(true);
            groupBox->setChecked(visualizer->robotIsSelected(robot));
            QObject::connect(groupBox, &QGroupBox::clicked, [=]() {
                visualizer->toggleSelectedRobot(robot.id);
            });
            groupBox->setLayout(createRobotGroupItem(robot));
            VLayout->addWidget(groupBox);
        }
    }
    else {
        for (int i = 0; i<static_cast<int>(us.size()); i++) {
            if (VLayout->itemAt(i)) {
                auto robotwidget = VLayout->itemAt(i)->widget();
                clearLayout(robotwidget->layout());
                delete robotwidget->layout();
                if (!robotwidget->layout()) {
                    robotwidget->setLayout(createRobotGroupItem(us.at(i)));
                }
            }
        }
    }
    auto robotSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    VLayout->addSpacerItem(robotSpacer);
}

/// create a single layout with robot information for a specific robot
QVBoxLayout* RobotsWidget::createRobotGroupItem(roboteam_msgs::WorldRobot robot) {
    auto vbox = new QVBoxLayout();

    auto velLabel = new QLabel(
            "vel: (x = "+QString::number(robot.vel.x, 'G', 3)+", y = "+QString::number(robot.vel.y, 'g', 3)+") m/s");
    velLabel->setFixedWidth(250);
    vbox->addWidget(velLabel);

    auto angleLabel = new QLabel("angle: "+QString::number(robot.angle, 'g', 3)+" radians");
    angleLabel->setFixedWidth(250);
    vbox->addWidget(angleLabel);

    auto posLabel = new QLabel(
            "pos: (x = "+QString::number(robot.pos.x, 'g', 3)+", y = "+QString::number(robot.pos.y, 'g', 3)+")");
    posLabel->setFixedWidth(250);
    vbox->addWidget(posLabel);

    auto wLabel = new QLabel("w: "+QString::number(robot.w, 'g', 3)+"rad/s");
    wLabel->setFixedWidth(250);
    vbox->addWidget(wLabel);

    return vbox;
}

/// delete a layout and its children
void RobotsWidget::clearLayout(QLayout* layout)
{
    QLayoutItem* item;
    while ((item = layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
}


} // interface
} // ai
} // rtt