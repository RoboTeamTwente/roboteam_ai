//
// Created by mrlukasbos on 1-2-19.
//

#include "interface/widgets/RobotsWidget.h"

#include <QScrollArea>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>

#include "interface/widgets/mainWindow.h"

namespace rtt::ai::interface {

RobotsWidget::RobotsWidget(QWidget *parent) : QWidget(parent) {
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

void RobotsWidget::updateContents(Visualizer *visualizer, rtt::world::view::WorldDataView world) {
    std::optional<rtt::world::Field> field;
    {
        auto const &[_, world] = rtt::world::World::instance();
        field = world->getField();
    }

    if (!field) {
        RTT_ERROR("Could not get field!")
        return;
    }
    auto us = world->getUs();

    // reload the widgets completely if a robot is added or removed
    // or if the amount of selected robots is not accurate
    if (VLayout->count() != static_cast<int>(world.getUs().size()) || amountOfSelectedRobots != static_cast<int>(visualizer->getSelectedRobots().size())) {
        amountOfSelectedRobots = visualizer->getSelectedRobots().size();
        MainWindow::clearLayout(VLayout);

        for (auto &robot : world.getUs()) {
            QGroupBox *groupBox = new QGroupBox("Robot " + QString::number(robot->getId()));
            groupBox->setCheckable(true);
            groupBox->setChecked(visualizer->robotIsSelected(robot->getId()));
            QObject::connect(groupBox, &QGroupBox::clicked, [=]() { visualizer->toggleSelectedRobot(robot); });
            groupBox->setLayout(createRobotGroupItem(*field, robot));
            VLayout->addWidget(groupBox);
        }
    } else {
        for (int i = 0; i < static_cast<int>(world->getUs().size()); i++) {
            if (VLayout->itemAt(i) && VLayout->itemAt(i)->widget()) {
                auto robotwidget = VLayout->itemAt(i)->widget();
                MainWindow::clearLayout(robotwidget->layout());
                delete robotwidget->layout();
                if (!robotwidget->layout()) {
                    robotwidget->setLayout(createRobotGroupItem(*field, world.getUs().at(i)));
                }
            }
        }
    }
    auto robotSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    VLayout->addSpacerItem(robotSpacer);
}

/// create a single layout with robot information for a specific robot
QVBoxLayout *RobotsWidget::createRobotGroupItem(const rtt::world::Field &field, rtt::world::view::RobotView robot) {
    auto vbox = new QVBoxLayout();

    auto absVel = robot->getVel().length();

    auto velLabel = new QLabel("vel: {x = " + QString::number(robot->getVel().x, 'G', 3) + ", y = " + QString::number(robot->getVel().y, 'g', 3) +
                               "} m/s,\n"
                               "    absolute: " +
                               QString::number(absVel, 'G', 3) + " m/s");
    velLabel->setFixedWidth(250);
    vbox->addWidget(velLabel);

    auto angleLabel = new QLabel("angle: " + QString::number(robot->getAngle(), 'g', 3) + " radians");
    angleLabel->setFixedWidth(250);
    vbox->addWidget(angleLabel);

    auto posLabel = new QLabel("pos: (x = " + QString::number(robot->getPos().x, 'g', 3) + ", y = " + QString::number(robot->getPos().y, 'g', 3) + ")");
    posLabel->setFixedWidth(250);
    vbox->addWidget(posLabel);

    auto wLabel = new QLabel("w: " + QString::number(robot->getAngularVelocity(), 'g', 3) + "rad/s");
    wLabel->setFixedWidth(250);
    vbox->addWidget(wLabel);

    return vbox;
}

}  // namespace rtt::ai::interface
