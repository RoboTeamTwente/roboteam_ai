//
// Created by mrlukasbos on 1-2-19.
//

#include "interface/widgets/RobotsWidget.h"
#include <include/roboteam_ai/utilities/IOManager.h>
#include <QScrollArea>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include "analysis/GameAnalyzer.h"
#include "interface/widgets/mainWindow.h"
#include "roboteam_proto/WorldRobot.pb.h"

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

void RobotsWidget::updateContents(Visualizer *visualizer) {
    auto const &field = io::io.getField();
    auto us = rtt::ai::world::world->getUs();

    // reload the widgets completely if a robot is added or removed
    // or if the amount of selected robots is not accurate
    if (VLayout->count() != static_cast<int>(us.size()) || amountOfSelectedRobots != static_cast<int>(visualizer->getSelectedRobots().size())) {
        amountOfSelectedRobots = visualizer->getSelectedRobots().size();
        MainWindow::clearLayout(VLayout);

        for (auto &robot : us) {
            QGroupBox *groupBox = new QGroupBox("Robot " + QString::number(robot->id));
            groupBox->setCheckable(true);
            groupBox->setChecked(visualizer->robotIsSelected((*robot)));
            QObject::connect(groupBox, &QGroupBox::clicked, [=]() { visualizer->toggleSelectedRobot(robot->id); });
            groupBox->setLayout(createRobotGroupItem(field, *robot));
            VLayout->addWidget(groupBox);
        }
    } else {
        for (int i = 0; i < static_cast<int>(us.size()); i++) {
            if (VLayout->itemAt(i) && VLayout->itemAt(i)->widget()) {
                auto robotwidget = VLayout->itemAt(i)->widget();
                MainWindow::clearLayout(robotwidget->layout());
                delete robotwidget->layout();
                if (!robotwidget->layout()) {
                    robotwidget->setLayout(createRobotGroupItem(field, *us.at(i)));
                }
            }
        }
    }
    auto robotSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    VLayout->addSpacerItem(robotSpacer);
}

/// create a single layout with robot information for a specific robot
QVBoxLayout *RobotsWidget::createRobotGroupItem(const Field &field, Robot robot) {
    auto vbox = new QVBoxLayout();

    auto absVel = robot.vel.length();

    auto velLabel = new QLabel("vel: {x = " + QString::number(robot.vel.x, 'G', 3) + ", y = " + QString::number(robot.vel.y, 'g', 3) +
                               "} m/s,\n"
                               "    absolute: " +
                               QString::number(absVel, 'G', 3) + " m/s");
    velLabel->setFixedWidth(250);
    vbox->addWidget(velLabel);

    auto angleLabel = new QLabel("angle: " + QString::number(robot.angle, 'g', 3) + " radians");
    angleLabel->setFixedWidth(250);
    vbox->addWidget(angleLabel);

    auto posLabel = new QLabel("pos: (x = " + QString::number(robot.pos.x, 'g', 3) + ", y = " + QString::number(robot.pos.y, 'g', 3) + ")");
    posLabel->setFixedWidth(250);
    vbox->addWidget(posLabel);

    auto wLabel = new QLabel("w: " + QString::number(robot.angularVelocity, 'g', 3) + "rad/s");
    wLabel->setFixedWidth(250);
    vbox->addWidget(wLabel);

    auto report = rtt::ai::analysis::GameAnalyzer::getInstance().getMostRecentReport();
    if (report) {
        analysis::RobotDanger danger = report->getRobotDangerForId(robot.id, true);

        auto dangerTotalLabel = new QLabel("danger total: " + QString::number(danger.getTotalDanger(field), 'g', 3));
        dangerTotalLabel->setFixedWidth(250);
        vbox->addWidget(dangerTotalLabel);

        auto goalVisionLabel = new QLabel("goalvision: " + QString::number(danger.goalVisionPercentage, 'g', 3));
        goalVisionLabel->setFixedWidth(250);
        vbox->addWidget(goalVisionLabel);
    }

    return vbox;
}

}  // namespace rtt::ai::interface

// QT performance improvement
#include "include/roboteam_ai/interface/widgets/moc_RobotsWidget.cpp"