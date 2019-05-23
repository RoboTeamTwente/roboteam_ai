//
// Created by mrlukasbos on 15-5-19.
//

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QSpacerItem>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include <QtWidgets/QLabel>
#include "RefereeWidget.h"
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

RefereeWidget::RefereeWidget(QWidget * parent) {
    vLayout = new QVBoxLayout();
    this->setLayout(vLayout);
}

void RefereeWidget::updateContents() {
    MainWindow::clearLayout(vLayout);
    updateLabels();
}

void RefereeWidget::updateLabels() {
    auto spacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);

    RuleSet ruleset = GameStateManager::getCurrentGameState().getRuleSet();

    auto velLabel = new QLabel("Max robot velocity: " + QString::number(ruleset.maxRobotVel, 'G', 3) + " m/s");
    vLayout->addWidget(velLabel);

    auto maxCollisionVelLabel = new QLabel("Max collision velocity: "+QString::number(ruleset.maxCollisionVel, 'G', 3) + " m/s");
    vLayout->addWidget(maxCollisionVelLabel);

    auto maxBallVelLabel = new QLabel("Max ball velocity: "+QString::number(ruleset.maxBallVel, 'G', 3) + " m/s");
    vLayout->addWidget(maxBallVelLabel);

    auto enterDefenseLabel = new QLabel("Robots can enter defense area: " + (ruleset.robotsCanEnterDefenseArea ? QString("true") : QString("false")));
    vLayout->addWidget(enterDefenseLabel);

    auto outOfFieldLabel = new QLabel("Robots can go out of field : " + (ruleset.robotsCanGoOutOfField ? QString("true") : QString("false")));
    vLayout->addWidget(outOfFieldLabel);

    auto touchBallLabel = new QLabel("Min distance to ball "+QString::number(ruleset.minDistanceToBall, 'G', 3) + " m");
    vLayout->addWidget(touchBallLabel);

    vLayout->addSpacerItem(spacer);
}

} // interface
} // ai
} // rtt
