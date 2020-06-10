//
// Created by mrlukasbos on 15-5-19.
//

#include "interface/widgets/RuleSetWidget.h"

#include <QtWidgets/QLabel>
#include <utilities/GameStateManager.hpp>

#include "interface/widgets/mainWindow.h"

namespace rtt::ai::interface {

RuleSetWidget::RuleSetWidget(QWidget *parent) {
    vLayout = new QVBoxLayout();
    this->setLayout(vLayout);
}

void RuleSetWidget::updateContents() {
    MainWindow::clearLayout(vLayout);
    updateLabels();
}

void RuleSetWidget::updateLabels() {
    auto spacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);

    RuleSet ruleset = GameStateManager::getCurrentGameState().getRuleSet();

    auto velLabel = new QLabel("Max robot velocity: " + QString::number(ruleset.maxRobotVel, 'G', 3) + " m/s");
    vLayout->addWidget(velLabel);

    auto maxBallVelLabel = new QLabel("Max ball velocity: " + QString::number(ruleset.maxBallVel, 'G', 3) + " m/s");
    vLayout->addWidget(maxBallVelLabel);

    auto distDefenceArea = new QLabel("Min distance to defense area " + QString::number(ruleset.minDistanceToDefenseArea, 'G', 3) + " m");
    vLayout->addWidget(distDefenceArea);

    auto enterDefenseLabel = new QLabel("Robots can enter defense area: " + (ruleset.robotsCanEnterDefenseArea() ? QString("true") : QString("false")));
    vLayout->addWidget(enterDefenseLabel);

    auto outOfFieldLabel = new QLabel("Robots can go out of field : " + (ruleset.robotsCanGoOutOfField ? QString("true") : QString("false")));
    vLayout->addWidget(outOfFieldLabel);

    auto touchBallLabel = new QLabel("Min distance to ball " + QString::number(ruleset.minDistanceToBall, 'G', 3) + " m");
    vLayout->addWidget(touchBallLabel);

    vLayout->addSpacerItem(spacer);
}

}  // namespace rtt::ai::interface
