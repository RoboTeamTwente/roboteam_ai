//
// Created by mrlukasbos on 7-5-19.
//

#include "interface/widgets/MainControlsWidget.h"

#include <stp/PlayDecider.hpp>
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::interface {

MainControlsWidget::MainControlsWidget(QWidget *parent, STPManager *appManager) : QWidget(parent), manager{appManager} {
    Output::setUseRefereeCommands(Constants::STD_USE_REFEREE());

    // todo: 2 dropdown menus, fix them to reflect new STP
    vLayout = new QVBoxLayout();

    pauseBtn = new QPushButton("Stop");
    QObject::connect(pauseBtn, SIGNAL(clicked()), this, SLOT(sendPauseSignal()));
    vLayout->addWidget(pauseBtn);
    pauseBtn->setStyleSheet("background-color: #cc0000;");
    pauseBtn->setMinimumHeight(40);

    spaceClick = new QShortcut(QKeySequence(Qt::Key_Space), this, SLOT(sendPauseSignal()));
    spaceClick->setAutoRepeat(false);

    auto refHorizontalLayout = new QHBoxLayout();

    // todo: the hacks for SLOTS() are unbelievable
    // functions to select strategies
    MainWindow::configureCheckBox("Use referee", refHorizontalLayout, this, SLOT(setUseReferee(bool)), Constants::STD_USE_REFEREE());
    MainWindow::configureCheckBox("Ignore invariants", refHorizontalLayout, this, SLOT(setIgnoreInvariants(bool)), false);

    toggleRobotHubModeBtn = new QPushButton("--");

    QObject::connect(toggleRobotHubModeBtn, SIGNAL(clicked()), this, SLOT(toggleRobotHubModeParam()));
    refHorizontalLayout->addWidget(toggleRobotHubModeBtn);
    setToggleRobotHubModeBtnLayout();

    vLayout->addLayout(refHorizontalLayout);

    auto gameStateBox = new QGroupBox();
    auto gameStateLayout = new QVBoxLayout();

    // get the strategy names from Switches
    select_play = new QComboBox();
    gameStateLayout->addWidget(select_play);
    select_play->setStyleSheet(
        QString::fromUtf8("QComboBox:disabled"
                          "{ color: gray }"));

    auto keeperHorizontalLayout = new QHBoxLayout();

    select_goalie = new QComboBox();
    keeperHorizontalLayout->addWidget(select_goalie);
    for (int i = 0; i < 16; i++) {
        select_goalie->addItem(QString::fromStdString(std::to_string(i)));
    }
    select_goalie->setMaximumWidth(100);
    select_goalie->setStyleSheet(
        QString::fromUtf8("QComboBox:disabled"
                          "{ color: gray }"));

    gameStateLayout->addLayout(keeperHorizontalLayout);

    select_ruleset = new QComboBox();
    gameStateLayout->addWidget(select_ruleset);
    for (RuleSet const &ruleSet : Constants::ruleSets()) {
        select_ruleset->addItem(QString::fromStdString(ruleSet.title));
    }
    select_ruleset->setStyleSheet(
        QString::fromUtf8("QComboBox:disabled"
                          "{ color: gray }"));

    auto settingsButtonsLayout = new QHBoxLayout();
    toggleColorBtn = new QPushButton("Color");
    QObject::connect(toggleColorBtn, SIGNAL(clicked()), this, SLOT(toggleOurColorParam()));
    settingsButtonsLayout->addWidget(toggleColorBtn);
    toggleColorBtn->setStyleSheet(
        QString::fromUtf8("QPushButton:disabled"
                          "{ color: gray }"));

    setToggleColorBtnLayout();

    toggleSideBtn = new QPushButton("Side");
    QObject::connect(toggleSideBtn, SIGNAL(clicked()), this, SLOT(toggleOurSideParam()));
    settingsButtonsLayout->addWidget(toggleSideBtn);
    toggleSideBtn->setStyleSheet(
        QString::fromUtf8("QPushButton:disabled"
                          "{ color: gray }"));

    setToggleSideBtnLayout();

    gameStateLayout->addLayout(settingsButtonsLayout);

    gameStateBox->setLayout(gameStateLayout);
    vLayout->addWidget(gameStateBox);

    // todo: figure out why this cast exists
    QObject::connect(select_play, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [=](int index) {
        // if number == -1 then the plays were refreshed, hence just keep the current play
        if (index == -1) {
            return;
        }
        // simply plays[index] because they're inserted in-order
        stp::PlayDecider::lockInterfacePlay(rtt::STPManager::plays[index].get());
        GameStateManager::updateInterfaceGameState(rtt::STPManager::plays[index].get()->getName());
    });

    QObject::connect(select_goalie, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated), [=](const QString &goalieId) {
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        interface::Output::setKeeperId(goalieId.toInt());
    });

    QObject::connect(select_ruleset, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated), [=](const QString &rulesetName) {
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        interface::Output::setRuleSetName(rulesetName.toStdString());
    });

    setUseReferee(Output::usesRefereeCommands());
    this->setLayout(vLayout);
}

void MainControlsWidget::setUseReferee(bool useRef) {
    Output::setUseRefereeCommands(useRef);

    select_play->setDisabled(useRef);
    select_ruleset->setDisabled(useRef);
    select_goalie->setDisabled(useRef);
    toggleSideBtn->setDisabled(useRef);
    toggleColorBtn->setDisabled(useRef);

    if (!useRef) {
        updatePlays();
    }
}

/// toggle the setting 'isYellow'
void MainControlsWidget::toggleOurColorParam() {
    SETTINGS.setYellow(!SETTINGS.isYellow());
    setToggleColorBtnLayout();
}

/// toggle the the setting 'isLeft'
void MainControlsWidget::toggleOurSideParam() {
    SETTINGS.setLeft(!SETTINGS.isLeft());
    setToggleSideBtnLayout();
}

/// toggle the the setting 'isSerialMode'
void MainControlsWidget::toggleRobotHubModeParam() {
    switch (SETTINGS.getRobotHubMode()) {
        case Settings::RobotHubMode::BASESTATION: {
            SETTINGS.setRobotHubMode(Settings::RobotHubMode::SIMULATOR);
            break;
        }
        case Settings::RobotHubMode::SIMULATOR: {
            SETTINGS.setRobotHubMode(Settings::RobotHubMode::BASESTATION);
            break;
        }
        default: {
            // In other cases, do not change anything
            break;
        }
    }

    // Wether the setting has changed or not, update the text anyways
    setToggleRobotHubModeBtnLayout();
}

/// send a halt signal to stop all trees from executing
void MainControlsWidget::sendPauseSignal() { Output::sendHaltCommand(); }

void MainControlsWidget::setToggleColorBtnLayout() const {
    if (SETTINGS.isYellow()) {
        toggleColorBtn->setStyleSheet("background-color: orange;");  // orange is more readable
        toggleColorBtn->setText("Playing as Yellow");
    } else {
        toggleColorBtn->setStyleSheet("background-color: blue;");
        toggleColorBtn->setText("Playing as Blue");
    }
}

void MainControlsWidget::setToggleSideBtnLayout() const {
    if (SETTINGS.isLeft()) {
        toggleSideBtn->setText("◀ Playing as left");
    } else {
        toggleSideBtn->setText("Playing as right ▶");
    }
}

void MainControlsWidget::setToggleRobotHubModeBtnLayout() const {
    std::string modeText = Settings::robotHubModeToString(SETTINGS.getRobotHubMode());

    QString buttonText = QString::fromStdString(modeText);
    toggleRobotHubModeBtn->setText(buttonText);
}

void MainControlsWidget::updateContents() {
    auto ruleSetText = QString::fromStdString(GameStateManager::getCurrentGameState().ruleSetName);
    if (ruleSetText != select_ruleset->currentText()) {
        select_ruleset->setCurrentText(ruleSetText);
    }

    auto goalieIdText = QString::number(GameStateManager::getCurrentGameState().keeperId);
    if (goalieIdText != select_goalie->currentText()) {
        select_goalie->setCurrentText(goalieIdText);
    }

    // visual indication if we have a keeper or not
    if (GameStateManager::getCurrentGameState().keeperId == -1) {
        select_goalie->setStyleSheet("background-color: #00b200;");
    } else {
        select_goalie->setStyleSheet("background-color: #cc0000;");
    }

    this->setToggleSideBtnLayout();
    this->setToggleColorBtnLayout();
    this->setToggleRobotHubModeBtnLayout();
}

void MainControlsWidget::updatePlays() {
    select_play->clear();
    for (auto const &each : manager->plays) {
        select_play->addItem(each->getName());
    }
}

void MainControlsWidget::setIgnoreInvariants(bool ignore) { MainControlsWidget::ignoreInvariants = ignore; }

}  // namespace rtt::ai::interface
