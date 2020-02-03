//
// Created by mrlukasbos on 7-5-19.
//

#include "interface/widgets/MainControlsWidget.h"

#include <Switches.h>
#include <include/roboteam_ai/interface/api/Input.h>
#include <include/roboteam_ai/utilities/Settings.h>
#include <interface/api/Output.h>
#include <treeinterp/BTFactory.h>
#include <utilities/RobotDealer.h>

#include <utilities/GameStateManager.hpp>

#include "interface/widgets/mainWindow.h"
#include "treeinterp/BTFactory.h"
#include "utilities/GameState.h"

namespace rtt {
namespace ai {
namespace interface {

MainControlsWidget::MainControlsWidget(QWidget *parent) {
    Output::setUseRefereeCommands(Constants::STD_USE_REFEREE());

    vLayout = new QVBoxLayout();

    pauseBtn = new QPushButton("Stop");
    QObject::connect(pauseBtn, SIGNAL(clicked()), this, SLOT(sendPauseSignal()));
    vLayout->addWidget(pauseBtn);
    pauseBtn->setStyleSheet("background-color: #cc0000;");
    pauseBtn->setMinimumHeight(40);

    spaceClick = new QShortcut(QKeySequence(Qt::Key_Space), this, SLOT(sendPauseSignal()));
    spaceClick->setAutoRepeat(false);

    auto refHorizontalLayout = new QHBoxLayout();

    // functions to select strategies
    MainWindow::configureCheckBox("Use referee", refHorizontalLayout, this, SLOT(setUseReferee(bool)), Constants::STD_USE_REFEREE());

    toggleSerialBtn = new QPushButton("Serial");
    QObject::connect(toggleSerialBtn, SIGNAL(clicked()), this, SLOT(toggleSerialParam()));
    refHorizontalLayout->addWidget(toggleSerialBtn);
    setToggleSerialBtnLayout();

    vLayout->addLayout(refHorizontalLayout);

    auto gameStateBox = new QGroupBox();
    auto gameStateLayout = new QVBoxLayout();

    // get the strategy names from Switches
    select_strategy = new QComboBox();
    gameStateLayout->addWidget(select_strategy);
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }
    select_strategy->setStyleSheet(
        QString::fromUtf8("QComboBox:disabled"
                          "{ color: gray }"));

    auto keeperHorizontalLayout = new QHBoxLayout();

    // get the keeper tree names from Switches
    select_keeper_strategy = new QComboBox();
    keeperHorizontalLayout->addWidget(select_keeper_strategy);
    for (std::string const &keeperTacticName : Switches::keeperJsonFiles) {
        select_keeper_strategy->addItem(QString::fromStdString(keeperTacticName));
    }
    select_keeper_strategy->setStyleSheet(
        QString::fromUtf8("QComboBox:disabled"
                          "{ color: gray }"));

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

    // MainWindow::configureCheckBox("TimeOut to top", controlsLayout, this, SLOT(setTimeOutTop(bool)), Constants::STD_TIMEOUT_TO_TOP());

    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated), [=](const QString &strategyName) {
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        interface::Output::setStrategyTree(strategyName.toStdString());
        emit treeHasChanged();
    });

    QObject::connect(select_keeper_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated), [=](const QString &keeperStrategyName) {
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        interface::Output::setKeeperTree(keeperStrategyName.toStdString());
        emit treeHasChanged();
    });

    QObject::connect(select_goalie, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated), [=](const QString &goalieId) {
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        interface::Output::setKeeperId(goalieId.toInt());
        emit treeHasChanged();
    });

    QObject::connect(select_ruleset, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated), [=](const QString &rulesetName) {
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        // robotDealer::RobotDealer::setKeeperID(goalieId.toInt());.
        interface::Output::setRuleSetName(rulesetName.toStdString());
        emit treeHasChanged();
    });

    setUseReferee(Output::usesRefereeCommands());
    this->setLayout(vLayout);
}

void MainControlsWidget::setTimeOutTop(bool top) { Output::setTimeOutTop(top); }

void MainControlsWidget::setUseReferee(bool useRef) {
    Output::setUseRefereeCommands(useRef);

    select_strategy->setDisabled(useRef);
    select_keeper_strategy->setDisabled(useRef);
    select_ruleset->setDisabled(useRef);
    select_goalie->setDisabled(useRef);
    toggleSideBtn->setDisabled(useRef);
    toggleColorBtn->setDisabled(useRef);
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
void MainControlsWidget::toggleSerialParam() {
    SETTINGS.setSerialMode(!SETTINGS.isSerialMode());
    setToggleSerialBtnLayout();
}

/// send a halt signal to stop all trees from executing
void MainControlsWidget::sendPauseSignal() { Output::sendHaltCommand(); }

void MainControlsWidget::updatePause() {
    rtt::ai::Pause pause;
    if (pause.getPause()) {
        pauseBtn->setText("Resume");
        pauseBtn->setStyleSheet("background-color: #00b200;");
    } else {
        pauseBtn->setText("Stop");
        pauseBtn->setStyleSheet("background-color: #cc0000;");
    }
}

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

void MainControlsWidget::setToggleSerialBtnLayout() const {
    if (SETTINGS.isSerialMode()) {
        toggleSerialBtn->setText("BaseStation");
    } else {
        toggleSerialBtn->setText("GrSim");
    }
}

void MainControlsWidget::updateContents() {
    auto strategyText = QString::fromStdString(BTFactory::getCurrentTree());
    if (strategyText != select_strategy->currentText()) {
        select_strategy->setCurrentText(strategyText);
    }

    auto keeperStrategyText = QString::fromStdString(BTFactory::getKeeperTreeName());
    if (keeperStrategyText != select_keeper_strategy->currentText()) {
        select_keeper_strategy->setCurrentText(keeperStrategyText);
    }

    auto ruleSetText = QString::fromStdString(GameStateManager::getCurrentGameState().ruleSetName);
    if (ruleSetText != select_ruleset->currentText()) {
        select_ruleset->setCurrentText(ruleSetText);
    }

    auto goalieIdText = QString::number(GameStateManager::getCurrentGameState().keeperId);
    if (goalieIdText != select_goalie->currentText()) {
        select_goalie->setCurrentText(goalieIdText);
    }

    // visual indication if we have a keeper or not
    if (robotDealer::RobotDealer::keeperExistsInWorld()) {
        select_goalie->setStyleSheet("background-color: #00b200;");
    } else {
        select_goalie->setStyleSheet("background-color: #cc0000;");
    }
}

}  // namespace interface
}  // namespace ai
}  // namespace rtt

// QT performance improvement
#include "include/roboteam_ai/interface/widgets/moc_MainControlsWidget.cpp"