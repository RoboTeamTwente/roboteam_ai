//
// Created by mrlukasbos on 7-5-19.
//

#include <include/roboteam_ai/utilities/RobotDealer.h>
#include <include/roboteam_ai/Switches.h>
#include <include/roboteam_ai/treeinterp/BTFactory.h>
#include <include/roboteam_ai/interface/api/Output.h>
#include "include/roboteam_ai/interface/widgets/MainControlsWidget.h"
#include "include/roboteam_ai/interface/widgets/mainWindow.h"
#include "include/roboteam_ai/treeinterp/BTFactory.h"
#include <include/roboteam_ai/utilities/GameStateManager.hpp>
#include "include/roboteam_ai/utilities/GameState.h"

namespace rtt {
namespace ai {
namespace interface {

MainControlsWidget::MainControlsWidget(QWidget * parent) {

    Output::setUseRefereeCommands(Constants::STD_USE_REFEREE());

    vLayout = new QVBoxLayout();

    // functions to select strategies
    MainWindow::configureCheckBox("Use referee", vLayout, this, SLOT(setUseReferee(bool)), Constants::STD_USE_REFEREE());

    // get the strategy names from Switches
    select_strategy = new QComboBox();
    vLayout->addWidget(select_strategy);
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }
    select_strategy->setStyleSheet(QString::fromUtf8("QComboBox:disabled" "{ color: gray }"));

    // get the keeper tree names from Switches
    select_keeper_strategy = new QComboBox();
    vLayout->addWidget(select_keeper_strategy);
    for (std::string const &keeperTacticName : Switches::keeperJsonFiles) {
        select_keeper_strategy->addItem(QString::fromStdString(keeperTacticName));
    }
    select_keeper_strategy->setStyleSheet(QString::fromUtf8("QComboBox:disabled" "{ color: gray }"));

    select_goalie = new QComboBox();
    vLayout->addWidget(select_goalie);
    for (int i = 0; i < 16; i++) {
        select_goalie->addItem(QString::fromStdString(std::to_string(i)));
    }
    select_goalie->setStyleSheet(QString::fromUtf8("QComboBox:disabled" "{ color: gray }"));

    select_ruleset = new QComboBox();
    vLayout->addWidget(select_ruleset);
    for (RuleSet const &ruleSet : Constants::ruleSets()) {
        select_ruleset->addItem(QString::fromStdString(ruleSet.title));
    }
    select_ruleset->setStyleSheet(QString::fromUtf8("QComboBox:disabled" "{ color: gray }"));

    auto refreshHButtonsLayout = new QHBoxLayout();

    refreshBtn = new QPushButton("Soft refresh");
    QObject::connect(refreshBtn, SIGNAL(clicked()), this, SLOT(refreshSignal()));
    refreshHButtonsLayout->addWidget(refreshBtn);
    refreshBtn->setStyleSheet("background-color: #0000cc;");

    refreshJsonBtn = new QPushButton("Hard refresh");
    QObject::connect(refreshJsonBtn, SIGNAL(clicked()), this, SLOT(refreshJSONSignal()));
    refreshHButtonsLayout->addWidget(refreshJsonBtn);
    refreshJsonBtn->setStyleSheet("background-color: #0000cc;");
    vLayout->addLayout(refreshHButtonsLayout);


    auto hButtonsLayout = new QHBoxLayout();

    haltBtn = new QPushButton("Halt");
    QObject::connect(haltBtn, SIGNAL(clicked()), this, SLOT(sendHaltSignal()));
    hButtonsLayout->addWidget(haltBtn);
    haltBtn->setStyleSheet("background-color: #cc0000;");

    pauseBtn = new QPushButton("Pause");
    QObject::connect(pauseBtn, SIGNAL(clicked()), this, SLOT(sendPauseSignal()));
    hButtonsLayout->addWidget(pauseBtn);
    pauseBtn->setStyleSheet("background-color: #cc0000;");

    spaceClick = new QShortcut(QKeySequence(Qt::Key_Space), this, SLOT(sendPauseSignal()));
    spaceClick->setAutoRepeat(false);

    toggleColorBtn = new QPushButton("Color");
    QObject::connect(toggleColorBtn, SIGNAL(clicked()), this, SLOT(toggleOurColorParam()));
    hButtonsLayout->addWidget(toggleColorBtn);
    setToggleColorBtnLayout(); // set the btn color and text to the current our_color

    toggleSideBtn = new QPushButton("Side");
    QObject::connect(toggleSideBtn, SIGNAL(clicked()), this, SLOT(toggleOurSideParam()));
    hButtonsLayout->addWidget(toggleSideBtn);
    setToggleColorBtnLayout(); // set the btn color and text to the current our_side

    vLayout->addLayout(hButtonsLayout);

    MainWindow::configureCheckBox("TimeOut to top", vLayout, this, SLOT(setTimeOutTop(bool)), Constants::STD_TIMEOUT_TO_TOP());

    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &strategyName) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         interface::Output::setStrategyTree(strategyName.toStdString());
                         emit treeHasChanged();
                     });

    QObject::connect(select_keeper_strategy,
                     static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &keeperStrategyName) {
// http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         interface::Output::setKeeperTree(keeperStrategyName.toStdString());
                         emit treeHasChanged();
                     });

    QObject::connect(select_goalie, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &goalieId) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         interface::Output::setKeeperId(goalieId.toInt());
                         emit treeHasChanged();
                     });

    QObject::connect(select_ruleset, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &rulesetName) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         //robotDealer::RobotDealer::setKeeperID(goalieId.toInt());.
                         interface::Output::setRuleSetName(rulesetName.toStdString());
                         emit treeHasChanged();
                     });

    this->setLayout(vLayout);
}


void MainControlsWidget::setTimeOutTop(bool top) {
    Output::setTimeOutTop(top);
}

void MainControlsWidget::setUseReferee(bool useRef) {
    Output::setUseRefereeCommands(useRef);

    select_strategy->setDisabled(useRef);
    select_keeper_strategy->setDisabled(useRef);
    select_ruleset->setDisabled(useRef);
    select_goalie->setDisabled(useRef);
}


/// toggle the ROS param 'our_color'
void MainControlsWidget::toggleOurColorParam() {
//    ros::NodeHandle nh;
//    std::string ourColorParam, newParam;
//    nh.getParam("our_color", ourColorParam);
//    newParam = ourColorParam == "yellow" ? "blue" : "yellow";
//    nh.setParam("our_color", newParam);
//
//    setToggleColorBtnLayout();
}

/// toggle the ROS param 'our_color'
void MainControlsWidget::toggleOurSideParam() {
//    ros::NodeHandle nh;
//    std::string ourColorParam, newParam;
//    nh.getParam("our_side", ourColorParam);
//    newParam = ourColorParam == "left" ? "right" : "left";
//    nh.setParam("our_side", newParam);
//
//    setToggleSideBtnLayout();
}

/// send a halt signal to stop all trees from executing
void MainControlsWidget::sendPauseSignal() {
    Output::sendHaltCommand();
}

void MainControlsWidget::updatePause() {
    rtt::ai::Pause pause;
    if (pause.getPause()) {
        pauseBtn->setText("Resume");
        pauseBtn->setStyleSheet("background-color: #00b200;");
    }
    else {
        pauseBtn->setText("Pause");
        pauseBtn->setStyleSheet("background-color: #cc0000;");
    }
}

void MainControlsWidget::setToggleColorBtnLayout() const {
//    ros::NodeHandle nh;
//    std::string ourColorParam;
//    nh.getParam("our_color", ourColorParam);
//    if (ourColorParam == "yellow") {
//        toggleColorBtn->setStyleSheet("background-color: orange;"); // orange is more readable
//    } else {
//        toggleColorBtn->setStyleSheet("background-color: blue;");
//    }
//    toggleColorBtn->setText(QString::fromStdString(ourColorParam));
}

void MainControlsWidget::setToggleSideBtnLayout() const {
//    ros::NodeHandle nh;
//    std::string ourSideParam;
//    nh.getParam("our_side", ourSideParam);
//    if (ourSideParam == "left") {
//        toggleSideBtn->setStyleSheet("background-color: #cc0000;");
//        toggleSideBtn->setText("◀ Left");
//
//    } else {
//        toggleSideBtn->setText("right ▶");
//        toggleSideBtn->setStyleSheet("background-color: #cc0000;");
//    }
}


void MainControlsWidget::refreshSignal() {
    robotDealer::RobotDealer::refresh();
    emit treeHasChanged();
}

void MainControlsWidget::refreshJSONSignal() {
    BTFactory::makeTrees();
    robotDealer::RobotDealer::refresh();
    emit treeHasChanged();
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

void MainControlsWidget::sendHaltSignal() {
    if (isHalted) {
        Output::setInterfaceGameState(prevGameState);
        haltBtn->setText("Halt");
        haltBtn->setStyleSheet("background-color: #cc0000;");
    } else {
        prevGameState = GameStateManager::getCurrentGameState();
        GameStateManager::forceNewGameState(RefCommand::HALT);
        haltBtn->setText("unHalt");
        haltBtn->setStyleSheet("background-color: #00b200;");
    }
    isHalted = !isHalted;
}

} // interface
} // ai
} // rtt

// QT performance improvement
#include "include/roboteam_ai/interface/widgets/moc_MainControlsWidget.cpp"