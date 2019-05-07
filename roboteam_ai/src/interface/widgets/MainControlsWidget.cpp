//
// Created by mrlukasbos on 7-5-19.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/Switches.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include <roboteam_ai/src/interface/api/Output.h>
#include "MainControlsWidget.h"
#include "mainWindow.h"

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

    // get the keeper tree names from Switches
    select_keeper_strategy = new QComboBox();
    vLayout->addWidget(select_keeper_strategy);
    for (std::string const &keeperTacticName : Switches::keeperJsonFiles) {
        select_keeper_strategy->addItem(QString::fromStdString(keeperTacticName));
    }

    select_goalie = new QComboBox();
    vLayout->addWidget(select_goalie);
    for (int i = 0; i < 16; i++) {
        select_goalie->addItem(QString::fromStdString(to_string(i)));
    }
    auto hButtonsLayout = new QHBoxLayout();
    haltBtn = new QPushButton("Pause");
    QObject::connect(haltBtn, SIGNAL(clicked()), this, SLOT(sendHaltSignal()));
    hButtonsLayout->addWidget(haltBtn);
    haltBtn->setStyleSheet("background-color: #cc0000;");

    refreshBtn = new QPushButton("Refresh");
    QObject::connect(refreshBtn, SIGNAL(clicked()), this, SLOT(refreshSignal()));
    hButtonsLayout->addWidget(refreshBtn);
    refreshBtn->setStyleSheet("background-color: #0000cc;");

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
    MainWindow::configureCheckBox("Use keeper (does not work when referee used)", vLayout, this, SLOT(setUsesKeeper(bool)),
                      robotDealer::RobotDealer::usesSeparateKeeper());

    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &strategyName) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         BTFactory::setCurrentTree(strategyName.toStdString());
                         robotDealer::RobotDealer::refresh();
                         emit treeHasChanged();
                     });

    QObject::connect(select_keeper_strategy,
                     static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &keeperStrategyName) {
// http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         BTFactory::setKeeperTree(keeperStrategyName.toStdString());
                         robotDealer::RobotDealer::refresh();
                         emit treeHasChanged();
                     });

    QObject::connect(select_goalie, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
                     [=](const QString &goalieId) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         robotDealer::RobotDealer::setKeeperID(goalieId.toInt());
                         emit treeHasChanged();
                     });

    this->setLayout(vLayout);
}


void MainControlsWidget::setTimeOutTop(bool top) {
    Output::setTimeOutTop(top);
}

void MainControlsWidget::setUsesKeeper(bool usekeeper) {
    robotDealer::RobotDealer::setUseSeparateKeeper(usekeeper);
    robotDealer::RobotDealer::refresh();
}

QString MainControlsWidget::getSelectStrategyText() const {
    return select_strategy->currentText();
}

void MainControlsWidget::setSelectStrategyText(QString text) {
    select_strategy->setCurrentText(text);
}

void MainControlsWidget::setUseReferee(bool useRef) {
    Output::setUseRefereeCommands(useRef);
}


/// toggle the ROS param 'our_color'
void MainControlsWidget::toggleOurColorParam() {
    ros::NodeHandle nh;
    std::string ourColorParam, newParam;
    nh.getParam("our_color", ourColorParam);
    newParam = ourColorParam == "yellow" ? "blue" : "yellow";
    nh.setParam("our_color", newParam);

    setToggleColorBtnLayout();
}

/// toggle the ROS param 'our_color'
void MainControlsWidget::toggleOurSideParam() {
    ros::NodeHandle nh;
    std::string ourColorParam, newParam;
    nh.getParam("our_side", ourColorParam);
    newParam = ourColorParam == "left" ? "right" : "left";
    nh.setParam("our_side", newParam);

    setToggleSideBtnLayout();
}

/// send a halt signal to stop all trees from executing
void MainControlsWidget::sendHaltSignal() {
    Output::sendHaltCommand();
}

void MainControlsWidget::updatePause() {
    rtt::ai::Pause pause;
    if (pause.getPause()) {
        haltBtn->setText("Resume");
        haltBtn->setStyleSheet("background-color: #00b200;");
    }
    else {
        haltBtn->setText("Pause");
        haltBtn->setStyleSheet("background-color: #cc0000;");
    }
}

void MainControlsWidget::setToggleColorBtnLayout() const {
    ros::NodeHandle nh;
    std::string ourColorParam;
    nh.getParam("our_color", ourColorParam);
    if (ourColorParam == "yellow") {
        toggleColorBtn->setStyleSheet("background-color: orange;"); // orange is more readable
    } else {
        toggleColorBtn->setStyleSheet("background-color: blue;");
    }
    toggleColorBtn->setText(QString::fromStdString(ourColorParam));
}

void MainControlsWidget::setToggleSideBtnLayout() const {
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);
    if (ourSideParam == "left") {
        toggleSideBtn->setStyleSheet("background-color: #cc0000;");
        toggleSideBtn->setText("◀ Left");

    } else {
        toggleSideBtn->setText("right ▶");
        toggleSideBtn->setStyleSheet("background-color: #cc0000;");
    }
}


void MainControlsWidget::refreshSignal() {
    robotDealer::RobotDealer::refresh();
    emit treeHasChanged();
}

} // interface
} // ai
} // rtt