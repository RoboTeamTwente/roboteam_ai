//
// Created by mrlukasbos on 7-5-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "MainControlsWidget.h"
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

MainControlsWidget::MainControlsWidget(QWidget * parent) {
    
    vLayout = new QVBoxLayout();

    // functions to select strategies
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

    this->setLayout(vLayout);
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

} // interface
} // ai
} // rtt

// QT performance improvement
#include "moc_MainControlsWidget.cpp"