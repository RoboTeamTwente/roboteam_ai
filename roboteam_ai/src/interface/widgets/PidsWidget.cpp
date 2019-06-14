/*
 *
 * This file contains the contents of the 'PID' tab in the interface
 */

#include <roboteam_ai/src/interface/api/Output.h>
#include "PidsWidget.h"
#include "PidBox.h"
#include "QLayout"

namespace rtt {
namespace ai {
namespace interface {

PidsWidget::PidsWidget(QWidget * parent) {
    // initialize values for interface to display
    Output::setNumTreePid(Constants::standardNumTreePID());
    Output::setBasicPid(Constants::standardBasicPID());
    Output::setKeeperPid(Constants::standardKeeperPID());
    Output::setKeeperInterceptPid(Constants::standardKeeperInterceptPID());
    Output::setBallHandlePid(Constants::standardBallHandlePID());

     auto pidVLayout = new QVBoxLayout();

    // create the widgets for the pids
    auto numTreePidBox = new PidBox("NumTree");
    auto basicPidBox = new PidBox("Basic");
    auto keeperPidBox = new PidBox("Keeper");
    auto keeperInterceptPidBox = new PidBox("Keeper Intercept");
    auto ballHandlePidBox = new PidBox("Ball handle");

    // initialize them with the default values
    numTreePidBox->setPid(Output::getNumTreePid());
    basicPidBox->setPid(Output::getBasicPid());
    keeperPidBox->setPid(Output::getKeeperPid());
    keeperInterceptPidBox->setPid(Output::getKeeperInterceptPid());
    ballHandlePidBox->setPid(Output::getBallHandlePid());

    QObject::connect(numTreePidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setNumTreePid(pid); });

    QObject::connect(basicPidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setBasicPid(pid); });

    QObject::connect(keeperPidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setKeeperPid(pid); });

    QObject::connect(keeperInterceptPidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setKeeperInterceptPid(pid); });

    QObject::connect(ballHandlePidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
            [=](const pidVals &pid) { Output::setBallHandlePid(pid); });

    // add the pid widgets to the layout
    pidVLayout->addWidget(numTreePidBox);
    pidVLayout->addWidget(basicPidBox);
    pidVLayout->addWidget(keeperPidBox);
    pidVLayout->addWidget(keeperInterceptPidBox);
    pidVLayout->addWidget(ballHandlePidBox);

    auto pidSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    pidVLayout->addSpacerItem(pidSpacer);
    this->setLayout(pidVLayout);
}

} // interface
} // ai
} // rtt

// QT performance improvement
#include "moc_PidsWidget.cpp"