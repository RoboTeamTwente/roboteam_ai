/*
 *
 * This file contains the contents of the 'PID' tab in the interface
 */

#include "PidsWidget.h"
#include "PidBox.h"

namespace rtt {
namespace ai {
namespace interface {

PidsWidget::PidsWidget(QWidget * parent) {


    // initialize values for interface to display
    Output::setNumTreePid(Constants::standardNumTreePID());
    Output::setForcePid(Constants::standardForcePID());
    Output::setBasicPid(Constants::standardBasicPID());
    Output::setShotControllerPID(Constants::standardShotControllerPID());

     auto pidVLayout = new QVBoxLayout();

    // create the widgets for the pids
    auto numTreePidBox = new PidBox("NumTree");
    auto forcePidBox = new PidBox("Force");
    auto basicPidBox = new PidBox("Basic");
    auto shotControlPidbox = new PidBox("Shotcontroller ball approach");

    // initialize them with the default values
    numTreePidBox->setPid(Output::getNumTreePid());
    forcePidBox->setPid(Output::getForcePid());
    basicPidBox->setPid(Output::getBasicPid());
    shotControlPidbox->setPid(Output::getShotControllerPID());

    QObject::connect(numTreePidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setNumTreePid(pid); });

    QObject::connect(forcePidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setForcePid(pid); });

    QObject::connect(basicPidBox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
                     [=](const pidVals &pid) { Output::setBasicPid(pid); });

    QObject::connect(shotControlPidbox, static_cast<void (PidBox::*)(pidVals)>(&PidBox::pidChanged),
            [=](const pidVals &pid) { Output::setShotControllerPID(pid); });

    // add the pid widgets to the layout
    pidVLayout->addWidget(numTreePidBox);
    pidVLayout->addWidget(forcePidBox);
    pidVLayout->addWidget(basicPidBox);
    pidVLayout->addWidget(shotControlPidbox);

    auto pidSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    pidVLayout->addSpacerItem(pidSpacer);
    this->setLayout(pidVLayout);
}

} // interface
} // ai
} // rtt