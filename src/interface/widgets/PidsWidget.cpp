/*
 *
 * This file contains the contents of the 'PID' tab in the interface
 */

#include "interface/widgets/PidsWidget.h"

#include "interface/widgets/PidBox.h"

namespace rtt::ai::interface {

PidsWidget::PidsWidget(QWidget *parent) {
    // initialize values for interface to display
    Output::setNumTreePid(Constants::standardNumTreePID());
    Output::setReceivePid(Constants::standardReceivePID());
    Output::setInterceptPid(Constants::standardInterceptPID());
    Output::setKeeperPid(Constants::standardKeeperPID());
    Output::setKeeperInterceptPid(Constants::standardKeeperInterceptPID());

    auto pidVLayout = new QVBoxLayout();

    // create the widgets for the pids
    auto numTreePidBox = new PidBox("NumTree");
    auto receivePidBox = new PidBox("Receive");
    auto interceptPidBox = new PidBox("Intercept");
    auto keeperPidBox = new PidBox("Keeper");
    auto keeperInterceptPidBox = new PidBox("Keeper Intercept");

    // initialize them with the default values
    numTreePidBox->setPid(Output::getNumTreePid());
    receivePidBox->setPid(Output::getReceivePid());
    interceptPidBox->setPid(Output::getInterceptPid());
    keeperPidBox->setPid(Output::getKeeperPid());
    keeperInterceptPidBox->setPid(Output::getKeeperInterceptPid());

    QObject::connect(numTreePidBox, &PidBox::pidChanged, [=](const pidVals &pid) { Output::setNumTreePid(pid); });

    QObject::connect(receivePidBox, &PidBox::pidChanged, [=](const pidVals &pid) { Output::setReceivePid(pid); });

    QObject::connect(interceptPidBox, &PidBox::pidChanged, [=](const pidVals &pid) { Output::setInterceptPid(pid); });

    QObject::connect(keeperPidBox, &PidBox::pidChanged, [=](const pidVals &pid) { Output::setKeeperPid(pid); });

    QObject::connect(keeperInterceptPidBox, &PidBox::pidChanged, [=](const pidVals &pid) { Output::setKeeperInterceptPid(pid); });

    // add the pid widgets to the layout
    pidVLayout->addWidget(numTreePidBox);
    pidVLayout->addWidget(receivePidBox);
    pidVLayout->addWidget(interceptPidBox);
    pidVLayout->addWidget(keeperPidBox);
    pidVLayout->addWidget(keeperInterceptPidBox);

    auto pidSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    pidVLayout->addSpacerItem(pidSpacer);
    this->setLayout(pidVLayout);
}

}  // namespace rtt::ai::interface
