#include <roboteam_utils/Print.h>
#include "interface/widgets/SettingsWidget.h"

#include "interface/widgets/mainWindow.h"
#include "utilities/Settings.h"

namespace rtt::ai::interface {

SettingsWidget::SettingsWidget(QWidget *parent) {
    vLayout = new QVBoxLayout();
    this->setLayout(vLayout);

    // grsim ip + port settings
    QGroupBox *grsimSettingsGroup = new QGroupBox("grsim transmission ip + port");
    auto grsimSettingsWidgetLayout = new QHBoxLayout();
    grsimIpText = new QLineEdit();
    grsimIpText->setText(QString::fromStdString(SETTINGS.getRobothubSendIp()));
    QObject::connect(grsimIpText, SIGNAL(textChanged(QString)), this, SLOT(changeGrSimIp(QString)));
    grsimSettingsWidgetLayout->addWidget(grsimIpText);
    grsimPort = new QSpinBox();
    grsimPort->setRange(0, 999999);
    grsimPort->setValue(SETTINGS.getRobothubSendPort());
    grsimSettingsWidgetLayout->addWidget(grsimPort);
    grsimSettingsGroup->setLayout(grsimSettingsWidgetLayout);
    vLayout->addWidget(grsimSettingsGroup);

    auto spacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    vLayout->addSpacerItem(spacer);
}

void SettingsWidget::changeTeamColor(bool isYellow) { SETTINGS.setYellow(isYellow); }

void SettingsWidget::changeTeamSide(bool isLeft) { SETTINGS.setLeft(isLeft); }

void SettingsWidget::changeRobotHubMode(Settings::RobotHubMode mode) { SETTINGS.setRobotHubMode(mode); }

void SettingsWidget::changeGrSimIp(QString ip) {
    RTT_INFO("Setting GrSim IP address")
    SETTINGS.setRobothubSendIp(ip.toStdString());
}

void SettingsWidget::changeGrSimPort(int port) {
    RTT_INFO("Setting GrSim port")
    SETTINGS.setRobothubSendPort(port);
}

}  // namespace rtt::ai::interface
