
#include "interface/widgets/SettingsWidget.h"

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QLineEdit>
#include "interface/widgets/mainWindow.h"

namespace rtt::ai::interface {

SettingsWidget::SettingsWidget(QWidget * parent, ::rtt::world::settings::Settings& settings)
    : settings{ settings } {
    vLayout = new QVBoxLayout();
    this->setLayout(vLayout);

    // grsim ip + port settings
    QGroupBox * grsimSettingsGroup = new QGroupBox("grsim transmission ip + port");
    auto grsimSettingsWidgetLayout = new QHBoxLayout();
    grsimIpText = new QLineEdit();
    grsimIpText->setText(QString::fromStdString(settings.getRobothubSendIp()));
    QObject::connect(grsimIpText, SIGNAL(textChanged(QString)), this, SLOT(changeGrSimIp(QString)));
    grsimSettingsWidgetLayout->addWidget(grsimIpText);
    grsimPort = new QSpinBox();
    grsimPort->setRange(0, 999999);
    grsimPort->setValue(settings.getRobothubSendPort());
    grsimSettingsWidgetLayout->addWidget(grsimPort);
    grsimSettingsGroup->setLayout(grsimSettingsWidgetLayout);
    QObject::connect(grsimPort, SIGNAL(textChanged(QString)), this, SLOT(changeGrSimPort(int)));
    vLayout->addWidget(grsimSettingsGroup);

    auto spacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    vLayout->addSpacerItem(spacer);
}

void SettingsWidget::changeTeamColor(bool isYellow) {
    settings.setYellow(isYellow);
}

void SettingsWidget::changeTeamSide(bool isLeft) {
    settings.setLeft(isLeft);
}

void SettingsWidget::changeMode(bool serial) {
    settings.setSerialMode(serial);
}

void SettingsWidget::changeGrSimIp(QString ip) {
    std::cout << "setting grsimip" << std::endl;
    settings.setRobothubSendIp(ip.toStdString());
}

void SettingsWidget::changeGrSimPort(int port) {
    std::cout << "setting grsimport" << std::endl;
    settings.setRobothubSendPort(port);
}

} // rtt


// QT performance improvement
#include "include/roboteam_ai/interface/widgets/moc_SettingsWidget.cpp"
