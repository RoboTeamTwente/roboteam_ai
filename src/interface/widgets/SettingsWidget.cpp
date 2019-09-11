
#include "interface/widgets/SettingsWidget.h"

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QLineEdit>
#include <Settings/Settings.h>
#include "interface/widgets/mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

SettingsWidget::SettingsWidget(QWidget * parent) {
    vLayout = new QVBoxLayout();
    this->setLayout(vLayout);

    // id setting
    QGroupBox * idSettingsGroup = new QGroupBox("AI ID");
    auto idSettingsWidgetLayout = new QHBoxLayout();
    auto idPort = new QSpinBox();
    idPort->setRange(0, 999999);
    idPort->setValue(10006);
    idPort->setDisabled(true);
    idSettingsWidgetLayout->addWidget(idPort);
    idSettingsGroup->setLayout(idSettingsWidgetLayout);
    vLayout->addWidget(idSettingsGroup);

    MainWindow::configureCheckBox("We are yellow", vLayout, this, SLOT(changeTeamColor(bool)), SETTINGS.isYellow());
    MainWindow::configureCheckBox("We are left", vLayout, this, SLOT(changeTeamSide(bool)), SETTINGS.isLeft());
    MainWindow::configureCheckBox("Serial output", vLayout, this, SLOT(changeMode(bool)), SETTINGS.isSerialMode());

    // vision ip + port settings
    QGroupBox * visionSettingsGroup = new QGroupBox("Vision ip + port");
    auto visionSettingsWidgetLayout = new QHBoxLayout();
    auto visionIpText = new QLineEdit();
    visionIpText->setText(QString::fromStdString(SETTINGS.getVisionIp()));
    QObject::connect(visionIpText, SIGNAL(textChanged(QString)), this, SLOT(changeVisionIp(QString)));
    visionSettingsWidgetLayout->addWidget(visionIpText);
    auto visionPort = new QSpinBox();
    visionPort->setRange(0, 999999);
    visionPort->setValue(SETTINGS.getVisionPort());
    visionSettingsWidgetLayout->addWidget(visionPort);
    visionSettingsGroup->setLayout(visionSettingsWidgetLayout);
    QObject::connect(visionPort, SIGNAL(textChanged(QString)), this, SLOT(changeVisionPort(int)));
    vLayout->addWidget(visionSettingsGroup);

    // referee ip + port settings
    QGroupBox * refereeSettingsGroup = new QGroupBox("Referee ip + port");
    auto refereeSettingsWidgetLayout = new QHBoxLayout();
    auto refereeIpText = new QLineEdit();
    refereeIpText->setText(QString::fromStdString(SETTINGS.getRefereeIp()));
    QObject::connect(refereeIpText, SIGNAL(textChanged(QString)), this, SLOT(changeRefereeIp(QString)));
    refereeSettingsWidgetLayout->addWidget(refereeIpText);
    auto refereePort = new QSpinBox();
    refereePort->setRange(0, 999999);
    refereePort->setValue(SETTINGS.getRefereePort());
    refereeSettingsWidgetLayout->addWidget(refereePort);
    refereeSettingsGroup->setLayout(refereeSettingsWidgetLayout);
    QObject::connect(refereePort, SIGNAL(textChanged(QString)), this, SLOT(changeRefereePort(int)));
    vLayout->addWidget(refereeSettingsGroup);

    // grsim ip + port settings
    QGroupBox * grsimSettingsGroup = new QGroupBox("grsim transmission ip + port");
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
    QObject::connect(grsimPort, SIGNAL(textChanged(QString)), this, SLOT(changeGrSimPort(int)));
    vLayout->addWidget(grsimSettingsGroup);

    auto spacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    vLayout->addSpacerItem(spacer);
}

void SettingsWidget::changeTeamColor(bool isYellow) {
    SETTINGS.setYellow(isYellow);
}

void SettingsWidget::changeTeamSide(bool isLeft) {
    SETTINGS.setLeft(isLeft);
}

void SettingsWidget::changeMode(bool serial) {
    SETTINGS.setSerialMode(serial);
}

void SettingsWidget::changeVisionIp(QString ip) {
    SETTINGS.setVisionIp(ip.toStdString());
}

void SettingsWidget::changeVisionPort(int port) {
    SETTINGS.setVisionPort(port);
}

void SettingsWidget::changeRefereeIp(QString ip) {
    SETTINGS.setRefereeIp(ip.toStdString());
}

void SettingsWidget::changeRefereePort(int port) {
    SETTINGS.setRefereePort(port);
}

void SettingsWidget::changeGrSimIp(QString ip) {
    std::cout << "setting grsimip" << std::endl;
    SETTINGS.setRobothubSendIp(ip.toStdString());
}

void SettingsWidget::changeGrSimPort(int port) {
    std::cout << "setting grsimport" << std::endl;
    SETTINGS.setRobothubSendPort(port);
}

} // interface
} // ai
} // rtt


// QT performance improvement
#include "include/roboteam_ai/interface/widgets/moc_SettingsWidget.cpp"
