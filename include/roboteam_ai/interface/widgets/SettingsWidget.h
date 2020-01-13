//
// Created by mrlukasbos on 1-9-19.
//

#ifndef RTT_SETTINGSWIDGET_H
#define RTT_SETTINGSWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>

#include "include/roboteam_ai/settings/settings.hpp"

namespace rtt::ai::interface {

class SettingsWidget : public QWidget {
Q_OBJECT

private:
    QVBoxLayout *vLayout;
    QLineEdit * grsimIpText;
    QSpinBox * grsimPort;
    rtt::Settings& settings;
public:
    SettingsWidget(QWidget*parent, Settings& settings);

public slots:
    void changeTeamColor(bool isYellow);
    void changeTeamSide(bool isLeft);
    void changeMode(bool serial);
    void changeGrSimIp(QString ip);
    void changeGrSimPort(int port);
};

}


#endif //RTT_SETTINGSWIDGET_H
