//
// Created by mrlukasbos on 1-9-19.
//

#ifndef RTT_SETTINGSWIDGET_H
#define RTT_SETTINGSWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>

#include "roboteam_world/world/settings.hpp"

namespace rtt::ai::interface {

class SettingsWidget : public QWidget {
Q_OBJECT

private:
    QVBoxLayout *vLayout;
    QLineEdit * grsimIpText;
    QSpinBox * grsimPort;
    ::rtt::world::settings::Settings& settings;
public:
    SettingsWidget(QWidget*parent, ::rtt::world::settings::Settings& settings);

public slots:
    void changeTeamColor(bool isYellow);
    void changeTeamSide(bool isLeft);
    void changeMode(bool serial);
    void changeGrSimIp(QString ip);
    void changeGrSimPort(int port);
};

}


#endif //RTT_SETTINGSWIDGET_H
