//
// Created by mrlukasbos on 1-9-19.
//

#ifndef RTT_SETTINGSWIDGET_H
#define RTT_SETTINGSWIDGET_H

#include <utilities/Settings.h>

#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

namespace rtt::ai::interface {

class SettingsWidget : public QWidget {
    Q_OBJECT

   private:
    QVBoxLayout *vLayout;
    QLineEdit *grsimIpText;
    QSpinBox *grsimPort;

   public:
    explicit SettingsWidget(QWidget *parent = nullptr);

   public slots:
    void changeTeamColor(bool isYellow);
    void changeTeamSide(bool isLeft);
    void changeRobotHubMode(Settings::RobotHubMode mode);
    void changeGrSimIp(QString ip);
    void changeGrSimPort(int port);
};

}  // namespace rtt::ai::interface

#endif  // RTT_SETTINGSWIDGET_H
