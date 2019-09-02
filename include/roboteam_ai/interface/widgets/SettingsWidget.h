//
// Created by mrlukasbos on 1-9-19.
//

#ifndef RTT_SETTINGSWIDGET_H
#define RTT_SETTINGSWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>

namespace rtt {
namespace ai {
namespace interface {

class SettingsWidget : public QWidget {
Q_OBJECT

private:
    QVBoxLayout *vLayout;
    void updateLabels();
  QLineEdit * grsimIpText;
  QSpinBox * grsimPort;
public:
    explicit SettingsWidget(QWidget*parent = nullptr);



public slots:
    void changeTeamColor(bool isYellow);
    void changeTeamSide(bool isLeft);
    void changeMode(bool serial);
    void changeVisionIp(QString ip);
    void changeVisionPort(int port);
    void changeRefereeIp(QString ip);
    void changeRefereePort(int port);
    void changeGrSimIp(QString ip);
    void changeGrSimPort(int port);
};

}
}
}


#endif //RTT_SETTINGSWIDGET_H
