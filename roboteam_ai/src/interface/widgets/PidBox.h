//
// Created by mrlukasbos on 8-4-19.
//

#ifndef ROBOTEAM_AI_PIDBOX_H
#define ROBOTEAM_AI_PIDBOX_H

#include <QtWidgets/QGroupBox>

class QHBoxLayout;
class QDoubleSpinBox;

namespace rtt {
namespace ai {
namespace interface {

typedef std::tuple<double, double, double> pidVals;

class PidBox : public QGroupBox {
Q_OBJECT
private:
    QHBoxLayout* spinBoxLayout;
    QDoubleSpinBox* select_p;
    QDoubleSpinBox* select_i;
    QDoubleSpinBox* select_d;
    pidVals pid;

public slots:
    void updatePID();

public:
    const pidVals &getPid() const;
    void setPid(const pidVals &pid);
    explicit PidBox(const QString &title, QWidget * parent = nullptr);

signals:
    void pidChanged(pidVals newPid);
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_PIDBOX_H
