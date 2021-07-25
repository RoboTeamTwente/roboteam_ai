//
// Created by mrlukasbos on 8-4-19.
//

#ifndef ROBOTEAM_AI_PIDBOX_H
#define ROBOTEAM_AI_PIDBOX_H

#include "interface/api/Output.h"
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>

namespace rtt::ai::interface {

class PidBox : public QGroupBox {
    Q_OBJECT
   private:
    QHBoxLayout *spinBoxLayout;
    QDoubleSpinBox *select_p;
    QDoubleSpinBox *select_i;
    QDoubleSpinBox *select_d;
    pidVals pid;

   public slots:
    void updatePID();

   public:
    const pidVals &getPid() const;
    void setPid(const pidVals &pid);
    explicit PidBox(const QString &title, QWidget *parent = nullptr);

   signals:
    void pidChanged(pidVals newPid);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_PIDBOX_H
