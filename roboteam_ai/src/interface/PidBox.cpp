//
// Created by mrlukasbos on 8-4-19.
//

#include "PidBox.h"
namespace rtt {
namespace ai {
namespace interface {

PidBox::PidBox(const QString &title, QWidget *parent) : QGroupBox(title, parent) {
    spinBoxLayout = new QHBoxLayout();

    select_p = new QDoubleSpinBox();
    select_p->setRange(-20, 20);
    select_p->setSingleStep(0.1f);
    QObject::connect(select_p, SIGNAL(valueChanged(double)), this, SLOT(updatePID()));
    spinBoxLayout->addWidget(select_p);

    select_i = new QDoubleSpinBox();
    select_i->setRange(-20, 20);
    select_i->setSingleStep(0.1f);
    QObject::connect(select_i, SIGNAL(valueChanged(double)), this, SLOT(updatePID()));
    spinBoxLayout->addWidget(select_i);

    select_d = new QDoubleSpinBox();
    select_d->setRange(-20, 20);
    select_d->setSingleStep(0.1f);
    QObject::connect(select_d, SIGNAL(valueChanged(double)), this, SLOT(updatePID()));
    spinBoxLayout->addWidget(select_d);

    this->setLayout(spinBoxLayout);
}

const pidVals &PidBox::getPid() const {
    return pid;
}

// update the checkbox values
void PidBox::setPid(const pidVals &pid) {
    select_p->setValue(std::get<0>(pid));
    select_i->setValue(std::get<1>(pid));
    select_d->setValue(std::get<2>(pid));
    this->pid = pid;
}

// update the pid and indicate as an outward signal that a change has occured.
void PidBox::updatePID() {
    pidVals newPid = pidVals(select_p->value(), select_i->value(), select_d->value());
    if (this->pid != newPid) {
        this->pid = newPid;
        emit pidChanged(this->pid);
    }
}


} // interface
} // ai
} // rtt
