#include <QVBoxLayout>
#include <QCheckBox>
#include "include/roboteam_ai/interface/widgets/ManualControlWidget.h"

namespace rtt{
namespace interface {
ManualControlWidget::ManualControlWidget(QWidget *parent) : QWidget(parent) {
    auto layout = new QVBoxLayout();
    setLayout(layout);

    auto allowCheckBox = new QCheckBox();
    allowCheckBox->setText("Allow Manual takeover");
    allowCheckBox->setChecked(false);
    layout->addWidget(allowCheckBox);
    connect(allowCheckBox, &QCheckBox::toggled, [](bool checked) {
       // do whatever on toggle
    });

}

}
}