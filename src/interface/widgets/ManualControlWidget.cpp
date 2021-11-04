#include "interface/widgets/ManualControlWidget.h"

#include <QCheckBox>
#include <QVBoxLayout>

namespace rtt::ai::interface {
ManualControlWidget::ManualControlWidget(QWidget *parent) : QWidget(parent) {
    auto layout = new QVBoxLayout();
    setLayout(layout);

    auto allowCheckBox = new QCheckBox();
    allowCheckBox->setText("Allow Manual takeover");
    allowCheckBox->setChecked(false);
    layout->addWidget(allowCheckBox);

    joyThread = std::thread(&rtt::input::JoystickManager::run, &manager);

    connect(allowCheckBox, &QCheckBox::toggled, [this](bool checked) {
        if (checked) {
            manager.activate();
        } else {
            manager.deactivate();
        }
    });
}

}  // namespace rtt::ai::interface