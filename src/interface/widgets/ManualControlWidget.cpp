#include "include/roboteam_ai/interface/widgets/ManualControlWidget.h"
#include <include/roboteam_ai/interface/api/Toggles.h>
#include <include/roboteam_ai/manual/JoystickManager.h>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QtWidgets/QComboBox>

namespace rtt::ai::interface {
ManualControlWidget::ManualControlWidget(QWidget *parent) : QWidget(parent) {
    auto layout = new QVBoxLayout();
    setLayout(layout);

    auto allowCheckBox = new QCheckBox();
    allowCheckBox->setText("Allow Manual takeover");
    allowCheckBox->setChecked(false);
    layout->addWidget(allowCheckBox);
    manager = new rtt::input::JoystickManager(&rtt::ai::io::io);

    QComboBox *comboBox = new QComboBox;
    //    for(int i = 0; i < 12; i++) {
    //        comboBox->addItem(tr("robots"));
    //    }
    layout->addWidget(comboBox);

    joyThread = std::thread(&rtt::input::JoystickManager::run, manager);

    connect(allowCheckBox, &QCheckBox::toggled, [this, comboBox](bool checked) {
        if (checked) {
            manager->activate();
        } else {
            manager->deactivate();
        }
    });
}

}  // namespace rtt::ai::interface