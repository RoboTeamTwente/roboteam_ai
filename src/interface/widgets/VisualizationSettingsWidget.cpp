//
// Created by mrlukasbos on 7-5-19.
//

#include "VisualizationSettingsWidget.h"
#include "mainWindow.h"
#include "../api/Toggles.h"

namespace rtt {
namespace ai {
namespace interface {

VisualizationSettingsWidget::VisualizationSettingsWidget(Visualizer * visualizer, QWidget * parent) {
    auto cbVLayout = new QVBoxLayout();

    for (int i = 0; i < Toggles::toggles.size(); i++) {
        auto customToggle = new QWidget;
        auto hbox = new QHBoxLayout();

        // set the label
        auto label = new QLabel(Toggles::toggles[i].title);
        hbox->addWidget(label);

        // get the strategy names from Switches
        auto select = new QComboBox();
        hbox->addWidget(select);

        if (Toggles::toggles[i].showType == GENERAL) {
            select->addItem("Off");
            select->addItem("On");

            std::vector<QString> colors = { "red", "green"};
            select->setCurrentIndex(Toggles::toggles[i].generalShowType);
            select->setStyleSheet("QComboBox { background-color: " + colors[Toggles::toggles[i].generalShowType] + " }");

            QObject::connect(select, static_cast<void (QComboBox::*)(const int)>(&QComboBox::activated),
                    [=](const int index) {
                      Toggles::toggles[i].generalShowType = static_cast<GeneralShowType>(index);
                      select->setStyleSheet("QComboBox { background-color: " + colors[static_cast<GeneralShowType>(index)] + " }");
                    });
        }
        else if (Toggles::toggles[i].showType == ROBOT) {
            select->addItem("No robots");
            select->addItem("Selected robots");
            select->addItem("All robots");

            std::vector<QString> colors = { "red", "#888800", "green"};
            select->setCurrentIndex(Toggles::toggles[i].robotShowType);
            select->setStyleSheet("QComboBox { background-color: " + colors[Toggles::toggles[i].robotShowType] + " }");

            QObject::connect(select, static_cast<void (QComboBox::*)(const int)>(&QComboBox::activated),
                    [=](const int index) {
                      Toggles::toggles[i].robotShowType = static_cast<RobotShowType>(index);
                      select->setStyleSheet("QComboBox { background-color: " + colors[static_cast<RobotShowType>(index)] + " }");
                    });
        }


        customToggle->setLayout(hbox);
        cbVLayout->addWidget(customToggle);
    }

    auto cbVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    cbVLayout->addSpacerItem(cbVSpacer);
    this->setLayout(cbVLayout);
}



} // interface
} // ai
} // rtt

// QT performance improvement
#include "moc_VisualizationSettingsWidget.cpp"