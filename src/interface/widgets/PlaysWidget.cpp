//
// Created by john on 4/30/20.
//

#include <include/roboteam_ai/ApplicationManager.h>
#include "include/roboteam_ai/interface/widgets/PlaysWidget.hpp"

namespace rtt::ai::interface {
    PlaysWidget::PlaysWidget(QWidget* parent) : QTextEdit(parent) {
        setReadOnly(true);
    }

    void PlaysWidget::updatePlays() {
        std::lock_guard mtxLck{ dataMtx };
        for (auto& each : ApplicationManager::plays) {
            data << each->getName() << " -> " << static_cast<int>(each->score(world_new::World::instance())) << "<br>";
        }
        auto sliderPos = verticalScrollBar()->sliderPosition();
        setHtml(QString::fromStdString(data.str()));
        verticalScrollBar()->setSliderPosition(sliderPos);
    }
}
