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
        clear();
        QString ss;
        for (auto& each : ApplicationManager::plays) {
            ss += each->getName();
            ss += " -> ";
            ss += each->score(world_new::World::instance());
            ss += "<br>";
        }
        auto sliderPos = verticalScrollBar()->sliderPosition();
        setHtml(ss);
        verticalScrollBar()->setSliderPosition(sliderPos);
    }
}
