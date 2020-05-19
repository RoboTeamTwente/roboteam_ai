//
// Created by john on 4/30/20.
//

#include <include/roboteam_ai/ApplicationManager.h>
#include "include/roboteam_ai/interface/widgets/PlaysWidget.hpp"

namespace rtt::ai::interface {
    inline QString formatPlay(stp::Play* play)
    {
        auto* data = world_new::World::instance();
        auto const& field = *data->getField();
        auto const& world = *data->getWorld();
        QString ss = "";
        ss += play->getName();
        ss += ":<br>  keep:<br>";
        for (auto& each : play->keepPlayInvariants)
        {
            ss += "    ";
            ss += each->getName();
            ss += ": ";
            ss += each->checkInvariant(world, &field);
            ss += "<br>";
        }

        ss += ":<br>  start:<br>";
        for (auto& each : play->startPlayInvariants)
        {
            ss += "    ";
            ss += each->getName();
            ss += ": ";
            ss += each->checkInvariant(world, &field);
            ss += "<br>";
        }
        ss += "<br>";
        return ss;
    }
	
    PlaysWidget::PlaysWidget(QWidget* parent) : QTextEdit(parent) {
        setReadOnly(true);
    }

	/**
	 * NAME:
	 *   keep:
	 *     Invariant -> true / false
	 *   start:
	 */
    void PlaysWidget::updatePlays() {
        QString ss = {};
        for (auto& each : ApplicationManager::plays) {
            ss += formatPlay(each.get());
        }
        auto sliderPos = verticalScrollBar()->sliderPosition();
        setHtml(ss);
        verticalScrollBar()->setSliderPosition(sliderPos);
    }
}
