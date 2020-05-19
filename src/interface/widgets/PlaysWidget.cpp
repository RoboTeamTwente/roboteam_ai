//
// Created by john on 4/30/20.
//

#include <include/roboteam_ai/ApplicationManager.h>
#include "include/roboteam_ai/interface/widgets/PlaysWidget.hpp"

namespace rtt::ai::interface {
    inline QString formatPlay(stp::Play* play)
    {
        auto const& field = world_new::World::instance()->getField();
        auto const& world = world_new::World::instance()->getWorld();
        if (!world.has_value()) {
            return "Unable to read world...";
        }

        QString ss = "";
        ss += play->getName();
        ss += ":<br>&nbsp;&nbsp;keep:<br>";
        for (auto& each : play->keepPlayInvariants)
        {
            ss += "&nbsp;&nbsp;&nbsp;&nbsp;";
            ss += each->getName();
            ss += ":&nbsp;";
            ss += (each->checkInvariant(world.value(), &*field) ? "true" : "false");
            ss += "<br>";
        }

        ss += "&nbsp;&nbsp;start:<br>";
        for (auto& each : play->startPlayInvariants)
        {
            ss += "&nbsp;&nbsp;&nbsp;&nbsp;";
            ss += each->getName();
            ss += ":&nbsp;";
            ss += (each->checkInvariant(world.value(), &*field) ? "true" : "false");
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
