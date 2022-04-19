//
// Created by john on 4/30/20.
//

#include "interface/widgets/PlaysWidget.hpp"

#include "STPManager.h"

namespace rtt::ai::interface {
inline QString formatPlay(stp::Play* play) {
    std::optional<rtt::world::view::WorldDataView> world;
    std::optional<rtt::world::Field> field;

    {
        auto const& [_, worldOwner] = rtt::world::World::instance();
        field = worldOwner->getField();
        world = worldOwner->getWorld();
    }

    if (!world.has_value()) {
        return "Unable to read world...";
    }
    if (!field.has_value()) {
        return "Unable to read field...";
    }
    if (!world.value()) {
        return "World is null";
    }

    rtt::world::WorldData data = *world.value();

    QString ss = "";
    ss += play->getName();
    ss += ":<br>&nbsp;&nbsp;score: ";
    ss += QString::number(play->getLastScore());
    ss += "<br>";
    return ss;
}

PlaysWidget::PlaysWidget(QWidget* parent) : QTextEdit(parent) { setReadOnly(true); }

/**
 * NAME:
 *   keep:
 *     Invariant -> true / false
 *   start:
 */
void PlaysWidget::updatePlays() {
    QString ss = {};
    for (auto& each : STPManager::plays) {
        ss += formatPlay(each.get());
    }
    auto sliderPos = verticalScrollBar()->sliderPosition();
    setHtml(ss);
    verticalScrollBar()->setSliderPosition(sliderPos);
}
}  // namespace rtt::ai::interface
