#include "interface/InterfaceControllerServer.h"
#include <utilities/Settings.h>
#include <roboteam_utils/Print.h>
#include <utilities/GameStateManager.hpp>

namespace rtt::Interface {


void InterfaceControllerServer::handleData(const proto::UiValues &state) {
    this->vals->handleData(state, this->decls);
    for (const auto& val : state.ui_values()) {
        // Handle settings from the interface
        if (val.first == "IS_YELLOW" && val.second.has_bool_value()) {
            SETTINGS.setYellow(val.second.bool_value());
        } else if (val.first == "IS_RIGHT" && val.second.has_bool_value()) {
            SETTINGS.setLeft(! val.second.bool_value());
        } else if (val.first == "IGNORE_INVARIANTS" && val.second.has_bool_value()) {
            SETTINGS.setIgnoreInvariants(val.second.bool_value());
        } else if (val.first == "USE_REFEREE" && val.second.has_bool_value()) {
            SETTINGS.setUseReferee(val.second.bool_value());

            if (val.second.bool_value()) {
                auto plays = decls->getDeclaration("PLAY_SELECTOR");
                plays->isMutable = false;
                if (!plays.has_value()) return;

                decls->addDeclaration(*plays);
            } else {
                auto plays = decls->getDeclaration("PLAY_SELECTOR");
                plays->isMutable = true;
                if (!plays.has_value()) return;

                decls->addDeclaration(*plays);
            }
        } else if (val.first == "PLAY_SELECTOR" && val.second.has_text_value()) {
            this->setSelectedPlay(val.second.text_value());
        } else {
            std::cout << "Unhandled interface setting: '" << val.first << "'" << std::endl;
        }
    }
}

proto::ModuleState InterfaceControllerServer::getDataForRemote() const noexcept {
    proto::ModuleState mod;
    proto::Handshake hand;
    hand.set_module_name("IFACE");

    auto val = this->vals->toProto();
    hand.mutable_values()->Swap(&val);
    auto decls = this->decls->toProto();

    hand.mutable_declarations()->Swap(&decls);

    mod.mutable_handshakes()->Add(std::move(hand));

    return mod;
}

std::string InterfaceControllerServer::getSelectedPlay() {
    return this->selectedPlay;
}

void InterfaceControllerServer::setSelectedPlay(const std::string &newPlay) {
    if (newPlay != this->selectedPlay) {
        rtt::ai::GameStateManager::updateInterfaceGameState(newPlay.c_str());
    }

    this->selectedPlay = newPlay;
}

void InterfaceControllerServer::loop() {
    while (this->shouldRun) {
        this->update_marker.acquire();
        loop_iter();
    }
}

void InterfaceControllerServer::trigger_update() {
    this->update_marker.release();
}

}  // namespace rtt::Interface