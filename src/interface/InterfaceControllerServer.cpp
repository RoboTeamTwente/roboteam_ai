#include "interface/InterfaceControllerServer.h"
#include <utilities/Settings.h>
#include <roboteam_utils/Print.h>
#include <utilities/GameStateManager.hpp>

namespace rtt::Interface {

bool InterfaceControllerServer::hasPriorityData() const noexcept {
    return false;
}

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
        } else if (val.first == "PLAY_SELECTOR" && val.second.has_text_value()) {
            this->setSelectedPlay(val.second.text_value());
        } else {
            std::cout << "Unhandled interface setting: '" << val.first << "'" << std::endl;
        }
    }
}

proto::ModuleState InterfaceControllerServer::getDataForRemote(bool expired) const noexcept {
    auto state = rtt::ai::io::io.getState();

    proto::ModuleState mod;
    mod.mutable_system_state()->Swap(&state);

    if (expired) {
        proto::Handshake hand;
        hand.set_module_name("IFACE");

        auto val = this->vals->toProto();
        hand.mutable_values()->Swap(&val);

        auto dec = this->decls->toProto();
        hand.mutable_declarations()->Swap(&dec);

        mod.mutable_handshakes()->Add(std::move(hand));
    }

    return mod;
}

std::string InterfaceControllerServer::getSelectedPlay() {
    //std::cout << "Current interface play: " << this->selectedPlay << std::endl;
    return this->selectedPlay;
//    std::string selectedPlay = "";
//
//    auto pl = this->vals->getSetting("PLAY_SELECTOR");
//    if (pl.has_value()) {
//        RTT_DEBUG("HAS VALUE")
//    }
//
//    auto interfacePlaySelector = this->decls->getDeclaration("PLAY_SELECTOR");;
//    if (interfacePlaySelector.has_value()) {
//        auto dropdownMenu = std::get_if<Interface::InterfaceDropdown>(&interfacePlaySelector.value().options);
//        selectedPlay = dropdownMenu->text;
//    } else {
    //    }
    //
//        RTT_DEBUG("No play selector yet")
//    return selectedPlay;
}

void InterfaceControllerServer::setSelectedPlay(const std::string &newPlay) {
    if (newPlay != this->selectedPlay) {
        RTT_DEBUG("Changed play to: ", newPlay)
        rtt::ai::GameStateManager::updateInterfaceGameState(newPlay.c_str());
    }

    this->selectedPlay = newPlay;
}

} // namespace rtt::Interface