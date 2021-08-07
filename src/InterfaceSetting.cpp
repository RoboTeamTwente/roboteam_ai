//
// Created by Dawid Kulikowski on 07/08/2021.
//

#include "InterfaceSetting.h"
#include <exception>


void InterfaceSetting::handleNewData(const proto::ModuleState newState) {
    if (newState.handshakes_size() != 1) {
            throw std::range_error{"No handshakes available"};
    }

    this->values = newState.handshakes().at(0).values();
}

