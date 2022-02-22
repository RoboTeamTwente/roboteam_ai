//
// Created by emiel on 21-02-22.
//

#include "utilities/WebSocketManager.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>
#include "google/protobuf/util/json_util.h"

#include <iostream>

namespace rtt::ai::io {

WebSocketManager& WebSocketManager::instance() {
    static WebSocketManager instance;
    return instance;
}

// r = 114. t = 116. rtt = 11400+1160+116 = 12676
WebSocketManager::WebSocketManager() : webSocketServer(12676){

    webSocketServer.setOnConnectionCallback(
        [this](const std::weak_ptr<ix::WebSocket>& webSocket, const std::shared_ptr<ix::ConnectionState>& connectionState){
            RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp())

            std::shared_ptr<ix::WebSocket> ws = webSocket.lock();
            if(ws){
                ws->setOnMessageCallback(

                    [webSocket, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg){
                        RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId(), ", ", WebSocketMessageTypeToString(msg->type));
                        if(msg->type == ix::WebSocketMessageType::Message){
                            RTT_INFO("Message: ", msg->str);
                            auto ws = webSocket.lock();
                            if(ws){
                                ws->send("Message received by WebSocketManager", false);
                            }
                        }
                    }

                );
            }
        }
    );

    std::pair<bool, std::string> res = webSocketServer.listen();
    if(!res.first){
        RTT_ERROR("Could not start WebSocketServer :", res.second);
    }else{
        webSocketServer.start();
    }

}

std::string WebSocketManager::WebSocketMessageTypeToString(ix::WebSocketMessageType type) {
    switch (type) {
        case ix::WebSocketMessageType::Message:
            return "MESSAGE";
        case ix::WebSocketMessageType::Open:
            return "OPEN";
        case ix::WebSocketMessageType::Close:
            return "CLOSE";
        case ix::WebSocketMessageType::Error:
            return "ERROR";
        case ix::WebSocketMessageType::Ping:
            return "PING";
        case ix::WebSocketMessageType::Pong:
            return "PONG";
        case ix::WebSocketMessageType::Fragment:
            return "FRAGMENT";
        default:
            return "UNKNOWN; fix function pl0x;";
    }
}

void WebSocketManager::broadcastWorld(){
    static float avg;

    auto started = std::chrono::high_resolution_clock::now();

    auto state = io::io.getState();
    /* JSON */
    std::string state_str;
    google::protobuf::util::MessageToJsonString(state, &state_str);
    /* Binary. Around 10 times as fast */
//    std::string state_str = state.SerializeAsString();

    // Broadcast to all connected clients
    for(const auto& client : webSocketServer.getClients()){
        client->send(state_str, false);
    }

    auto done = std::chrono::high_resolution_clock::now();
    int us = std::chrono::duration_cast<std::chrono::microseconds>(done-started).count();
    avg = avg * 0.95 + us*0.05;
    std::cout << "broadcast duration: " << avg << " : " << us << std::endl;
}

}