//
// Created by rolf on 07-03-21.
//

#include "AISettings.h"
namespace rtt {
AISettings::AISettings(int initial_id) :
    id{initial_id},
    is_paused(true),
    listenToReferee(false){
  //these settings are for testing and convenience, basically
  if (initial_id == 1) {
    is_yellow = false;
    is_left = false;
  } else {
    is_yellow = true;
    is_left = true;
  }
}

int AISettings::getId() const {
  return id;
}

bool AISettings::isYellow() const {
  return is_yellow;
}
void AISettings::setYellow(bool isYellow) {
  is_yellow = isYellow;

}
bool AISettings::isLeft() const {
  return is_left;
}
void AISettings::setLeft(bool left) {
  is_left = left;
}


void AISettings::setPause(bool paused) {
  is_paused = paused; //TODO: when the AI is just paused, send a halt command to all robots
}
bool AISettings::isPaused() const {
  return is_paused;
}
bool AISettings::getListenToReferee() const {
  return listenToReferee;
}
void AISettings::setListenToReferee(bool listen) {
  listenToReferee = listen;
}

std::string AISettings::name() const {
  return "ai"+std::to_string(id);
}
proto::Handshake AISettings::getButtonDeclarations() const{

  proto::UiOptionDeclaration pause_button;
  pause_button.set_name("pause_button");
  pause_button.set_is_mutable(false);
  proto::Checkbox pause_box;
  pause_box.set_text("Pause");
  pause_box.set_default_(is_paused);
  pause_button.mutable_checkbox()->CopyFrom(pause_box);

  proto::UiOptionDeclaration listen_to_referee_button;
  listen_to_referee_button.set_name("listen_to_referee_button");
  listen_to_referee_button.set_is_mutable(false);
  proto::Checkbox referee_box;
  referee_box.set_text("Listen to Referee");
  referee_box.set_default_(listenToReferee);
  listen_to_referee_button.mutable_checkbox()->CopyFrom(referee_box);

  proto::UiOptionDeclaration side_button;
  side_button.set_name("side_button");
  side_button.set_is_mutable(true);
  proto::Checkbox side_box;
  side_box.set_text("We play left");
  side_box.set_default_(is_left);
  side_button.mutable_checkbox()->CopyFrom(side_box);

  proto::UiOptionDeclaration color_button;
  color_button.set_name("side_button");
  color_button.set_is_mutable(true);
  proto::Checkbox color_box;
  color_box.set_text("We are the yellow team");
  color_box.set_default_(is_yellow);
  color_button.mutable_checkbox()->CopyFrom(color_box);


  proto::UiOptionDeclarations declarations;
  declarations.mutable_options()->Add(std::move(pause_button));
  declarations.mutable_options()->Add(std::move(listen_to_referee_button));
  declarations.mutable_options()->Add(std::move(side_button));
  declarations.mutable_options()->Add(std::move(color_button));

  proto::Handshake message;
  message.set_module_name(name());
  message.mutable_declarations()->CopyFrom(declarations);

  return message;
}
}
