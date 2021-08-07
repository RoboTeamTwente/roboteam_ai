//
// Created by rolf on 07-03-21.
//

#include "AppSettings.h"
#include <roboteam_utils/Print.h>

static std::string serialModeName(SerialMode mode){
  switch (mode) {
    case SerialMode::GRSIM: return "GrSim";
    case SerialMode::SERIAL: return "Serial";
  }
}

AppSettings::AppSettings() :
referee_ip("224.5.23.1"),
referee_port(10003),
vision_ip("224.5.23.2"),
vision_port(10006),
robothub_ip("127.0.0.1"),
robothub_port(20011),
mode{SerialMode::GRSIM}{

}
const std::string &AppSettings::getRefereeIp() const {
  return referee_ip;
}
void AppSettings::setRefereeIp(const std::string &refereeIp) {
  referee_ip = refereeIp;
}
int AppSettings::getRefereePort() const {
  return referee_port;
}
void AppSettings::setRefereePort(int port) {
  referee_port = port;
}
const std::string &AppSettings::getVisionIp() const {
  return vision_ip;
}
void AppSettings::setVisionIp(const std::string &visionIp) {
  vision_ip = visionIp;
}
int AppSettings::getVisionPort() const {
  return vision_port;
}
void AppSettings::setVisionPort(int port) {
  vision_port = port;
}
proto::ObserverSettings AppSettings::obsMessage() const{

  proto::ObserverSettings settings;
  settings.set_visionip(vision_ip);
  settings.set_visionport(vision_port);
  settings.set_refereeip(referee_ip);
  settings.set_refereeport(referee_port);
  return settings;
}
proto::Setting AppSettings::toMessage() const {
  proto::Setting setting;
  setting.mutable_obs_settings()->CopyFrom(obsMessage());
  setting.mutable_rh_settings()->CopyFrom(rhMessage());
  return setting;
}

proto::RobotHubSettings AppSettings::rhMessage() const {
  proto::RobotHubSettings settings;
  settings.set_robothubip(robothub_ip);
  settings.set_robothubport(robothub_port);
  settings.set_serialmode(mode == SerialMode::SERIAL);//TODO: extend proto with an enum to support more modes
  return settings;
}
proto::Handshake AppSettings::getButtonDeclarations() const {
  //referee ip
  proto::UiOptionDeclarations declarations;
  proto::UiOptionDeclaration ref_ip_textbox;
  ref_ip_textbox.set_path("referee_ip");
  ref_ip_textbox.set_is_mutable(false);
  proto::TextField ref_ip_text;
  ref_ip_text.set_text(referee_ip);
  ref_ip_textbox.mutable_textfield()->CopyFrom(ref_ip_text);
  //referee port
  proto::UiOptionDeclaration ref_port_textbox;
  ref_port_textbox.set_path("referee_port");
  ref_port_textbox.set_is_mutable(false);
  proto::TextField ref_port_text;
  ref_port_text.set_text(std::to_string(referee_port));
  ref_port_textbox.mutable_textfield()->CopyFrom(ref_ip_text);

  // serial mode dropdown
  proto::UiOptionDeclaration serial_mode_dropdown;
  serial_mode_dropdown.set_path("serial_mode");
  serial_mode_dropdown.set_is_mutable(false);
  proto::Dropdown dropdown;
  dropdown.set_text("serial_mode");
  dropdown.set_default_(0);
  dropdown.mutable_options()->Add()->append(serialModeName(SerialMode::GRSIM));
  dropdown.mutable_options()->Add()->append(serialModeName(SerialMode::SERIAL));
  serial_mode_dropdown.mutable_dropdown()->CopyFrom(dropdown);


  declarations.mutable_options()->Add(std::move(ref_ip_textbox));
  declarations.mutable_options()->Add(std::move(ref_port_textbox));
  declarations.mutable_options()->Add(std::move(serial_mode_dropdown));

  proto::Handshake message;
  message.set_module_name("application");
  message.mutable_declarations()->CopyFrom(declarations);
  return message;
}
proto::Handshake AppSettings::getValues() const {

  proto::UiValue ref_ip;
  ref_ip.set_text_value(referee_ip);

  proto::UiValue ref_port;
  ref_port.set_text_value(std::to_string(referee_port));

  proto::UiValue serial_mode;
  serial_mode.set_integer_value(static_cast<long>(mode));
  proto::UiValues values;
  (*values.mutable_ui_values())["referee_ip"] = ref_ip;
  (*values.mutable_ui_values())["referee_port"] = ref_port;
  (*values.mutable_ui_values())["serial_mode"] = serial_mode;

  proto::Handshake message;
  message.set_module_name("application");
  message.mutable_values()->CopyFrom(values);

  return message;
}
void AppSettings::updateValuesFromInterface(const proto::UiValues &values) {
   if(values.ui_values().contains("referee_ip")){
     proto::UiValue value = values.ui_values().at("referee_ip");
     if(value.value_case() == proto::UiValue::kTextValue){
       referee_ip = value.text_value();
     }else{
       RTT_ERROR("\"referee_ip\" did not have the correct type of \"text\" in received message from interface");
     }
   }

  if(values.ui_values().contains("referee_port")){
    proto::UiValue value = values.ui_values().at("referee_port");
    if(value.value_case() == proto::UiValue::kTextValue){
      try {
        int found = std::stoi(value.text_value());
        referee_port = found;
      }catch (...){
        RTT_ERROR("Could not convert referee_port string to integer");
      }
    }else{
      RTT_ERROR("\"referee_port\" did not have the correct type of \"text\" in received message from interface");
    }
  }

  if(values.ui_values().contains("serial_mode")){
    proto::UiValue value = values.ui_values().at("serial_mode");
    if(value.value_case() == proto::UiValue::kIntegerValue){
      SerialMode new_mode = static_cast<SerialMode>(value.integer_value());
      bool valid = true;
      switch(new_mode){
        case SerialMode::GRSIM:
        case SerialMode::SERIAL:
          break;
        default: RTT_ERROR("Wrong integer conversion for serial mode!"); valid = false;
      }
      if(valid){
        mode = new_mode;
      }
    }else{
      RTT_ERROR("\"serial_mode\" did not have the correct type of \"integer\" in received message from interface");
    }
  }
}



