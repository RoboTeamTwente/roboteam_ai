//
// Created by rolf on 07-03-21.
//

#ifndef RTT_ROBOTEAM_AI_SRC_AISETTINGS_H_
#define RTT_ROBOTEAM_AI_SRC_AISETTINGS_H_

#include <string>
#include <roboteam_proto/Handshake.pb.h>
namespace rtt {
  class AISettings {
   public:
    explicit AISettings(int id);


    [[nodiscard]] int getId() const;

    [[nodiscard]] bool isYellow() const;
    void setYellow(bool isYellow);

    [[nodiscard]] bool isLeft() const;
    void setLeft(bool left);

    [[nodiscard]] bool isPaused() const;
    void setPause(bool paused);

    [[nodiscard]] bool getListenToReferee() const;
    void setListenToReferee(bool listen);

    [[nodiscard]] proto::Handshake getButtonDeclarations() const;
    [[nodiscard]] proto::Handshake getValues() const;
    void updateValuesFromInterface(const proto::UiValues& values);
   private:
    [[nodiscard]] std::string name() const;

    const int id;
    bool is_yellow;
    bool is_left;

    bool is_paused;
    bool listenToReferee;

  };
}
#endif //RTT_ROBOTEAM_AI_SRC_AISETTINGS_H_
