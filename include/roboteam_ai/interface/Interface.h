//
// Created by Dawid Kulikowski on 08/08/2021.
//

#ifndef RTT_INTERFACE_H
#define RTT_INTERFACE_H

#include <roboteam_proto/State.pb.h>
#include <atomic>
#include <optional>

class Interface  {
   private:
    std::atomic<bool> doesNeedUpdate;

   public:
    Interface();

    void handleUpdate(proto::ModuleState);

    void registerChange();
    std::optional<proto::ModuleState> getChanges();
};

#endif  // RTT_INTERFACE_H
