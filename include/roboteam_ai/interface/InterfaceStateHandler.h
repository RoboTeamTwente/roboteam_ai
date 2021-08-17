//
// Created by Dawid Kulikowski on 16/08/2021.
//

#ifndef RTT_INTERFACESTATEHANDLER_H
#define RTT_INTERFACESTATEHANDLER_H

namespace rtt::Interface {

class InterfaceStateHandler {
    //    TODO: This is a pretty slow mechanism, maybe use atomic?
   private:
    bool didChange = false;
    mutable std::mutex mtx;

   public:
    void stateDidChange() {
        std::scoped_lock lck(mtx);
        didChange = true;
    }

    void clear() {
        std::scoped_lock lck(mtx);
        didChange = false;
    }
    bool getState() const {
        std::scoped_lock lck(mtx);
        return didChange;
    }
};
}  // namespace rtt::Interface

#endif  // RTT_INTERFACESTATEHANDLER_H
