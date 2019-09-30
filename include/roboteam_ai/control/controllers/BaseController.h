//
// Created by ratoone on 25-09-19.
//

#ifndef RTT_BASECONTROLLER_H
#define RTT_BASECONTROLLER_H

template <typename Input, typename Output>
class BaseController{
    public:
        virtual Output computeOutput(Input setPointInput, Input actualInput) = 0;
};

#endif //RTT_BASECONTROLLER_H
