//
// Created by ratoone on 25-09-19.
//

#ifndef RTT_BASECONTROLLER_H
#define RTT_BASECONTROLLER_H

template <typename Input, typename Output>
/**
 * The base abstract class for the controllers. Any control method should inherit from this
 * @tparam Input
 * @tparam Output
 */
class BaseController{
    public:
        /**
         * Compute the controller output. The method in the base class should always be implemented
         * @param setPointInput the setpoint of the controller
         * @param actualInput the real value
         * @return the output that needs to be fed into the plant
         */
        virtual Output computeOutput(Input setPointInput, Input actualInput) = 0;
};

#endif //RTT_BASECONTROLLER_H
