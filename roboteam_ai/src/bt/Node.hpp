#pragma once

#include <memory>
#include <vector>
#include <cstdio>
#include <cstdarg>

#include "Blackboard.hpp"
#include "../utilities/RobotDealer.h"

namespace bt {

class Node {
    public:
        // When this is updated, updated the tostring method below too!
        enum class Status {
                Waiting,
                Success,
                Failure,
                Running
        };

        std::string status_print(Status s) {
            switch (s) {
                case Status::Waiting:
                    return " Status : Waiting";
                case Status::Success:
                    return " Status : Success";
                case Status::Failure:
                    return " Status : Failure";
                case Status::Running:
                    return " Status : Running";
            }
        }

        virtual ~Node() = default;

        Node();

        using Ptr = std::shared_ptr<Node>;

        virtual Status update() = 0;

        Status NodeUpdate();
        void NodeInitialize();

        void NodeTerminate(Status s);

        virtual void initialize();

        virtual void terminate(Status s);

        virtual void addChild(bt::Node::Ptr);

        virtual std::vector<Node::Ptr> getChildren();

        virtual Status tick();

        bool IsSuccess() const;

        bool IsFailure() const;

        bool IsRunning() const;

        bool IsTerminated() const;

        Status getStatus() const;

        void setStatus(Status s);

        using dealer = robotDealer::RobotDealer;

        bt::Blackboard::Ptr properties = std::make_shared<bt::Blackboard>();

        bt::Blackboard::Ptr globalBB;

        virtual std::string node_name();

        void setProperties(bt::Blackboard::Ptr blackboard);

    protected:
        Status status = Status::Waiting;

        static void append_status(std::string fmt, ...);
};

std::string statusToString(bt::Node::Status status);

} // bt
