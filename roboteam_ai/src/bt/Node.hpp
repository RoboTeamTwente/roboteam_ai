#pragma once

#include <memory>
#include <vector>
#include <cstdio>
#include <cstdarg>
#include "Blackboard.hpp"

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

        bt::Blackboard::Ptr properties = std::make_shared<bt::Blackboard>();

        bt::Blackboard::Ptr globalBB;

        virtual std::string node_name();

        virtual void giveProperty(std::string a, std::string b);

        void setProperties(bt::Blackboard::Ptr blackboard);

        unsigned long long getAmountOfTicks() const;

        ros::Time getLastTickTime();

    protected:
        Status status = Status::Waiting;

        bool init = false;

        unsigned long long amountOfTicks = 0; // ticks can increase fast

        ros::Time lastTickTime;
};

std::string statusToString(bt::Node::Status status);

} // bt
