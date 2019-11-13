#pragma once

#include <memory>
#include <vector>
#include <cstdio>
#include <cstdarg>

#include "Blackboard.hpp"

// fwd declare
namespace rtt {
    namespace ai {
    namespace world {
        class World;

        class Field;

    }
}
}

namespace bt {

class Node {
    public:
        // When this is updated, updated the tostring method below too!
        /**
         * Why mix enum with enum class
         * stick to enum class entirely
         * 
         * enum classes are literally just typesafe enums,
         * prevents implicit conversions to integers etc
         * 
         * enum class Example { one, two, three };
         * int _one = Example::one; // compiler error
         * int _one = static_cast<int>(Example::one); // ok
         * 
         * enum Example { one, two, three };
         * int _one = Example::one; // ok
         * int _one = static_cast<int>(Example::one); // ok
         * 
         * You probably know that implicit conversions are considered to be bad
         */
        enum class Status {
                Waiting,
                Success,
                Failure,
                Running
        };

        std::string status_print(Status s);

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

        virtual Status tick(rtt::ai::world::World * world, rtt::ai::world::Field * field);

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

//        ros::Time getLastTickTime();

    protected:
        Status status = Status::Waiting;

        bool init = false;

        unsigned long long amountOfTicks = 0; // ticks can increase fast

        rtt::ai::world::World * world = nullptr;
        rtt::ai::world::Field * field = nullptr;
};

} // bt
