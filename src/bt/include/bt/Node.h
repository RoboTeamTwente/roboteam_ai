#pragma once

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <vector>
#include "Blackboard.h"

// fwd declare
namespace rtt::ai::world {
//class World;
class Field;
}  // namespace rtt::ai::world

namespace rtt::world_new::view {
class WorldDataView;
}

namespace bt {
using namespace rtt::ai::world;

class Node {
   public:
    // When this is updated, updated the tostring method below too!
    enum class Status { Waiting, Success, Failure, Running };

    std::string status_print(Status s);

    virtual ~Node() = default;

    Node();

    using Ptr = std::shared_ptr<Node>;

    /**
     * The update function executes the node. This function is overwritten a lot, most interestingly in the Selector/Sequence/Inverter nodes,
     * since these nodes can have children. Therefore the update function of these nodes ticks their children.
     * @return Status result of the update (Running, Success, Failure)
     */
    virtual Status update() = 0;

    Status NodeUpdate();
    void NodeInitialize();

    void NodeTerminate(Status s);

    virtual void initialize();

    virtual void terminate(Status s);

    virtual void addChild(bt::Node::Ptr);

    virtual std::vector<Node::Ptr> getChildren();

    virtual Status tick(rtt::world_new::view::WorldDataView world, const Field *field);

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

    [[nodiscard]] unsigned long long getAmountOfTicks() const;

    /**
     * recursively goes through all the children of the node and sets their blackboard property ROLE to roleName,
     * @param roleName the name you want the role to have
     */
    void setRoleString(std::string roleName);

   protected:
    Status status = Status::Waiting;

    bool init = false;

    unsigned long long amountOfTicks = 0;  // ticks can increase fast

    rtt::world_new::view::WorldDataView world = nullptr;
    const Field *field = nullptr;
};

}  // namespace bt
