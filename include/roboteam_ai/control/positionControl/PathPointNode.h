//
// Created by ratoone on 20-02-20.
//

#ifndef RTT_PATHPOINTNODE_H
#define RTT_PATHPOINTNODE_H

#include <roboteam_utils/Vector2.h>

namespace rtt::ai::control {
/**
 * A node in the path tree generation. Has a position and a parent
 */
class PathPointNode {
   private:
    Vector2 position;
    PathPointNode *parent = nullptr;

   public:
    /**
     * Create a new tree node with a null parent (this will usually be the root of the tree)
     * @param position the position represented by the node
     */
    explicit PathPointNode(const Vector2 &position);

    /**
     * Create a new tree node with the specified parent
     * @param position the position represented by the node
     * @param parent the parent node of the current node
     */
    PathPointNode(const Vector2 &position, PathPointNode &parent);

    [[nodiscard]] const Vector2 &getPosition() const;

    [[nodiscard]] PathPointNode *getParent() const;
};
}  // namespace rtt::ai::control

#endif  // RTT_PATHPOINTNODE_H
