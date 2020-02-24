//
// Created by ratoone on 20-02-20.
//

#ifndef RTT_PATHPOINTNODE_H
#define RTT_PATHPOINTNODE_H

#include <roboteam_utils/Vector2.h>

namespace rtt::ai::control{
class PathPointNode {
private:
    Vector2 position;
    double time;
    PathPointNode *parent = nullptr;
public:
    explicit PathPointNode(const Vector2& position);

    [[nodiscard]] const Vector2 &getPosition() const;

    [[nodiscard]] double getTime() const;

    [[nodiscard]] PathPointNode *getParent() const;

    void setParent(PathPointNode &parent);
};
}

#endif //RTT_PATHPOINTNODE_H
