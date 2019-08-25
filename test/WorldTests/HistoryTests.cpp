#include <gtest/gtest.h>
#include "../../src/world/History.h"

namespace rtt {
namespace ai {

TEST(HistoryTest, it_saves_world_states) {
    world::History history;

    world::WorldData wd;
    wd.time = 0.0;

    world::WorldData wd2;
    wd2.time = 1.0;

    std::shared_ptr<world::WorldData> wd3 = std::make_shared<world::WorldData>();
    wd3->time = 2.0;

    std::shared_ptr<world::WorldData> wd4 = std::make_shared<world::WorldData>();
    wd4->time = 3.0;

    history.addWorld(wd);
    history.addWorld(wd2);
    history.addWorld(wd3);
    history.addWorld(wd4);

    EXPECT_EQ(history.getPreviousWorld(0).time, 3.0);
    EXPECT_EQ(history.getPreviousWorld(1).time, 2.0);
    EXPECT_EQ(history.getPreviousWorld(2).time, 1.0);
    EXPECT_EQ(history.getPreviousWorld(3).time, 0.0);
}
}
}