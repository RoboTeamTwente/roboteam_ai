//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_LASTWORLD_H
#define ROBOTEAM_AI_LASTWORLD_H

namespace rtt {
namespace ai {

//class WorldBuffer {
//    private:
//        world::WorldData* worldBuffer;
//        unsigned int size;
//        unsigned int lastIndex;
//    public:
//        WorldBuffer() :
//                WorldBuffer(20) {};
//
//        explicit WorldBuffer(unsigned int size) {
//            WorldBuffer::size = size;
//            worldBuffer = new world::WorldData[size];
//            lastIndex = 0;
//        }
//
//        ~WorldBuffer() {
//            delete[] worldBuffer;
//        }
//
//        void addWorld(world::WorldData &world) {
//            worldBuffer[lastIndex--] = world;
//            if (lastIndex < 0) lastIndex = size - 1;
//        }
//
//        const world::WorldData &getLastWorld(const unsigned int w) {
//            unsigned int location = lastIndex + w + 1;
//            location %= size;
//            return worldBuffer[location];
//        }
//};

class LastWorld {
};

}
}

#endif //ROBOTEAM_AI_LASTWORLD_H
