//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_LASTWORLD_H
#define ROBOTEAM_AI_LASTWORLD_H

#include "World.h"

namespace rtt {
namespace ai {

class WorldBuffer {
    private:
        WorldData* worldBuffer;
        unsigned int size;
        unsigned int lastIndex;
    public:
        WorldBuffer() :
                WorldBuffer(20) {};

        explicit WorldBuffer(unsigned int size) {
            WorldBuffer::size = size;
            worldBuffer = new WorldData[size];
            lastIndex = 0;
        }

        ~WorldBuffer() {
            delete[] worldBuffer;
        }

        void addWorld(WorldData &world) {
            worldBuffer[lastIndex--] = world;
            if (lastIndex < 0) lastIndex = size - 1;
        }

        const WorldData &getLastWorld(const unsigned int w) {
            unsigned int location = lastIndex + w + 1;
            location %= size;
            return worldBuffer[location];
        }
};

class LastWorld : public World {
    protected:
        WorldBuffer worldBuffer;
    public:
        const WorldData &getLastWorld(unsigned int w = 0);
        void updateLastWorld() override;
        virtual void updatePredictedWorld() = 0;

};

}
}

#endif //ROBOTEAM_AI_LASTWORLD_H
