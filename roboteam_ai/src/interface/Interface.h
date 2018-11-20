//
// Created by mrlukasbos on 20-11-18.
//

#ifndef ROBOTEAM_AI_INTERFACE_H
#define ROBOTEAM_AI_INTERFACE_H

#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_utils/Vector2.h>
#include <SDL.h>
#include <iostream>
#include <SDL_ttf.h>
#include "../utilities/Constants.h"
#include "../utilities/Field.h"
#include "../utilities/World.h"
#include <exception>
#include <cmath>

namespace rtt {
namespace ai {
namespace interface {

class Interface {
    public:
        explicit Interface();
        ~Interface();
        void drawFrame();
    private:
        void drawField();
        void drawRobots();
        void drawBall();
        void drawText(std::string text, int x, int y);
        void drawLine(Vector2 p1, Vector2 p2, SDL_Color color);
        void drawRect(Vector2 position, int w, int h, SDL_Color color);
        void drawRobot(roboteam_msgs::WorldRobot robot, SDL_Color color);

        SDL_Renderer *renderer = nullptr;
        SDL_Window* window = nullptr;
        TTF_Font * font = nullptr;
        Vector2 factor;
        int fieldmargin;
        Vector2 toScreenPosition(Vector2 fieldPos);
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_INTERFACE_H

