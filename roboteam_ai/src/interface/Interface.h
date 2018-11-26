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
        void handleMouseClick(SDL_Event event);
    private:
        void drawField(roboteam_msgs::GeometryFieldSize field);
        void drawRobots();
        void drawBall();
        void drawText(std::string text, int x, int y, SDL_Color textColor = constants::TEXT_COLOR);
        void drawLine(Vector2 p1, Vector2 p2, SDL_Color color);
        void drawRect(Vector2 position, int w, int h, SDL_Color color);
        void drawRobot(roboteam_msgs::WorldRobot robot, bool ourTeam);
        void drawSideBar();

        std::shared_ptr<roboteam_msgs::WorldRobot> currentlySelectedRobot = nullptr;

        SDL_Renderer *renderer = nullptr;
        SDL_Window* window = nullptr;
        TTF_Font * font = nullptr;
        Vector2 factor;
        int fieldmargin;
        Vector2 toScreenPosition(Vector2 fieldPos);

        // map colors to tactic to visualize which robots work together
        std::vector<std::pair<std::string, SDL_Color>> tacticColors;
        int tacticCount = 0; // increases when a new tactic is used
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_INTERFACE_H

