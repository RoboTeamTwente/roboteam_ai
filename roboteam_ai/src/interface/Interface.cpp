//
// Created by mrlukasbos on 20-11-18.
//

#include "Interface.h"

namespace rtt {
namespace ai {
namespace interface {

namespace c = rtt::ai::constants;

Interface::Interface() : renderer(nullptr) {
    // set up window
    window = SDL_CreateWindow("RTT AI Interface",
            c::WINDOW_POS_X,
            c::WINDOW_POS_Y,
            c::WINDOW_SIZE_X,
            c::WINDOW_SIZE_Y,
            0);
    if (window == nullptr) {
        std::cout << "Failed to create window : " << SDL_GetError();
    }
    // set up renderer
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        std::cout << "Failed to create renderer : " << SDL_GetError();
    }

    // load the font
    // we must not forget to destroy it in the destructor
    if (TTF_Init() < 0) {
        std::cout << "TTF library could not be initialized!!";
    }
    font = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeMono.ttf",14);

}

Interface::~Interface() {
    TTF_CloseFont(font);
    TTF_Quit();
}


void Interface::drawFrame() {
    drawField();
    drawRobots();

    // render to screen
    SDL_RenderPresent(renderer);
}

void Interface::clearFrame() {
    SDL_RenderClear(renderer);
}

void Interface::drawField() {
    // set background
    SDL_SetRenderDrawColor(renderer, c::FIELD_COLOR.r, c::FIELD_COLOR.g, c::FIELD_COLOR.b, c::FIELD_COLOR.a);
    SDL_RenderClear(renderer);

    factor.x = c::WINDOW_SIZE_X / Field::get_field().field_length;
    factor.y = c::WINDOW_SIZE_Y / Field::get_field().field_width;

    // draw field lines
    for (auto line : Field::get_field().field_lines) {
        drawLine(line.begin, line.end, c::FIELD_LINE_COLOR);
    }
}

void Interface::drawRobots() {
    // draw us
    for (roboteam_msgs::WorldRobot robot : World::get_world().us) {
        SDL_SetRenderDrawColor(renderer, c::ROBOT_US_COLOR.r, c::ROBOT_US_COLOR.g, c::ROBOT_US_COLOR.b, c::ROBOT_US_COLOR.a);

        SDL_Rect rect;
        rect.x = static_cast<int>(robot.pos.x * factor.x) + c::WINDOW_SIZE_X/2;;
        rect.y = static_cast<int>(robot.pos.y * factor.y) + c::WINDOW_SIZE_Y/2;;
        rect.w = 10;
        rect.h = 10;
        SDL_RenderFillRect(renderer, &rect);
    }

    // draw them
    for (roboteam_msgs::WorldRobot robot : World::get_world().them) {
        SDL_SetRenderDrawColor(renderer, c::ROBOT_THEM_COLOR.r, c::ROBOT_THEM_COLOR.g, c::ROBOT_THEM_COLOR.b, c::ROBOT_THEM_COLOR.a);

        SDL_Rect rect;
        rect.x = static_cast<int>(robot.pos.x * factor.x) + c::WINDOW_SIZE_X/2;
        rect.y = static_cast<int>(robot.pos.y * factor.y)+ c::WINDOW_SIZE_Y/2;
        rect.w = 10;
        rect.h = 10;
        SDL_RenderFillRect(renderer, &rect);

    }
}

void Interface::drawLine(Vector2 p1, Vector2 p2, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLine(renderer,
        static_cast<int>(p1.x * factor.x)+ c::WINDOW_SIZE_X/2,
        static_cast<int>(p1.y * factor.y)+ c::WINDOW_SIZE_Y/2,
        static_cast<int>(p2.x * factor.x)+ c::WINDOW_SIZE_X/2,
        static_cast<int>(p2.y * factor.y)+ c::WINDOW_SIZE_Y/2
        );
}


} // interface
} // ai
} // rtt