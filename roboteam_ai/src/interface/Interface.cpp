/*
 * SDL coordinates are different from field coordinates.
 * So, use the draw[whatever] functions to draw stuff.
 * these functions automatically translate the axes for you.
 */

#include "Interface.h"

namespace rtt {
namespace ai {
namespace interface {

namespace c = rtt::ai::constants;

Interface::Interface() : renderer(nullptr) {
    // set up window
    window = SDL_CreateWindow("RTT AI Interface",
            c::WINDOW_POS_X, c::WINDOW_POS_Y, c::WINDOW_SIZE_X, c::WINDOW_SIZE_Y, 0);

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
    font = TTF_OpenFont("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",16);
    drawText("No world state has been received yet", 20, 20);
    SDL_RenderPresent(renderer);
}

Interface::~Interface() {
    TTF_CloseFont(font);
    TTF_Quit();
}

void Interface::drawFrame() {
    drawField();
    drawRobots();
    drawBall();

    // render to screen
    SDL_RenderPresent(renderer);
}

void Interface::drawField() {
    // set background
    SDL_SetRenderDrawColor(renderer, c::FIELD_COLOR.r, c::FIELD_COLOR.g, c::FIELD_COLOR.b, c::FIELD_COLOR.a);
    SDL_RenderClear(renderer);

    roboteam_msgs::GeometryFieldSize field = Field::get_field();

    fieldmargin = c::WINDOW_FIELD_MARGIN + field.boundary_width;
    factor.x = c::WINDOW_SIZE_X / field.field_length - (2 * fieldmargin);
    factor.y = c::WINDOW_SIZE_Y / field.field_width - (2 * fieldmargin);

    // draw field lines
    for (auto line : Field::get_field().field_lines) {
        drawLine(line.begin, line.end, c::FIELD_LINE_COLOR);
    }
}

void Interface::drawRobots() {
    // draw us
    for (roboteam_msgs::WorldRobot robot : World::get_world().us) {
        drawRobot(robot, c::ROBOT_US_COLOR);
    }

    for (roboteam_msgs::WorldRobot robot : World::get_world().them) {
        drawRobot(robot, c::ROBOT_THEM_COLOR);
    }
}

void Interface::drawBall() {
    drawRect(World::get_world().ball.pos, 10, 10, c::BALL_COLOR);
}

void Interface::drawLine(Vector2 p1, Vector2 p2, SDL_Color color) {

    Vector2 p1_draw = toScreenPosition(p1);
    Vector2 p2_draw = toScreenPosition(p2);

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLine(renderer,
        static_cast<int>(p1_draw.x),
        static_cast<int>(p1_draw.y),
        static_cast<int>(p2_draw.x),
        static_cast<int>(p2_draw.y)
        );
}

void Interface::drawText(std::string text, int x, int y) {
    SDL_Color textColor = c::TEXT_COLOR;
    SDL_Surface* surfaceMessage = TTF_RenderText_Blended(font, text.c_str(), textColor);
    SDL_Texture* message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
    SDL_FreeSurface(surfaceMessage);
    SDL_Rect textRect{x, y, surfaceMessage->w, surfaceMessage->h};
    SDL_RenderCopy(renderer, message, nullptr, &textRect);
    SDL_DestroyTexture(message);
}

// convert field coordinates to screen coordinates
Vector2 Interface::toScreenPosition(Vector2 fieldPos) {
    return {(fieldPos.x * factor.x) + c::WINDOW_SIZE_X/2 + fieldmargin,
    (fieldPos.y * factor.y * -1) + c::WINDOW_SIZE_Y/2 + fieldmargin};
}

void Interface::drawRobot(roboteam_msgs::WorldRobot robot, SDL_Color color) {
    drawRect(robot.pos, c::ROBOT_DRAWING_SIZE, c::ROBOT_DRAWING_SIZE, color);
    drawText(std::to_string(robot.id), toScreenPosition(robot.pos).x, toScreenPosition(robot.pos).y - 20);

    Vector2 angleDestPoint;
    angleDestPoint.x = robot.pos.x + cos(robot.angle);
    angleDestPoint.y = robot.pos.y + sin(robot.angle);
    drawLine(robot.pos, angleDestPoint, color);

}


void Interface::drawRect(Vector2 position, int w, int h, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    Vector2 pos = toScreenPosition(position);
    SDL_Rect rect;
    rect.x = static_cast<int>(pos.x - w/2);
    rect.y = static_cast<int>(pos.y - h/2);
    rect.w = w;
    rect.h = h;

    SDL_RenderFillRect(renderer, &rect);

}

} // interface
} // ai
} // rtt