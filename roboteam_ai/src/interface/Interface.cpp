/*
 * SDL coordinates are different from field coordinates.
 * So, use the draw[whatever] functions to draw stuff.
 * these functions automatically translate the axes for you.
 */

#include "Interface.h"
#include "../utilities/RobotDealer.h"

namespace rtt {
namespace ai {
namespace interface {

namespace c = rtt::ai::constants;

Interface::Interface() : renderer(nullptr) {
    // set up window
    window = SDL_CreateWindow("RTT AI Interface",
            c::WINDOW_POS_X, c::WINDOW_POS_Y, c::WINDOW_SIZE_X+300, c::WINDOW_SIZE_Y, 0);

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
    font = TTF_OpenFont("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",14);
    drawText("No world state has been received yet", 20, 20);
    SDL_RenderPresent(renderer);
}

Interface::~Interface() {
    TTF_CloseFont(font);
    TTF_Quit();
}

void Interface::drawFrame() {
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    fieldmargin = c::WINDOW_FIELD_MARGIN + field.boundary_width;
    factor.x = c::WINDOW_SIZE_X / field.field_length - (2 * fieldmargin);
    factor.y = c::WINDOW_SIZE_Y / field.field_width - (2 * fieldmargin);

    drawField(field);
    drawRobots();
    drawBall();
    drawSideBar();

    // render to screen
    SDL_RenderPresent(renderer);
}

void Interface::drawField(roboteam_msgs::GeometryFieldSize field) {
    SDL_SetRenderDrawColor(renderer, c::FIELD_COLOR.r, c::FIELD_COLOR.g, c::FIELD_COLOR.b, c::FIELD_COLOR.a);
    SDL_RenderClear(renderer);

    // draw field lines
    for (auto line : Field::get_field().field_lines) {
        drawLine(line.begin, line.end, c::FIELD_LINE_COLOR);
    }
}

void Interface::drawRobots() {
    // draw us
    for (roboteam_msgs::WorldRobot robot : World::get_world().us) {
        drawRobot(robot, true);
    }

    for (roboteam_msgs::WorldRobot robot : World::get_world().them) {
        drawRobot(robot, false);
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

void Interface::drawText(std::string text, int x, int y, SDL_Color textColor) {
    if (text != "") {
        SDL_Surface* surfaceMessage = TTF_RenderText_Blended(font, text.c_str(), textColor);
        SDL_Texture* message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
        SDL_FreeSurface(surfaceMessage);
        SDL_Rect textRect{x, y, surfaceMessage->w, surfaceMessage->h};
        SDL_RenderCopy(renderer, message, nullptr, &textRect);
        SDL_DestroyTexture(message);
    }
}

// convert field coordinates to screen coordinates
Vector2 Interface::toScreenPosition(Vector2 fieldPos) {
    return {(fieldPos.x * factor.x) + c::WINDOW_SIZE_X/2 + fieldmargin,
    (fieldPos.y * factor.y * -1) + c::WINDOW_SIZE_Y/2 + fieldmargin};
}

void Interface::drawRobot(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    std::map<std::string, std::set<std::pair<int, std::string>>> list = robotDealer::RobotDealer::getClaimedRobots();
    SDL_Color color = ourTeam ? c::ROBOT_US_COLOR : c::ROBOT_THEM_COLOR;

    if (ourTeam) {
        std::string roleName = "";
        for (auto &robotowner : list) {
            std::string tactic = robotowner.first;
            std::set<std::pair<int, std::string>> robots = robotowner.second;
            for (auto &ownedRobot : robots) {
                if (ownedRobot.first == robot.id) {
                    roleName = ownedRobot.second;

                    bool tacticExists = false;
                    SDL_Color c;
                    for (auto tac : tacticColors) {
                        if (tac.first == tactic) {
                            c = tac.second;
                            tacticExists = true;
                            break;
                        }
                    }

                    if (!tacticExists) {
                        SDL_Color newColor = c::TACTIC_COLORS[tacticCount];
                        tacticCount = (tacticCount + 1) % sizeof(c::TACTIC_COLORS);
                        tacticColors.push_back({tactic, newColor});
                        c = newColor;
                    }

                    drawRect(robot.pos, c::ROBOT_DRAWING_SIZE+4, c::ROBOT_DRAWING_SIZE+4, c);
                    drawText(tactic, toScreenPosition(robot.pos).x, toScreenPosition(robot.pos).y - 40, color);
                    drawText(roleName, toScreenPosition(robot.pos).x, toScreenPosition(robot.pos).y - 60, color);
                }
            }
        }
    }

    drawRect(robot.pos, c::ROBOT_DRAWING_SIZE, c::ROBOT_DRAWING_SIZE, color);
    drawText(std::to_string(robot.id), toScreenPosition(robot.pos).x, toScreenPosition(robot.pos).y - 20);

    Vector2 velocityDestPoint;
    velocityDestPoint.x = robot.pos.x + robot.vel.x;
    velocityDestPoint.y = robot.pos.y + robot.vel.y;
    drawLine(robot.pos, velocityDestPoint, c::TEXT_COLOR);

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

void Interface::handleMouseClick(SDL_Event event) {
    for (roboteam_msgs::WorldRobot robot : World::get_world().us) {

        Vector2 robotPos = toScreenPosition(robot.pos);
        if ((event.button.x > (robotPos.x - c::ROBOT_DRAWING_SIZE/2))
            && (event.button.x < (robotPos.x + c::ROBOT_DRAWING_SIZE/2))
            && (event.button.y < (robotPos.y + c::ROBOT_DRAWING_SIZE/2))
            && (event.button.y < (robotPos.y + c::ROBOT_DRAWING_SIZE/2))) {
            currentlySelectedRobot = std::make_shared<roboteam_msgs::WorldRobot>(robot);
        }
    }
}

void Interface::drawSideBar() {
    if (currentlySelectedRobot) {
        drawText(std::to_string(currentlySelectedRobot->id), c::WINDOW_SIZE_X + 20, 30);
    }
}

} // interface
} // ai
} // rtt