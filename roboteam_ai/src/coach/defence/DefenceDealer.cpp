//
// Created by rolf on 3-4-19.
//

#include "DefenceDealer.h"
#include "roboteam_ai/src/interface/drawer.h"
namespace rtt{
namespace ai{
namespace coach{
DefenceDealer g_DefenceDealer;
void DefenceDealer::setDoUpdate() {
    doUpdate = true;
}
/// adds a defender to the available defendersList
void DefenceDealer::addDefender(int id) {
    bool robotIsRegistered = std::find(defenders.begin(), defenders.end(), id) != defenders.end();
    if (! robotIsRegistered) {
        defenders.push_back(id);
        setDoUpdate();
        //std::cout << "registered defender id:" << id << std::endl;
    }
    else {
        std::cerr << "Defender is already registered, check your tree!!" << std::endl;
    }
}

/// removes a defender from the available id's
void DefenceDealer::removeDefender(int id) {
    auto defender = std::find(defenders.begin(), defenders.end(), id);
    if (defender != defenders.end()) {
        defenders.erase(defender);
        setDoUpdate();
//        std::cout << "removed defender id:" << id << std::endl;
    }
    else {
        std::cerr << "Defender cannot be removed as it is not registered! Check your skill!" << std::endl;
    }
}

/// get the specific position of a defender with specified id
std::shared_ptr<std::pair<Vector2, double>> DefenceDealer::getDefenderPosition(int id) {
    auto element = defenderLocations.find(id);
    if (element == defenderLocations.end()) {
        return nullptr;
    }
    else return std::make_shared<std::pair<Vector2, double>>(defenderLocations[id]);
}
void DefenceDealer::visualizePoints() {
    int i = 0;
    std::vector<std::pair<Vector2, QColor>> vis2;
    for (auto location : defenderLocations) {
        std::pair<Vector2, QColor> pair;
        int colourcount = 6;
        if (i%colourcount == 0) {
            pair = std::make_pair(location.second.first, Qt::green);
        }
        else if (i%colourcount == 1) {
            pair = std::make_pair(location.second.first, Qt::red);
        }
        else if (i%colourcount == 2) {
            pair = std::make_pair(location.second.first, Qt::blue);
        }
        else if (i%colourcount == 3) {
            pair = std::make_pair(location.second.first, Qt::darkYellow);
        }
        else if (i%colourcount == 4) {
            pair = std::make_pair(location.second.first, Qt::darkMagenta);
        }
        else if (i%colourcount == 5) {
            pair = std::make_pair(location.second.first, Qt::cyan);
        }
        vis2.emplace_back(pair);
        i ++;
    }
    ai::interface::Drawer::setTestPoints(vis2);
}
/// calculates the defender locations for all available defenders
void DefenceDealer::updateDefenderLocations() {
    if (doUpdate) {
        doUpdate = false;
        auto start = std::chrono::high_resolution_clock::now();
        // clear the defenderLocations
        defenderLocations.clear();
        std::vector<int> availableDefenders = defenders;
        // decide the locations to defend
        std::vector<DefencePositionCoach::DefenderBot> botposses = g_defensivePositionCoach.decidePositions(availableDefenders.size());
        // the following algorithm takes the closest robot for each available defender to decide which robot goes where.
        // Since the points are ordered on priority from the above algorithm the most important points come first
        // It might be better to use an algorithm that is more complicated (e.g. hungarian) but then we might need some kind of system which gives the first points more 'priority'
        for (auto botpos : botposses) {
            int bestId = - 1;
            double bestDist = 10000000000;
            for (int botId : availableDefenders) {
                auto bot = world::world->getRobotForId(botId, true);
                if (bot) {
                    if ((botpos.targetPos - bot->pos).length() < bestDist) {
                        bestId = botId;
                        bestDist = (botpos.targetPos - bot->pos).length();
                    }
                }
                else {
                    std::cerr << "Could not find robot " << botId << " to defend!";
                }
            }
            if (bestId != - 1) {
                defenderLocations[bestId] = {botpos.targetPos,botpos.orientation};
                availableDefenders.erase(std::find(availableDefenders.begin(), availableDefenders.end(), bestId));
            }
            else {
                std::cerr << "Could not find a robot to defend location!!!";
                return;
            }
        }
        //visualization
        visualizePoints();
        auto stop = std::chrono::high_resolution_clock::now();
        std::cout << "Computation time:" << (std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count()/1000000.0) << std::endl;
    }
}

}//coach
}//ai
}//rtt
