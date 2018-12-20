//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::GoToPosLuThPoints;
std::mutex Drawer::mutex;

void Drawer::setGoToPosLuThPoints(int id, std::vector<std::pair<rtt::Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(mutex);

    std::pair<int, std::vector<std::pair<rtt::Vector2, QColor>>> pair{id, std::move(points)};

    GoToPosLuThPoints.erase(id);
    GoToPosLuThPoints.insert(pair);
}

std::vector<std::pair<Vector2, QColor>> Drawer::getGoToPosLuThPoints(int id) {
    std::lock_guard<std::mutex> lock(mutex);

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints.at(id);
    }
    return {};

}

} // interface
} // ai
} // rtt