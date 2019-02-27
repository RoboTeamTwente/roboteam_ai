//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_DRAWER_H
#define ROBOTEAM_AI_DRAWER_H

#include <QtGui/QColor>
#include <roboteam_utils/Vector2.h>
#include <iostream>
#include <mutex>

namespace rtt {
namespace ai {
namespace interface {

class Drawer {
    public:
        explicit Drawer() = default;
        static void setGoToPosLuThPoints(int id, std::vector<std::pair<Vector2, QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getGoToPosLuThPoints(int id);
        static void setKeeperPoints(int id, std::vector<std::pair<Vector2,QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getKeeperPoints(int id);
        static void setInterceptPoints(int id, std::vector<std::pair<Vector2,QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getInterceptPoints(int id);
        static void setTestLines(std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> lines);
        static std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>>  getTestLines();
        static void setTestPoints(std::vector<std::pair<Vector2,QColor>> points);
        static std::vector<std::pair<Vector2,QColor>>  getTestPoints();
    private:
        static std::mutex goToPosMutex,keeperMutex,interceptMutex,testLineMutex,testPointMutex;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> GoToPosLuThPoints;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> KeeperPoints;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> InterceptPoints;
        static std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> testLines;
        static std::vector<std::pair<Vector2,QColor>> testPoints;

};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
