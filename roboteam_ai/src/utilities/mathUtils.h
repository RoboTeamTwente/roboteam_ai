//
// Created by rolf on 28/11/18.
//

#ifndef ROBOTEAM_AI_MATHUTILS_H
#define ROBOTEAM_AI_MATHUTILS_H

#include "roboteam_utils/Vector2.h"
namespace rtt{
namespace ai{
double TriangleArea(Vector2 A,Vector2 B,Vector2 C)
bool pointInTriangle(Vector2 PointToCheck,Vector2 TP1, Vector2 TP2, Vector2 TP3);
bool pointInRectangle(Vector2 PointToCheck,Vector2 SP1, Vector2 SP2, Vector2 SP3,Vector2 SP4);
}
}

#endif //ROBOTEAM_AI_MATHUTILS_H
