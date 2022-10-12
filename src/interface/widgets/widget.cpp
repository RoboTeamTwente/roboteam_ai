//
// Created by mrlukasbos on 27-11-18.
//

#include "interface/widgets/widget.h"

#include <proto/SimulationConfiguration.pb.h>

#include <QPainterPath>

#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"

namespace io = rtt::ai::io;
namespace rtt::ai::interface {

Visualizer::Visualizer(QWidget *parent) : QWidget(parent) {}

/// The update loop of the field widget. Invoked by widget->update();
void Visualizer::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::HighQualityAntialiasing, true);

    {
        // worldPtr is scoped in order to release the lock soon as possible
        auto const &[_, worldPtr] = rtt::world::World::instance();
        auto const &world = worldPtr->getWorld();
        auto const &field = worldPtr->getField();

        if (!world.has_value()) {
            painter.drawText(24, 24, "Waiting for incoming world state");
            return;
        }

        if (field.has_value()) {
            calculateFieldSizeFactor(field.value());
            drawBackground(painter);
            drawFieldHints(field.value(), painter);
            drawFieldLines(field.value(), painter);
        }

        auto s = QString::fromStdString("We have " + std::to_string(world->getUs().size()) + " robots");
        painter.drawText(24, 48, s.fromStdString("We have " + std::to_string(world->getUs().size()) + " robots"));

        if (showWorld) {
            drawRobots(painter, world.value());
            if (world->getBall().has_value()) drawBall(painter, world->getBall().value());
        }
    }

    // draw the drawings from the input
    auto drawings = Input::getDrawings();
    for (auto const &drawing : drawings) {
        if (!drawing.points.empty()) {
            bool shouldShow = false;
            for (auto const &toggle : Toggles::toggles) {
                if (drawing.visual == toggle.visual) {
                    shouldShow = shouldVisualize(toggle, drawing.robotId);
                }
            }
            if (shouldShow) {
                switch (drawing.method) {
                    case Drawing::DOTS: {
                        painter.setPen(Qt::NoPen);
                        painter.setBrush(drawing.color);
                        drawPoints(painter, drawing.points, drawing.width, drawing.height);
                    } break;
                    case Drawing::CIRCLES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawPoints(painter, drawing.points, drawing.width, drawing.height);
                    } break;
                    case Drawing::LINES_CONNECTED: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawLines(painter, drawing.points);
                    } break;
                    case Drawing::CROSSES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawCrosses(painter, drawing.points, drawing.width, drawing.height);
                    } break;
                    case Drawing::PLUSSES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawPlusses(painter, drawing.points, drawing.width, drawing.height);
                    } break;
                    case Drawing::ARROWS: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawArrows(painter, drawing.points, drawing.width, drawing.height, drawing.strokeWidth == 1);
                    } break;
                    case Drawing::REAL_LIFE_CIRCLES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawRealLifeSizedPoints(painter, drawing.points, drawing.width, drawing.height);
                    } break;
                    case Drawing::REAL_LIFE_DOTS: {
                        painter.setPen(Qt::NoPen);
                        painter.setBrush(drawing.color);
                        drawRealLifeSizedPoints(painter, drawing.points, drawing.width, drawing.height);
                    }
                }
            }
        }
    }

    Input::clearDrawings();

    if (showBallPlacementMarker) drawBallPlacementTarget(painter);

    /* Ball dragging using middle mouse button. Hold it to drag the ball */
    if (middle_mouse_pressed and SETTINGS.getRobotHubMode() == Settings::RobotHubMode::SIMULATOR) {
        QPoint qt_mouse_position = mapFromGlobal(QCursor::pos());              // Get mouse position on the widget
        Vector2 mouse_position(qt_mouse_position.x(), qt_mouse_position.y());  // Convert Qt to Vector2
        Vector2 field_position = toFieldPosition(mouse_position);              // Convert position on widget to position on field
        if (!SETTINGS.isLeft()) field_position *= -1;                          // Invert ball position if we play on the other side of the field

        proto::SimulationConfiguration configuration;                    // Create packet
        configuration.mutable_ball_location()->set_x(field_position.x);  // Set x
        configuration.mutable_ball_location()->set_y(field_position.y);  // Set y

        bool sentConfig = io::io.sendSimulationConfiguration(configuration);  // Send packet
        if (!sentConfig) {
            RTT_WARNING("Failed to send Simulation Configuration command. Is this the primary AI?")
        }
    }

    if (showWorldDetections) {
        drawRawDetectionPackets(painter);
    }
}

bool Visualizer::shouldVisualize(Toggle toggle, int robotId) {
    switch (toggle.showType) {
        default:
            return false;
        case GENERAL: {
            switch (toggle.generalShowType) {
                default:
                    return false;
                case OFF:
                    return false;
                case ON:
                    return true;
            }
            break;
        }
        case ROBOT: {
            switch (toggle.robotShowType) {
                default:
                    return false;
                case NO_ROBOTS:
                    return false;
                case SELECTED_ROBOTS:
                    return robotIsSelected(robotId);
                case ALL_ROBOTS:
                    return true;
            }
        }
    }
}

/// Calculates the factor variable which is used for mapping field coordinates with screen coordinates.
void Visualizer::calculateFieldSizeFactor(const rtt::world::Field &field) {
    fieldmargin = static_cast<int>(Constants::WINDOW_FIELD_MARGIN() + field.getBoundaryWidth());

    float widthFactor = this->size().width() / field.getFieldLength() - (2 * fieldmargin);
    float heightFactor = this->size().height() / field.getFieldWidth() - (2 * fieldmargin);
    factor = std::min(widthFactor, heightFactor);
}

/// draws background of the field
void Visualizer::drawBackground(QPainter &painter) {
    painter.setBrush(Constants::FIELD_COLOR());
    painter.drawRect(-10, -10, this->size().width() + 10, this->size().height() + 10);
}

// draws the field lines
void Visualizer::drawFieldLines(const rtt::world::Field &field, QPainter &painter) {
    painter.setPen(Constants::FIELD_LINE_COLOR());
    painter.setBrush(Qt::transparent);
    // draw lines
    for (const auto &fieldLine : field.getFieldLines()) {
        rtt::Vector2 start = toScreenPosition(fieldLine.begin);
        rtt::Vector2 end = toScreenPosition(fieldLine.end);
        painter.drawLine(start.x, start.y, end.x, end.y);
    }

    // draw the circle in the middle
    auto centercircle = field.getCenterCircle();
    Vector2 screenPos = toScreenPosition({centercircle.center.x, centercircle.center.y});
    painter.drawEllipse(QPointF(screenPos.x, screenPos.y), centercircle.radius * factor, centercircle.radius * factor);

    painter.setPen(Qt::red);
    auto line = field.getLeftPenaltyLine();
    rtt::Vector2 start = toScreenPosition(line.begin);
    rtt::Vector2 end = toScreenPosition(line.end);
    painter.drawLine(start.x, start.y, end.x, end.y);

    painter.setPen(Qt::green);
    line = field.getRightPenaltyLine();
    start = toScreenPosition(line.begin);
    end = toScreenPosition(line.end);
    painter.drawLine(start.x, start.y, end.x, end.y);

    painter.setPen(Qt::green);
    line = field.getRightLine();
    start = toScreenPosition(line.begin);
    end = toScreenPosition(line.end);
    painter.drawLine(start.x, start.y, end.x, end.y);

    painter.setPen(Qt::red);
    line = field.getLeftLine();
    start = toScreenPosition(line.begin);
    end = toScreenPosition(line.end);
    painter.drawLine(start.x, start.y, end.x, end.y);

    QPen pen;
    pen.setWidth(3);

    // update the we are yellow
    bool weAreYellow = SETTINGS.isYellow();

    // draw the hint for us
    LineSegment usGoalLine = FieldComputations::getGoalSides(field, true);
    Vector2 ourLineUpper = {usGoalLine.start.x, usGoalLine.start.y};
    Vector2 ourLineLower = {usGoalLine.end.x, usGoalLine.end.y};
    ourLineUpper = toScreenPosition(ourLineUpper);
    ourLineLower = toScreenPosition(ourLineLower);

    auto color = weAreYellow ? QColor(255, 255, 0, 255) : QColor(80, 80, 255, 255);
    pen.setBrush(color);
    pen.setColor(color);
    painter.setPen(pen);
    painter.drawLine(ourLineUpper.x, ourLineUpper.y, ourLineLower.x, ourLineLower.y);

    LineSegment theirGoalLine = FieldComputations::getGoalSides(field, false);
    Vector2 theirLineUpper = {theirGoalLine.start.x, theirGoalLine.start.y};
    Vector2 theirLineLower = {theirGoalLine.end.x, theirGoalLine.end.y};
    theirLineUpper = toScreenPosition(theirLineUpper);
    theirLineLower = toScreenPosition(theirLineLower);

    color = weAreYellow ? QColor(80, 80, 255, 255) : QColor(255, 255, 0, 255);
    pen.setBrush(color);
    pen.setColor(color);
    painter.setPen(pen);
    painter.drawLine(theirLineUpper.x, theirLineUpper.y, theirLineLower.x, theirLineLower.y);
}

void Visualizer::drawFieldHints(const rtt::world::Field &field, QPainter &painter) {
    QPen pen;

    // draw the position where robots would be for timeout
    int inv = rtt::ai::interface::Output::isTimeOutAtTop() ? 1 : -1;
    int lineY = (field.getFieldWidth() / 2 + 1) * inv;

    pen.setBrush(Qt::gray);
    pen.setColor(Qt::gray);
    painter.setPen(pen);

    auto lineStart = toScreenPosition(Vector2(field.getLeftmostX(), lineY));
    auto lineEnd = toScreenPosition(Vector2(0, lineY));

    painter.drawLine(lineStart.x, lineStart.y, lineEnd.x, lineEnd.y);
}

// draw the ball on the screen
void Visualizer::drawBall(QPainter &painter, rtt::world::view::BallView ball) {
    rtt::Vector2 ballPosition = toScreenPosition(ball->position);
    QPointF qballPosition(ballPosition.x, ballPosition.y);

    painter.setBrush(ball->visible ? Constants::BALL_COLOR() : Qt::red);

    // draw a see-through gradient around the ball to make it more visible
    painter.setPen(Qt::NoPen);  // stroke
    painter.setOpacity(0.5);
    painter.drawEllipse(qballPosition, Constants::BALL_DRAWING_SIZE(), Constants::BALL_DRAWING_SIZE());
    painter.setOpacity(1);
    int ballSize = Constants::BALL_RADIUS() * 2 * factor;
    painter.drawEllipse(qballPosition, ballSize, ballSize);
}

// draw the robots
void Visualizer::drawRobots(QPainter &painter, rtt::world::view::WorldDataView world) {
    // draw us
    for (auto const &robot : world->getUs()) {
        std::string role{};
        if (robot) {
            if (rolesForRobots.find(robot->getId()) != rolesForRobots.end()) {
                std::lock_guard mtx{rolesUpdate};
                role = rolesForRobots[robot->getId()];
            }
            drawRobot(painter, robot, true, role);
        }
    }

    // draw them
    for (auto const &robot : world->getThem()) {
        if (robot) {
            drawRobot(painter, robot, false);
        }
    }
}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toScreenPosition(rtt::Vector2 fieldPos) {
    int inv = fieldInversed ? -1 : 1;
    auto x = (fieldPos.x * factor * inv) + static_cast<float>(this->size().width() / 2 + fieldmargin);
    auto y = (fieldPos.y * factor * -1 * inv) + static_cast<float>(this->size().height() / 2 + fieldmargin);
    return Vector2(x, y);
}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toFieldPosition(rtt::Vector2 screenPos) {
    int inv = fieldInversed ? -1 : 1;
    auto x = (screenPos.x - fieldmargin - static_cast<float>(this->size().width() / 2)) / factor;
    auto y = ((screenPos.y - fieldmargin - static_cast<float>(this->size().height() / 2)) / factor) * -1;
    return Vector2(x, y) * inv;
}

// draw a single robot
void Visualizer::drawRobot(QPainter &painter, rtt::world::view::RobotView robot, bool ourTeam, std::string role) {
    Vector2 robotpos = toScreenPosition(robot->getPos());

    // update the we are yellow
    bool weAreYellow = SETTINGS.isYellow();

    QColor robotColor;
    if (ourTeam) {
        // our robots have our_color
        robotColor = weAreYellow ? Constants::ROBOT_COLOR_YELLOW() : Constants::ROBOT_COLOR_BLUE();
    } else {
        // the enemy robot should have the inverse of our_color
        robotColor = weAreYellow ? Constants::ROBOT_COLOR_BLUE() : Constants::ROBOT_COLOR_YELLOW();
    }

    if (ourTeam && robot->getId() == GameStateManager::getCurrentGameState().keeperId) {
        robotColor = QColor(255, 255, 255);
    }

    if (showAngles) {
        Vector2 angle = toScreenPosition({robot->getPos().x + cos(robot->getAngle()) / 3, robot->getPos().y + sin(robot->getAngle()) / 3});
        QPen pen;
        pen.setWidth(2);
        pen.setBrush(robotColor);
        painter.setPen(pen);
        painter.drawLine(robotpos.x, robotpos.y, angle.x, angle.y);
    }

    if (showVelocities) {
        Vector2 vel = toScreenPosition({robot->getPos().x + robot->getVel().x, robot->getPos().y + robot->getVel().y});
        painter.setPen(Qt::white);
        painter.drawLine(robotpos.x, robotpos.y, vel.x, vel.y);
    }

    if (showTacticColors && ourTeam) {
        drawTacticColorForRobot(painter, robot);
    }
    int ypos = robotpos.y;
    if (showTactics && ourTeam) {
        painter.setPen(Constants::TEXT_COLOR());

        painter.drawText(robotpos.x, ypos += 20, QString::fromStdString(getTacticNameForRobot(robot)));
    }

    if (showRoles && ourTeam) {
        painter.setPen(Constants::TEXT_COLOR());
        painter.drawText(robotpos.x, ypos += 20, QString::fromStdString(getRoleNameForRobot(robot)));
    }

    // Todo : Get working stuff in RobotView
    if (showRobotInvalids && ourTeam) {
        painter.setPen(Qt::red);
        QString text;
        if (!robot->isWorkingDribbler()) {
            text += "DR ";
        }
        if (!robot->isWorkingBallSensor()) {
            text += "BS ";
        }
        if (robot->isBatteryLow()) {
            text += "BATTERY LOW";
        }
        painter.drawText(robotpos.x, ypos += 20, text);
        painter.drawText(robotpos.x, ypos + 10, QString::fromStdString(role));
    }

    // Todo : Get working feedback in RobotView
    //    if (ourTeam) {
    //        if (Constants::FEEDBACK_ENABLED()) {
    //            if (robot.hasRecentFeedback()) {
    //                // green to indicate feedback is okay
    //                painter.setPen(Qt::green);
    //                painter.setBrush(Qt::green);
    //            } else {
    //                // yellow to indicate feedback is not okay
    //                painter.setPen(Qt::red);
    //                painter.setBrush(Qt::red);
    //            }
    //            painter.drawEllipse({(int)robotpos.x + 10, (int)robotpos.y - 10}, 2, 2);
    //        }
    //    }

    // draw the robots
    QColor color = (robotIsSelected(robot) && ourTeam) ? Constants::SELECTED_ROBOT_COLOR() : robotColor;
    painter.setBrush(color);
    painter.setPen(Qt::transparent);
    painter.setOpacity(1);

    int robotDrawSize = std::max(Constants::ROBOT_RADIUS() * factor * 2, (double)Constants::ROBOT_DRAWING_SIZE());

    // draw the shape of the robot with the right angle
    QPainterPath rectPath;
    rectPath.moveTo(0, -robotDrawSize / 2.0);
    rectPath.arcTo(-robotDrawSize / 2.0, -robotDrawSize / 2.0, robotDrawSize, robotDrawSize, 90, 270);
    rectPath.closeSubpath();

    painter.translate(robotpos.x, robotpos.y);  // move center of coordinates to the center of robot

    if (fieldInversed) {
        painter.rotate(-toDegrees(robot->getAngle()) + 45 + 180);  // rotate around the center of robot
    } else {
        painter.rotate(-toDegrees(robot->getAngle()) + 45);  // rotate around the center of robot
    }
    painter.drawPath(rectPath);
    painter.resetTransform();  // reset the translation and rotation

    // draw the id in it
    painter.setPen(Qt::black);
    painter.setFont(QFont("ubuntu", 9));  // 22 is a number which you have to change
    painter.drawText(robotpos.x - 3, robotpos.y + 5, QString::number(robot->getId()));
    painter.setFont(QFont("ubuntu", 11));  // 22 is a number which you have to change
}

// Handle mousePressEvents
void Visualizer::mousePressEvent(QMouseEvent *event) {
    Vector2 click_position;
    click_position.x = event->pos().x();
    click_position.y = event->pos().y();

    std::optional<rtt::world::view::WorldDataView> world;
    {
        auto const &[_, worldPtr] = rtt::world::World::instance();
        world = worldPtr->getWorld();
    }

    if (event->button() == Qt::LeftButton && world.has_value()) {
        for (auto &robot : world->getUs()) {
            if (click_position.dist(toScreenPosition(robot->getPos())) < 10) {
                this->toggleSelectedRobot(robot);
            }
        }
    } else if (event->button() == Qt::RightButton) {
        Output::setMarkerPosition(toFieldPosition(click_position));
    } else if (event->button() == Qt::MiddleButton && world.has_value()) {
        middle_mouse_pressed = true;
    }
}

void Visualizer::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::MiddleButton) middle_mouse_pressed = false;
}

void Visualizer::drawTacticColorForRobot(QPainter &painter, rtt::world::view::RobotView robot) {
    Vector2 robotpos = toScreenPosition(robot->getPos());
    QPointF qrobotPosition(robotpos.x, robotpos.y);
    std::string tacticName = getTacticNameForRobot(robot);
    bool tacticExists = false;
    QColor c;
    for (auto tac : tacticColors) {
        if (tac.first == tacticName) {
            c = tac.second;
            tacticExists = true;
            break;
        }
    }

    if (!tacticExists) {
        QColor newColor = Constants::TACTIC_COLORS().at(tacticCount);
        tacticCount = (tacticCount + 1) % Constants::TACTIC_COLORS().size();
        tacticColors.push_back({tacticName, newColor});
        c = newColor;
    }

    painter.setPen(Qt::transparent);
    painter.setBrush(c);
    painter.drawEllipse(qrobotPosition, Constants::TACTIC_COLOR_DRAWING_SIZE(), Constants::TACTIC_COLOR_DRAWING_SIZE());
}

std::string Visualizer::getTacticNameForRobot(rtt::world::view::RobotView robot) { return tacticsForRobots[robot->getId()]; }

std::string Visualizer::getRoleNameForRobot(rtt::world::view::RobotView robot) { return this->rolesForRobots[robot->getId()]; }

const std::unordered_map<int, rtt::world::view::RobotView> &Visualizer::getSelectedRobots() const { return selectedRobots; }

void Visualizer::toggleSelectedRobot(rtt::world::view::RobotView robot) {
    bool robotSelected = (selectedRobots.find(robot->getId()) != selectedRobots.end());

    if (robotSelected) {
        selectedRobots.erase(robot->getId());
    } else {
        selectedRobots.insert({robot->getId(), robot});
    }
}

bool Visualizer::robotIsSelected(rtt::world::view::RobotView robot) { return (selectedRobots.find(robot->getId()) != selectedRobots.end()); }

bool Visualizer::robotIsSelected(int robotId) { return (selectedRobots.find(robotId) != selectedRobots.end()); }

void Visualizer::drawBallPlacementTarget(QPainter &painter) {
    Vector2 marker = toScreenPosition(Output::getInterfaceMarkerPosition());
    painter.setBrush(Qt::transparent);
    painter.setPen(Qt::red);

    painter.drawLine(marker.x - 5, marker.y - 5, marker.x + 5, marker.y + 5);
    painter.drawLine(marker.x + 5, marker.y - 5, marker.x - 5, marker.y + 5);

    if (Output::usesRefereeCommands()) {
        Vector2 ballPlacementTarget = toScreenPosition(Vector2(GameStateManager::getRefereeDesignatedPosition()));
        painter.setBrush(Qt::transparent);
        painter.setPen(Qt::green);

        painter.drawLine(ballPlacementTarget.x - 5, ballPlacementTarget.y - 5, ballPlacementTarget.x + 5, ballPlacementTarget.y + 5);
        painter.drawLine(ballPlacementTarget.x + 5, ballPlacementTarget.y - 5, ballPlacementTarget.x - 5, ballPlacementTarget.y + 5);
    }
}

void Visualizer::drawPlusses(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);

        // draw a plus
        painter.drawLine(0, pointOnScreen.y - height / 2, 0, pointOnScreen.y + height / 2);
        painter.drawLine(pointOnScreen.x + width / 2, 0, pointOnScreen.x - width / 2, 0);
    }
}

void Visualizer::drawArrows(QPainter &painter, std::vector<Vector2> points, double factor, double maxSize, bool closedArrow) {
    if (points.size() >= 2) {
        for (size_t i = 1; i < points.size(); i += 2) {
            Vector2 &arrowEnd = points.at(i - 1);
            Vector2 &arrowStart = points.at(i);

            double arrowLength = (arrowEnd - arrowStart).length();
            Angle arrowAngle = (arrowEnd - arrowStart).toAngle();

            double arrowSizeFactor = factor == 4.0 ? 0.2 : std::min(1.0, factor);
            double maxArrowSize = maxSize == 4.0 ? 0.2 : std::min(1.0, maxSize);
            double arrowSize = arrowLength > maxArrowSize / arrowSizeFactor ? arrowSizeFactor : arrowSizeFactor * arrowLength;

            Vector2 startPoint = arrowEnd + (arrowStart - arrowEnd).stretchToLength(arrowSize);
            Vector2 pointyBitLeft = startPoint + (arrowAngle + Angle(M_PI_2)).toVector2(arrowSize);
            Vector2 pointyBitRight = startPoint + (arrowAngle + Angle(M_PI_2)).toVector2(-arrowSize);

            Vector2 arrowStartOnScreen = toScreenPosition(arrowStart);
            Vector2 arrowEndOnScreen = toScreenPosition(arrowEnd);
            Vector2 pointyBitLeftOnScreen = toScreenPosition(pointyBitLeft);
            Vector2 pointyBitRightOnScreen = toScreenPosition(pointyBitRight);
            Vector2 startPointOnScreen = toScreenPosition(startPoint);
            if (closedArrow) {
                painter.drawLine(arrowStartOnScreen.x, arrowStartOnScreen.y, startPointOnScreen.x, startPointOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitRightOnScreen.x, pointyBitRightOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitLeftOnScreen.x, pointyBitLeftOnScreen.y);
                painter.drawLine(pointyBitRightOnScreen.x, pointyBitRightOnScreen.y, pointyBitLeftOnScreen.x, pointyBitLeftOnScreen.y);
            } else {
                painter.drawLine(arrowStartOnScreen.x, arrowStartOnScreen.y, arrowEndOnScreen.x, arrowEndOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitRightOnScreen.x, pointyBitRightOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitLeftOnScreen.x, pointyBitLeftOnScreen.y);
            }
        }
    }
}

void Visualizer::drawCrosses(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);
        painter.drawLine(pointOnScreen.x - width / 2, pointOnScreen.y - height / 2, pointOnScreen.x + width / 2, pointOnScreen.y + height / 2);
        painter.drawLine(pointOnScreen.x + width / 2, pointOnScreen.y - height / 2, pointOnScreen.x - width / 2, pointOnScreen.y + height / 2);
    }
}

void Visualizer::drawPoints(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);
        painter.drawEllipse(pointOnScreen.x - width / 2, pointOnScreen.y - height / 2, width, height);
    }
}

void Visualizer::drawLines(QPainter &painter, std::vector<Vector2> points) {
    if (points.size() >= 2) {
        for (size_t i = 1; i < points.size(); i++) {
            Vector2 pointOnScreen = toScreenPosition(points.at(i));
            Vector2 prevPointOnScreen = toScreenPosition(points.at(i - 1));
            painter.drawLine(pointOnScreen.x, pointOnScreen.y, prevPointOnScreen.x, prevPointOnScreen.y);
        }
    }
}

void Visualizer::drawRealLifeSizedPoints(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    width = width * 2.0 * factor;
    height = height * 2.0 * factor;
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);
        painter.drawEllipse(pointOnScreen.x - width / 2, pointOnScreen.y - height / 2, width, height);
    }
}

void Visualizer::setPlayForRobot(std::string const &view, uint8_t i) {
    std::lock_guard mtx{rolesUpdate};
    rolesForRobots.insert({i, view});
}

void Visualizer::updateProcessedVisionPackets(const std::vector<proto::SSL_WrapperPacket> &packets) {
    std::lock_guard mtx{worldDetectionsMutex};
    raw_detection_packets = packets;
}
void Visualizer::drawRawDetectionPackets(QPainter &painter) {
    std::lock_guard mtx{worldDetectionsMutex};
    for (const auto &packet : raw_detection_packets) {
        if (packet.has_detection()) {
            for (const auto &ball : packet.detection().balls()) {
                drawDetectionBall(painter, ball);
            }
            for (const auto &robot : packet.detection().robots_blue()) {
                drawDetectionRobot(painter, true, robot);
            }
            for (const auto &robot : packet.detection().robots_yellow()) {
                drawDetectionRobot(painter, false, robot);
            }
        }
    }
}
void Visualizer::drawDetectionBall(QPainter &painter, const proto::SSL_DetectionBall &ball) {
    Vector2 ballWorldPos(ball.x() * 0.001, ball.y() * 0.001);  // ssl units are in millimeters
    rtt::Vector2 ballPosition = toScreenPosition(ballWorldPos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);

    painter.setBrush(Qt::cyan);

    painter.setPen(Qt::NoPen);  // stroke
    painter.setOpacity(0.8);
    int ballSize = Constants::BALL_RADIUS() * 2 * factor;

    painter.drawEllipse(qballPosition, ballSize, ballSize);
}
void Visualizer::drawDetectionRobot(QPainter &painter, bool robotIsBlue, const proto::SSL_DetectionRobot &robot) {
    Vector2 robotWorldPos = Vector2(robot.x() * 0.001, robot.y() * 0.001);
    Vector2 robotpos = toScreenPosition(robotWorldPos);

    // update the we are yellow
    bool weAreYellow = SETTINGS.isYellow();

    QColor robotColor = robotIsBlue ? Qt::green : Qt::red;  // use contrasting colors

    int ypos = robotpos.y;

    // draw the robots
    painter.setBrush(robotColor);
    painter.setPen(Qt::transparent);
    painter.setOpacity(0.5);

    int robotDrawSize = std::max(Constants::ROBOT_RADIUS() * factor * 2, (double)Constants::ROBOT_DRAWING_SIZE());

    // draw the shape of the robot with the right angle
    QPainterPath rectPath;
    rectPath.moveTo(0, -robotDrawSize / 2.0);
    rectPath.arcTo(-robotDrawSize / 2.0, -robotDrawSize / 2.0, robotDrawSize, robotDrawSize, 90, 270);
    rectPath.closeSubpath();

    painter.translate(robotpos.x, robotpos.y);  // move center of coordinates to the center of robot

    if (fieldInversed) {
        painter.rotate(-toDegrees(robot.orientation()) + 45 + 180);  // rotate around the center of robot
    } else {
        painter.rotate(-toDegrees(robot.orientation()) + 45);  // rotate around the center of robot
    }
    painter.drawPath(rectPath);
    painter.resetTransform();  // reset the translation and rotation

    // draw the id in it
    painter.setPen(Qt::black);
    painter.setFont(QFont("ubuntu", 9));  // 22 is a number which you have to change
    painter.drawText(robotpos.x - 3, robotpos.y + 5, QString::number(robot.robot_id()));
    painter.setFont(QFont("ubuntu", 11));  // 22 is a number which you have to change
}

}  // namespace rtt::ai::interface
