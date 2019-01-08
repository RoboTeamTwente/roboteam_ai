//
// Created by mrlukasbos on 8-1-19.
//

#ifndef ROBOTEAM_AI_VECTOR2_H
#define ROBOTEAM_AI_VECTOR2_H

#include <iostream>

#include "roboteam_msgs/Vector2f.h"

namespace rtt
{

/**
 * \class Vector2
 * \brief A vector of two doubles.
 * This class is not exactly immutable - the x and y fields are publicly accessible -
 * but none of its methods will modify it, they will instead return a new vector when needed.
 * The only exception is operator=(const roboteam_msgs::Vector2f&).
 */
class Vector2
{
public:
    /**
     * \brief The zero vector.
     */
    constexpr Vector2() : x(0), y(0) {}

    /**
     * \brief Construct a vector from two doubles.
     */
    constexpr Vector2(double x, double y) : x(x), y(y) {}

    /**
     * \brief Copy constructor.
     */
    constexpr Vector2(const Vector2& copy) : x(copy.x), y(copy.y) {}

    /**
     * \brief Convert a ROS Vector2f to an rtt::Vector2.
     */
    constexpr Vector2(const roboteam_msgs::Vector2f& msg) : Vector2(msg.x, msg.y) {}

    /**
     * \brief Calculate the dot product of this vector with another. (this . other)
     */
    double dot(const Vector2& other) const;

    /**
     * \brief Calculate the distance between this vector and another.
     */
    double dist(const Vector2& other) const;

    /**
     * \brief Calculate the square of the distance between this vector and another.
     * In cases where you only need to know how a number of distances relate to each other
     * (for example, which vector is closest to a specific other one), this method is
     * preferred to Vector2::dist, since you don't need the expensive sqrt operation.
     */
    double dist2(const Vector2& other) const;

    /**
     * \brief Scales this vector by a scalar.
     */
    Vector2 scale(double scalar) const;

    /**
     * \brief Normalizes this vector such that it will have a length of 1. The angle of the
     * vector is preserved. In other words, this method maps the vector onto the unit circle.
     */
    Vector2 normalize() const;

    /**
     * \brief Calculate the length of this vector.
     */
    double length() const;

    /**
     * \brief Calculate the angle of this vector, viewed from {0, 0}.
     * This uses the atan2 function, which has the following results:
     *     - The positive x-axis has angle 0
     *     - Vectors with a positive y value have a negative angle
     *     - Vectors with a negative or zero y value have a positive angle
     *     - The angle of the zero vector is undefined.
     */
    double angle() const;

    /**
     * \brief Performs linear interpolation/extrapolation.
     * 	   - If factor == 0, the result is a copy of this vector
     * 	   - If factor == 1, the result is a copy of other
     * 	   - For any other factor, the result is the vector at
     * 	     (factor*100)% along the way between this vector and the other.
     * For example, {0, 0}.lerp({1, -2}, 0.5) == {0.5, -1}
     */
    Vector2 lerp(const Vector2& other, double factor) const;

    /**
     * \brief Rotates this vector around the origin.
     */
    Vector2 rotate(double radials) const;

    /**
     * \brief Projects this vector onto a line segment defined by two other vectors.
     * The result is the point on the line segment (line_a, line_b) which is closest to
     * this vector.
     */
    Vector2 project(const Vector2& line_a, const Vector2& line_b) const;

    /**
     * \brief Projects this vector onto another vector.
     * The result is a vector.
     */
    Vector2 project2(const Vector2& ab) const;

    /**
     * \brief Checks whether both components of this vector are real (non-NaN) values.
     */
    bool isNotNaN() const;

    /**
     * Does the exact same as project... (?)
     */
    Vector2 closestPointOnVector(const Vector2& startPoint, const Vector2& point) const;

    /**
     * \brief Creates a vector with the same angle as this one, but with the specified length.
     */
    Vector2 stretchToLength(double length) const;

    /**
     * \brief Checks for equality.
     */
    bool operator==(const Vector2& other) const;

    /**
     * \brief Checks for inequality.
     */
    bool operator!=(const Vector2& other) const;

    /**
     * \brief Compares the length of this vector and another. The only real purpose for
     * this operator is to allow vectors to be used as map keys.
     */
    bool operator<(const Vector2& other) const;

    /**
     * \brief Adds another vector to this one, component-wise
     */
    Vector2 operator+(const Vector2& other) const;

    /**
     * \brief Adds a scalar to both components of this vector.
     */
    Vector2 operator+(const double& scalar) const;

    /**
     * \brief Subtracts another vector from this one, component-wise
     */
    Vector2 operator-(const Vector2& other) const;

    /**
     * \brief Subtracts a scalar from both components of this vector.
     */
    Vector2 operator-(const double& scalar) const;

    /**
     * \brief Multiplies another vector with this one, component-wise
     */
    Vector2 operator*(const Vector2& other) const;

    /**
     * \brief Scales this method by a scalar.
     * This operator behaves identically to the Vector2::scale(double) method.
     */
    Vector2 operator*(const double& scalar) const;

    /**
     * \brief Divides this vector by another, component-wise.
     */
    Vector2 operator/(const Vector2& other) const;

    /**
     * \brief Scales this method by the inverse of a scalar.
     */
    Vector2 operator/(const double& other) const;

    /**
     * \brief Set the values of this vector to the ones in the given ROS vector.
     */
    void operator=(const roboteam_msgs::Vector2f& msg);

    /**
     * \brief Casts or implicitly converts this vector to a ROS one.
     */
    operator roboteam_msgs::Vector2f() const;

    /**
     * \brief Writes a textual representation of this vector to the given output stream.
     */
    std::ostream& write(std::ostream& os) const;

    double x, y;
};

/**
 * \brief The zero vector, Vector2(0, 0)
 */
constexpr Vector2 ZERO_VECTOR{};

/**
 * \brief Writes a vector to an output stream.
 */
std::ostream& operator<<(std::ostream& os, const Vector2 vec);
}

#endif //ROBOTEAM_AI_VECTOR2_H
