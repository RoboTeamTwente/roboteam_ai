//
// Created by kjhertenberg on 18-12-18.
//

#include "Controller.h"

namespace rtt{
namespace ai {
namespace control {


Controller::Controller(double P, double I, double D) : kP(P), kI(I), kD(D) { }

Controller::Controller(double P, double I, double D, double time) : kP(P), kI(I), kD(D) {
    if (time != 0) {
        this->timeDiff = time;
    }
}

Controller::Controller(double P, double I, double D, double time, double initial, double initial2,
                       double prev, double prev2) : kP(P), kI(I), kD(D) {
    if (time != 0) {
        this->timeDiff = time;
    } else {
        this->timeDiff =
                1.0 / rtt::ai::constants::tickRate; //the time difference cannot be 0, we divide by this
    }
    this->initial_I = initial;
    this->initial_I2 = initial2;
    this->prev_error = prev;
    this->prev_error2 = prev2;
}

void Controller::setP(double P) {
    this->kP = P;
}

void Controller::setI(double I) {
    this->kI = I;
}

void Controller::setD(double D) {
    this->kD = D;
}

void Controller::setTimeDiff(double time) {
    this->timeDiff = time;
}

void Controller::setInitial(double initial) {
    this->initial_I = initial;
}

void Controller::setPrevErr(double prev) {
    this->prev_error = prev;
}

void Controller::setP(double P, double time) {
    this->setP(P);
    this->setTimeDiff(time);
}

void Controller::setI(double I, double time) {
    this->setI(I);
    this->setTimeDiff(time);
}

void Controller::setD(double D, double time) {
    this->setD(D);
    this->setTimeDiff(time);
}

void Controller::setPID(double P, double I, double D) {
    this->setP(P);
    this->setI(I);
    this->setD(D);
}

void Controller::setPID(double P, double I, double D, double time) {
    this->setP(P);
    this->setI(I);
    this->setD(D);
    this->setTimeDiff(time);
}

double Controller::controlP(double err) {
    double value_P = this->kP * err;
    return value_P;
}

double Controller::controlI(double err) {
    this->initial_I += err * this->timeDiff;
    double value_I = this->kI * this->initial_I;
    return value_I;
}

double Controller::controlD(double err) {
    double rateErr = (err - this->prev_error) / this->timeDiff;
    double value_D = this->kD * rateErr;
    this->prev_error = err;
    return value_D;
}

double Controller::controlPID(double err) {
    double value_P = this->controlP(err);
    double value_I = this->controlI(err);
    double value_D = this->controlD(err);
    return value_P + value_I + value_D;
}

double Controller::controlR(double rate) {
    double value_R = this->kD * rate * -1;
    return value_R;
}

double Controller::controlPIR(double err, double rate) {
    double value_P = this->controlP(err);
    double value_I = this->controlI(err);
    double value_R = this->controlR(rate);
    return value_P + value_I + value_R;
}

Vector2 Controller::controlP(Vector2 err) {
    Vector2 value_P2;
    value_P2.x = this->controlP(err.x);
    value_P2.y = this->controlP(err.y);
    return value_P2;
}

Vector2 Controller::controlI(Vector2 err) {
    Vector2 value_I2;
    value_I2.x = this->controlI(err.x);
    this->initial_I2 += err.y * this->timeDiff;
    value_I2.y = this->kI * this->initial_I2;
    return value_I2;
}

Vector2 Controller::controlD(Vector2 err) {
    Vector2 value_D2;
    value_D2.x = this->controlD(err.x);
    double rateErr2 = (err.y - this->prev_error2) / this->timeDiff;
    value_D2.y = this->kD * rateErr2;
    this->prev_error2 = err.y;
    return value_D2;
}

Vector2 Controller::controlPID(Vector2 err) {
    Vector2 value_P2 = this->controlP(err);
    Vector2 value_I2 = this->controlI(err);
    Vector2 value_D2 = this->controlD(err);
    return value_P2 + value_I2 + value_D2;
}

//To fill in your own measured velocity
Vector2 Controller::controlR(Vector2 rate) {
    Vector2 value_R2;
    value_R2.x = this->kD * rate.x * -1;
    value_R2.y = this->kD * rate.y * -1;
    return value_R2;
}

Vector2 Controller::controlPIR(Vector2 err, Vector2 rate) {
    Vector2 value_P2 = this->controlP(err);
    Vector2 value_I2 = this->controlI(err);
    Vector2 value_R2 = this->controlR(rate);
    return value_P2 + value_I2 + value_R2;
}
double Controller::getP() {
    return kP;
}
double Controller::getI() {
    return kI;
}
double Controller::getD() {
    return kD;
}

void Controller::reset() {
    kP = 0;
    kI = 0;
    kD = 0;
    timeDiff = 1.0 / rtt::ai::constants::tickRate;
    initial_I = 0;
    initial_I2 = 0; //only used in the case of 2 input variables
    prev_error = 0;
    prev_error2 = 0; //only used in the case of 2 input variables
}

} // control
} // ai
} // rtt