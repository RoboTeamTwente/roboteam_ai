//
// Created by kjhertenberg on 18-12-18.
//

#include "Controller.h"

namespace control {
    Controller::Controller() {
        this->kP = 0;
        this->kI = 0;
        this->kD = 0;
        this->timeDiff = 0.001; //1000HZ
        this->initial_I = 0;
        this->prev_error = 0;
    }

    Controller::Controller(double P, double I, double D) {
        this->kP = P;
        this->kI = I;
        this->kD = D;
        this->initial_I = 0;
        this->prev_error = 0;
    }

    Controller::Controller(double P, double I, double D, double time) {
        this->kP = P;
        this->kI = I;
        this->kD = D;
        if (time != 0) {
            this->timeDiff = time;
        } else {
            this->timeDiff = 0.001; //the time difference cannot be 0, we divide by this, now it's at 1000HZ
        }
        this->initial_I = 0;
        this->prev_error = 0;
    }

    Controller::Controller(double P, double I, double D, double time, double initial, double prev) {
        this->kP = P;
        this->kI = I;
        this->kD = D;
        if (time != 0) {
            this->timeDiff = time;
        } else {
            this->timeDiff = 0.001; //the time difference cannot be 0, we divide by this, now it's at 1000HZ
        }
        this->initial_I = initial;
        this->prev_error = prev;
    }

    void Controller::setP(double P){
        this->kP = P;
    }

    void Controller::setI(double I){
        this->kI = I;
    }

    void Controller::setD(double D){
        this->kD = D;
    }

    void Controller::setInitial(double initial){
        this->initial_I = initial;
    }

    void Controller::setTimeDiff(double time){
        this->timeDiff = time;
    }

    void Controller::setPrevErr(double prev){
        this->prev_error = prev;
    }

    void Controller::setP(double P, double time){
        this->setP(P);
        this->setTimeDiff(time);
    }

    void Controller::setI(double I, double time){
        this->setI(I);
        this->setTimeDiff(time);
    }

    void Controller::setD(double D, double time){
        this->setD(D);
        this->setTimeDiff(time);
    }

    void Controller::setPI(double P, double I){
        this->setP(P);
        this->setI(I);
    }

    void Controller::setPD(double P, double D){
        this->setP(P);
        this->setD(D);
    }

    void Controller::setPI(double P, double I, double time){
        this->setP(P);
        this->setI(I);
        this->setTimeDiff(time);
    }

    void Controller::setPD(double P, double D, double time){
        this->setP(P);
        this->setD(D);
        this->setTimeDiff(time);
    }

    void Controller::setPID(double P, double I, double D){
        this->setP(P);
        this->setI(I);
        this->setD(D);
    }

    void Controller::setPID(double P, double I, double D, double time){
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
        double rateErr = (err - this->prev_error)/this->timeDiff;
        double value_D = this->kD * rateErr;
        this->prev_error = err;
        return value_D;
    }

    double Controller::controlPI(double err) {
        double value_P = this->controlP(err);
        double value_I = this->controlI(err);
        return value_P + value_I;
    }

    double Controller::controlPD(double err) {
        double value_P = this->controlP(err);
        double value_D = this->controlD(err);
        return value_P + value_D;
    }

    double Controller::controlID(double err) {
        double value_I = this->controlI(err);
        double value_D = this->controlD(err);
        return value_I + value_D;
    }

    double Controller::controlPID(double err) {
        double value_P = this->controlP(err);
        double value_I = this->controlI(err);
        double value_D = this->controlD(err);
        return value_P + value_I + value_D;
    }

}