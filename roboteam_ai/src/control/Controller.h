//
// Created by kjhertenberg on 18-12-18.
//

#ifndef ROBOTEAM_AI_CONTROLLER_H
#define ROBOTEAM_AI_CONTROLLER_H

namespace control {

    class Controller {
    private:
        double kP;
        double kI;
        double kD;
        double timeDiff;
        double initial_I;
        double prev_error;

    public:
        Controller();
        Controller(double P, double I, double D);
        Controller(double P, double I, double D, double timeDiff);
        Controller(double P, double I, double D, double timeDiff, double initial, double prev);

        void setP(double P);
        void setI(double I);
        void setD(double D);
        void setInitial(double initial);
        void setTimeDiff(double time);
        void setPrevErr(double prev);


        void setP(double P, double time);
        void setI(double I, double time);
        void setD(double D, double time);
        void setPI(double P, double I);
        void setPI(double P, double I, double time);
        void setPD(double P, double D);
        void setPD(double P, double D, double time);
        void setID(double I, double D);
        void setID(double I, double D, double time);
        void setPID(double P, double I, double D);
        void setPID(double P, double I, double D, double time);

        double controlP(double err);
        double controlI(double err);
        double controlD(double err);
        double controlPI(double err);
        double controlPD(double err);
        double controlID(double err);
        double controlPID(double err);

        //To fill in your own measured velocity
        double controlR(double rate);
        double controlPR(double err, double rate);
        double controlIR(double err, double rate);
        double controlPIR(double err, double rate);

    };

}

#endif //ROBOTEAM_AI_CONTROLLER_H
