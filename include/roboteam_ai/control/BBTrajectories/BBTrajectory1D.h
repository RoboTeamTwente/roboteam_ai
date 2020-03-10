//
// Created by rolf on 15-01-20.
//

#ifndef RTT_BBTRAJECTORY1D_H
#define RTT_BBTRAJECTORY1D_H

struct BBTrajectoryPart {
  double tEnd;
  double acc;
  double startVel;
  double startPos;
};

struct PosVelAcc {
  explicit PosVelAcc(double pos, double vel, double acc)
          :
          pos{pos},
          vel{vel},
          acc{acc} { };

  double pos;
  double vel;
  double acc;
};

class BBTrajectory1D {
    public:
        PosVelAcc getValues(double t) const;
        double getPosition(double t) const;
        double getVelocity(double t) const;
        double getAcceleration(double t) const;
        double getTotalTime() const;
        bool inLastPart(double t) const;
        /**
        * Generate a time-optimal trajectory given the parameters
        * @param initialPos Current position
        * @param initialVel Current velocity
        * @param finalPos Final position
        * @param maxVel Maximum allowed velocity (absolute)
        * @param maxAcc maximum allowed acceleration/deceleration
        */
        void generateTrajectory(double initialPos, double initialVel, double finalPos, double maxVel, double maxAcc);
        BBTrajectory1D(double initialPos, double initialVel,double finalPos, double maxVel, double maxAcc);
        BBTrajectory1D() = default;
    private:

        /**
         * Computes the position where we would end if we would initiate a maximal break given current state
         * @param pos Current position
         * @param vel Current velocity
         * @param maxAcc Maximum allowed acceleration/deceleration
         * @return final position (with zero velocity)
         */
        double fullBrakePos(double pos, double vel, double accMax);
        /**
         * Computes the position where we would end if we first accelerate to a target velocity and then immediately decelerate to 0
         * @param pos0 Current position
         * @param vel0 Current velocity
         * @param vel1 Target velocity
         * @param accMax Maximum allowed acceleration/deceleration
         * @return final position (with zero velocity)
         */
        double accelerateBrakePos(double pos0, double vel0, double vel1, double accMax);
        /**
         * Generates a time-optimal triangular profile (accelerating and then breaking)
         * @param initialPos Current position
         * @param initialVel Current velocity
         * @param finalPos Desired final psition
         * @param maxAcc Maximum allowed acceleration/deceleration
         * @param invertedSign Specifies whether or not to invert the sign of velocity / acceleration. This is more of a techniality
         */
        void triangularProfile(double initialPos, double initialVel, double finalPos, double maxAcc, bool invertedSign);
        /**
         * Generates a time-optimal trapezoidal profile (accelerating, coasting, then breaking)
         * @param initialPos Current position
         * @param initialVel Current velocity
         * @param maxVel Coasting velocity
         * @param finalPos Desired final position
         * @param maxAcc Maximum allowed acceleration/deceleration
         */
        void trapezoidalProfile(double initialPos, double initialVel, double finalPos, double maxVel, double maxAcc);
        /**
         * Updates a part of the trajectory
         * @param index which part to update
         * @param tEnd time spent on trajectory segment
         * @param acc acceleration on segment
         * @param vel initial velocity of segment
         * @param pos initial position of segment
         */
        void updatePart(int index, double tEnd, double acc, double vel, double pos);

        BBTrajectoryPart parts[3];
        unsigned short int numParts{};
        //TODO: if in the future one of these fields is uncalled (particularly initialVel, intialPos are redundant) they can be removed.
        //The memory overhead for now seems minimal
        double m_initialPos; // m
        double m_finalPos; // m
        double m_initialVel; // m/s
        double m_maxAcc; // m/s^2
        double m_maxVel; // m/s
};
#endif //RTT_BBTRAJECTORY1D_H
