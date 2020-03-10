//
// Created by rolf on 15-01-20.
//

#ifndef RTT_BBTRAJECTORY1D_H
#define RTT_BBTRAJECTORY1D_H

/**
 * Useful class helper.
 * This represents one segment of a bang-bang trajectory.
 * Each trajectory exists of 1,2 or 3 of these parts.
 */
struct BBTrajectoryPart {
  double tEnd;
  double acc;
  double startVel;
  double startPos;
};

/**
 * @brief small struct to keep Position, Velocity and Acceleration in at once.
 */
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

/**
 * @author Rolf
 * @brief Class that computes 1 dimensional bang bang trajectories.
 * These are time optimal given that bang-bang control where you pick an acceleration in (max,0,-max) is feasible.
 */
class BBTrajectory1D {
    public:
        /**
         * @brief Gets the position, velocity and acceleration at time t
         * @param t time to get values at
         * @return The PosVelAcc
         */
        PosVelAcc getValues(double t) const;
        /**
         * @brief Gets the position at time t
         * @param t time to get position at
         * @return Position at time t
         */
        double getPosition(double t) const;
        /**
         * @brief Gets the velocity at time t
         * @param t time to get velocity at
         * @return Velocity at time t
         */
        double getVelocity(double t) const;
        /**
         * @brief Gets the acceleration at time t
         * @param t time to get acceleration at.
         * @return Acceleration at time t
         */
        double getAcceleration(double t) const;
        /**
         * @brief Gets the total trajectory time.
         * @return Total time of the trajectory to end point
         */
        double getTotalTime() const;
        /**
         * Checks if time is in the last (decelerating to 0) part of the trajectory
         * @param t
         * @return true if t is in the last part of the trajectory
         */
        bool inLastPart(double t) const;
        /**
        * Generate a time-optimal trajectory given the parameters and a bang-bang control model where we
        * either accelerate/decelerate at max deceleration or drive at the max velocity.
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
         * @brief Computes the position where we would end if we would initiate a maximal break given current state
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
        double initialPos; // m
        double finalPos; // m
        double initialVel; // m/s
        double maxAcc; // m/s^2
        double maxVel; // m/s
};
#endif //RTT_BBTRAJECTORY1D_H
