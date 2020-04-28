//
// Created by RobotJesse on 1/4/2020
//

#ifndef RTT_PASSPROBLEM_H
#define RTT_PASSPROBLEM_H

#include <pagmo/problem.hpp>
using namespace pagmo;
namespace rtt::ai::stp {
    class PassProblem : public pagmo::problem {
        /**
         * Fitness for a point on the field
         * @param input
         * @return
         */
    private:
        std::mutex world_mutex;

        world_new::World* problemWorld{nullptr};

        static double shootSuccesReward(Vector2 point, world_new::World* world);
    public:
        /// Pagmo bounds function, only used for pagmo, don't change the signature as Pagmo algo's expect this exact function
        std::pair<vector_double, vector_double> get_bounds() const;

        /// Pagmo fitness function, only used for pagmo, don't change the signature as Pagmo algo's expect this exact function
        vector_double fitness(const vector_double &aDouble) const;

        /// Actual fitness function
        static double fitness(const Vector2 &point, world_new::World* world);

        // TODO: when this function is finished, refactor so it only uses those components of world it actually needs
        void updateInfoForProblem(world_new::World* problemWorld);

        PassProblem() = default;

    };
}



#endif //RTT_PASSPROBLEM_H
