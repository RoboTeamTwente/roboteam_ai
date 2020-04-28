//
// Created by no one on never/never/ever
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

        double shootSuccesReward(Vector2 point) const;
    public:
        std::pair<vector_double, vector_double> get_bounds() const;

        vector_double fitness(const vector_double &aDouble) const;

        // TODO: when this function is finished, refactor so it only uses those components of world it actually needs
        void updateInfoForProblem(world_new::World* problemWorld);

        const double rel(double x, double min, double max) const;

        PassProblem() = default;

    };
}



#endif //RTT_PASSPROBLEM_H
