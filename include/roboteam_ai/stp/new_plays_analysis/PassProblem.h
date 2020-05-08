//
// Created by RobotJesse on 1/4/2020
//

#ifndef RTT_PASSPROBLEM_H
#define RTT_PASSPROBLEM_H

#include <pagmo/problem.hpp>
#include "world_new/WorldData.hpp"
#include "world_new/views/WorldDataView.hpp"
#include "world/Field.h"

using namespace pagmo;

namespace rtt::ai::stp {
    class PassProblem : public pagmo::problem {
        /**
         * Fitness for a point on the field
         * @param input
         * @return
         */
    private:

        world_new::WorldData problemWorld{};
        world::Field problemField;

        static double shootSuccesReward(Vector2 point, world_new::view::WorldDataView world, const world::Field& field);
        
    public:
        /// Pagmo bounds function, only used for pagmo, don't change the signature as Pagmo algo's expect this exact function
        std::pair<vector_double, vector_double> get_bounds() const;

        /// Pagmo fitness function, only used for pagmo, don't change the signature as Pagmo algo's expect this exact function
        vector_double fitness(const vector_double &aDouble) const;

        /// Actual fitness function
        static double cost_function(const Vector2 &point, world_new::view::WorldDataView world, const world::Field& field);

        // TODO: when this function is finished, refactor so it only uses those components of world it actually needs
        void updateInfoForProblem(world_new::WorldData problemWorld, world::Field& field);
        PassProblem() = default;
    };
}
#endif //RTT_PASSPROBLEM_H
