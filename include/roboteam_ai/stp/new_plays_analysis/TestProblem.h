//
// Created by timo on 4/8/20.
//

#ifndef RTT_TESTPROBLEM_H
#define RTT_TESTPROBLEM_H

#include <pagmo/problem.hpp>
#include <world_new/World.hpp>

namespace rtt::ai::stp {
class TestProblem : public pagmo::problem {
    /**
     * Fitness for a point on the field
     * @param input
     * @return
     */
   private:
    world_new::World* problemWorld{nullptr};

   public:
    std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;

    pagmo::vector_double fitness(const pagmo::vector_double& aDouble) const;

    // TODO: when this function is finished, refactor so it only uses those components of world it actually needs
    void updateInfoForProblem(world_new::World* problemWorld);

    const double rel(double x, double min, double max) const;

    TestProblem() = default;
};
}  // namespace rtt::ai::stp

#endif  // RTT_TESTPROBLEM_H
