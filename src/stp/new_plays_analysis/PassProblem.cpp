//
// Created by jesse on 27-03-20.
//

#include <roboteam_utils/Print.h>
#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include "include/roboteam_ai/stp/new_plays_analysis/PassProblem.h"
using namespace pagmo;
namespace rtt::ai::stp{


    vector_double PassProblem::fitness(const vector_double &dv) const {
        //vector_double v = calculateScoreForPoint(dv[0],dv[1]);
        //return v;
        double alpha = 1;
        double alphaMax = 1;
        double pi = 3.14;
        auto passTarget = Vector2(dv[0], dv[1]);
        auto pointscore = 1 - rel(alpha, pi = 3.14 / 4, alphaMax);



        return {dv[0] + dv[1]};
    }

    std::pair<vector_double, vector_double> PassProblem::get_bounds() const {
        return {{-1., -1.}, {1., 1.}};
    }

    const double PassProblem::rel(double x, double min, double max) const {
        if (x >= max) {
            return 1;
        }
        if (min <= x <= max) {
            return (x-min)/(max-min);
        }
        else {
            return 0;
        }
    }


}
