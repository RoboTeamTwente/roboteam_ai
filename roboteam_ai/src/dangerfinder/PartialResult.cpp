//
// Created by mrlukasbos on 5-10-18.
//

#include "PartialResult.h"

PartialResult::PartialResult()
        :score(0), flags(0) { }

PartialResult::PartialResult(double score, DangerFlag flags)
        :score(score), flags(flags) { }


/**
* \function operator+=
* \brief Sums the scores, ORs the flags.
*/
PartialResult &PartialResult::operator+=(const PartialResult &b) {
    score += b.score;
    flags |= b.flags;
    return *this;
}

/**
 * \function operator+
 * \brief Combines two PartialResults by summing their scores and combining their flags.
 */
PartialResult operator+(PartialResult a, PartialResult b) {
    return {a.score + b.score, (DangerFlag) (a.flags | b.flags)};
}
