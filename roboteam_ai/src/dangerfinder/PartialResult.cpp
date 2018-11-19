//
// Created by mrlukasbos on 5-10-18.
//

#include "PartialResult.h"

PartialResult::PartialResult()
        :score(0), flags(0) { }

PartialResult::PartialResult(double score, DangerFlag flags)
        :score(score), flags(flags) { }

PartialResult &PartialResult::operator+=(const PartialResult &b) {
    score += b.score;
    flags |= b.flags;
    return *this;
}

PartialResult operator+(PartialResult a, PartialResult b) {
    return {a.score + b.score, (DangerFlag) (a.flags | b.flags)};
}
