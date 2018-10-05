#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

PartialResult::PartialResult() : score(0), flags(0) {}
PartialResult::PartialResult(double score, DangerFlag flags) : score(score), flags(flags) {}

PartialResult& PartialResult::operator+=(const PartialResult& b) {
	score += b.score;
	flags |= b.flags;
	return *this;
}

PartialResult operator+(PartialResult a, PartialResult b) {
	return { a.score + b.score, (DangerFlag)(a.flags | b.flags) };
}

} // dangerfinder
} // ai
} // rtt
