#include <NFParam/ParamEvent.h>

namespace nativeformat {
namespace param {

ParamEvent::ParamEvent(double start_time, double end_time, Anchor anchor) : start_time(start_time), end_time(end_time), start_value(0), anchor(anchor) {}

ParamEvent::~ParamEvent() {}

float ParamEvent::endValue() { return valueAtTime(end_time); }

}  // namespace param
}  // namespace nativeformat
