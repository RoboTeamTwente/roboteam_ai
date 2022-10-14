/*
 * Copyright (c) 2018 Spotify AB.
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 * Adapted by RoboTeam Twente
 */

#include "WAAParamEvents.h"

#include <cmath>

namespace nativeformat {
namespace param {

ValueAtTimeEvent::ValueAtTimeEvent(float value, double time) : ParamEvent(time, ParamEvent::INVALID_TIME, Anchor::START) { ParamEvent::start_value = value; }

ValueAtTimeEvent::~ValueAtTimeEvent() {}

float ValueAtTimeEvent::valueAtTime(double time) { return start_value; }

LinearRampEvent::LinearRampEvent(float value, double time) : ParamEvent(0.0, time, Anchor::END), target(value) {}
LinearRampEvent::~LinearRampEvent() {}

float LinearRampEvent::valueAtTime(double time) {
    if (time < start_time || start_time == end_time) {
        return start_value;
    }
    if (time > end_time) {
        return target;
    }
    double c = (time - start_time) / (end_time - start_time);
    return start_value + (target - start_value) * c;
}

DummyEvent::DummyEvent(float value) : ParamEvent(0.0, ParamEvent::INVALID_TIME, Anchor::NONE) { ParamEvent::start_value = value; }

DummyEvent::~DummyEvent() {}

float DummyEvent::valueAtTime(double time) { return start_value; }

}  // namespace param
}  // namespace nativeformat
