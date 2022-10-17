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
#pragma once

#include <functional>
#include <type_traits>

namespace nativeformat {
namespace param {

/* The ParamEvent Anchor determines whether a ParamEvent is
 * tied to the start time and value, end time and value, or
 * the full Range. This determines how the start and end
 * values and times will change (or remain fixed) when other
 * ParaEvents are added.
 */
enum class Anchor { NONE = 0x0, START = 0x1, END = 0x2 };
inline Anchor operator&(Anchor a, Anchor b) {
    using T = std::underlying_type<Anchor>::type;
    return static_cast<Anchor>(static_cast<T>(a) & static_cast<T>(b));
}

struct ParamEvent {
    double start_time;
    double end_time;
    double start_value;

    const Anchor anchor;

    ParamEvent(double start_time, double end_time, Anchor anchor);
    virtual ~ParamEvent();

    virtual float valueAtTime(double time) = 0;
    virtual float endValue();

    static constexpr double INVALID_TIME = -1.0;
};

}  // namespace param
}  // namespace nativeformat
