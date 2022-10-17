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
 * Adapted by Timo van der Kuil, RoboTeam Twente
 */
#pragma once

#include <NFParam/ParamEvent.h>

#include <memory>
#include <vector>

namespace nativeformat {
namespace param {

struct ValueAtTimeEvent : ParamEvent {
    ValueAtTimeEvent(float value, double time);
    virtual ~ValueAtTimeEvent();

    float valueAtTime(double time) override;
};

struct LinearRampEvent : ParamEvent {
    const float target;

    LinearRampEvent(float value, double time);
    virtual ~LinearRampEvent();

    float valueAtTime(double time) override;
};

struct DummyEvent : ParamEvent {
    DummyEvent(float value);
    virtual ~DummyEvent();

    float valueAtTime(double time) override;
};

template <typename EventClass, typename... Args>
std::unique_ptr<EventClass> createEvent(Args... args) {
    return std::unique_ptr<EventClass>(new EventClass(args...));
}

}  // namespace param
}  // namespace nativeformat
