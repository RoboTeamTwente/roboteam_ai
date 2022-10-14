/*
 * Copyright (c) 2016 Spotify AB.
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

#include <NFParam/Param.h>

#include <list>
#include <map>
#include <mutex>
#include <string>

#include "WAAParamEvents.h"

namespace nativeformat {
namespace param {

class ParamImplementation : public Param {
    typedef std::unique_ptr<ParamEvent> EVENT_PTR;

   public:
    ParamImplementation(float default_value, float max_value, float min_value, const std::string &name);
    virtual ~ParamImplementation();

    float yForX(double x) override;

    std::string name() override;

    // WAAParam
    float defaultY() const override;
    float maxY() const override;
    float minY() const override;
    void setYAtX(float y, double x) override;
    void linearRampToYAtX(float end_y, double end_x) override;

   private:
    const float _default_value;
    const float _max_value;
    const float _min_value;
    const std::string _name;
    std::list<EVENT_PTR> _events;
    std::mutex _events_mutex;
    std::vector<float> _smoothed_samples_buffer;
    std::map<double, std::map<double, float>> _cumulative_values_cache;

    // Find the event (if any) that governs the param curve at the given time
    std::list<std::unique_ptr<ParamEvent>>::iterator iteratorForTime(double time);

    // Find the last event (if any) whose anchor time is <= time
    std::list<std::unique_ptr<ParamEvent>>::iterator prevEvent(double time);

    // Populate start and end with the required start and end of an event.
    // If the event's anchor is NONE, getRequiredTimeRange will return false.
    // If the event's anchor is START or END, start = end.
    // If the event's anchor is ALL, end >= start.
    bool getRequiredTimeRange(const EVENT_PTR &event, double &start, double &end);

    // Throw an exception if an event overlaps with any existing events
    void checkOverlap(const EVENT_PTR &event);

    // Update events' start and end times according to their anchor.
    // Assumes prev's required time range ends before event's.
    void updateTimes(EVENT_PTR &prev, EVENT_PTR &event);

    // Update adjacent events on insertion of a new event
    void addEvent(EVENT_PTR new_event, std::list<EVENT_PTR>::iterator prev_event);

    void invalidateCachedCumulativeValuesAfterTime(double time);
};

}  // namespace param
}  // namespace nativeformat
