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
#include "ParamImplementation.h"

#include <cmath>
#include <cstring>
#include <sstream>

namespace nativeformat {
namespace param {

ParamImplementation::ParamImplementation(float default_value, float max_value, float min_value, const std::string &name)
    : _default_value(default_value), _max_value(max_value), _min_value(min_value), _name(name) {
    _events.push_back(createEvent<DummyEvent>(default_value));
}

ParamImplementation::~ParamImplementation() {}

float ParamImplementation::yForX(double x) {
    std::lock_guard<std::mutex> events_mutex(_events_mutex);
    auto current_iterator = iteratorForTime(x);

    if (current_iterator == _events.end()) {
        return defaultY();
    }
    return std::min(std::max((*current_iterator)->valueAtTime(x), minY()), maxY());
}

std::string ParamImplementation::name() { return _name; }

float ParamImplementation::defaultY() const { return _default_value; }

float ParamImplementation::maxY() const { return _max_value; }

float ParamImplementation::minY() const { return _min_value; }

void ParamImplementation::setYAtX(float y, double x) {
    std::lock_guard<std::mutex> events_mutex(_events_mutex);
    auto prev_it = prevEvent(x);
    auto event = createEvent<ValueAtTimeEvent>(y, x);
    addEvent(std::move(event), prev_it);
}

void ParamImplementation::linearRampToYAtX(float end_y, double end_x) {
    {
        std::lock_guard<std::mutex> events_mutex(_events_mutex);
        auto prev_it = prevEvent(end_x);
        auto event = createEvent<LinearRampEvent>(end_y, end_x);
        addEvent(std::move(event), prev_it);
    }

    // implicit setYAtX to maintain the end_value
    setYAtX(end_y, end_x);
}

std::list<std::unique_ptr<ParamEvent>>::iterator ParamImplementation::iteratorForTime(double time) {
    if (time < 0.0) {
        return _events.end();
    }
    for (auto it = _events.begin(); it != _events.end(); it++) {
        EVENT_PTR &event = *it;
        if ((event->end_time > time || event->end_time == ParamEvent::INVALID_TIME) && event->start_time <= time) {
            return it;
        }
    }
    return _events.end();
}

std::list<std::unique_ptr<ParamEvent>>::iterator ParamImplementation::prevEvent(double time) {
    auto prev_it = _events.end();
    double prev_time = 0.0;
    for (auto it = _events.begin(); it != _events.end(); it++) {
        double t = ((*it)->anchor & Anchor::END) == Anchor::END ? (*it)->end_time : (*it)->start_time;
        if (t >= prev_time && t <= time) {
            prev_time = t;
            prev_it = it;
        }
    }
    return prev_it;
}

bool ParamImplementation::getRequiredTimeRange(const EVENT_PTR &event, double &start, double &end) {
    if (event->anchor == Anchor::NONE) {
        return false;
    }
    start = ((event->anchor & Anchor::START) == Anchor::START) ? event->start_time : event->end_time;
    end = ((event->anchor & Anchor::END) == Anchor::END) ? event->end_time : event->start_time;
    return true;
}

void ParamImplementation::checkOverlap(const EVENT_PTR &event) {
    double s1, e1, s2, e2;
    if (!getRequiredTimeRange(event, s1, e1)) {
        return;
    }
    for (const auto &e : _events) {
        if (getRequiredTimeRange(e, s2, e2)) {
            if (s1 < e2 && s2 < e1) {
                std::stringstream msg;
                msg << "New event with required time range " << s1 << " - " << e1 << " conflicts with existing event with required time range " << s2 << " - " << e2;
                throw std::invalid_argument(msg.str());
            }
        }
    }
}

void ParamImplementation::updateTimes(EVENT_PTR &prev, EVENT_PTR &event) {
    Anchor prev_anchor = prev->anchor;
    if ((event->anchor & Anchor::START) == Anchor::NONE) {
        // If event does not have a fixed start, get it from previous event
        double prev_anchor_time = 0.0;
        if ((prev_anchor & Anchor::END) == Anchor::END) {
            prev_anchor_time = prev->end_time;
        } else {
            // New event has no fixed start and previous event has no fixed end
            // set duration of previous event to 0 and steal its end value
            prev_anchor_time = prev->start_time;
            prev->end_time = prev_anchor_time;
            event->start_value = prev->endValue();
        }
        event->start_time = prev_anchor_time;
    } else {
        // If event does have a fixed start, give it to previous event
        if ((prev_anchor & Anchor::END) == Anchor::NONE) {
            prev->end_time = event->start_time;
        }
    }
}

void ParamImplementation::addEvent(EVENT_PTR new_event, std::list<EVENT_PTR>::iterator prev_event) {
    checkOverlap(new_event);

    auto next_event = prev_event;
    if (prev_event != _events.end()) {
        updateTimes(*prev_event, new_event);
        ++next_event;
        if (next_event != _events.end()) {
            updateTimes(new_event, *next_event);
        }
    }
    invalidateCachedCumulativeValuesAfterTime(new_event->start_time);
    _events.insert(next_event, std::move(new_event));
}

void ParamImplementation::invalidateCachedCumulativeValuesAfterTime(double time) {
    for (auto &precision_map_pair : _cumulative_values_cache) {
        auto &precision_map = precision_map_pair.second;
        for (auto &map_pair : precision_map) {
            if (map_pair.first >= time) {
                precision_map.erase(map_pair.first);
            }
        }
    }
}

std::unique_ptr<Param> createParam(float default_y, float max_y, float min_y, const std::string &name) {
    return std::make_unique<ParamImplementation>(default_y, max_y, min_y, name);
}

}  // namespace param
}  // namespace nativeformat
