/*
 * Copyright (c) 2017 Spotify AB.
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

#include <NFParam/ParamEvent.h>

#include <memory>
#include <vector>

namespace nativeformat::param {

class Param {
public:
  virtual float defaultY() const = 0;
  virtual float maxY() const = 0;
  virtual float minY() const = 0;
  virtual void setY(float y) = 0;
  virtual void setYAtX(float y, double x) = 0;
  virtual void linearRampToYAtX(float end_y, double end_x) = 0;
  virtual void setTargetYAtX(float target_y, double start_x,
                             float x_constant) = 0;
  virtual void exponentialRampToYAtX(float y, double end_x) = 0;
  virtual void setYCurveAtX(std::vector<float> y_values, double start_x,
                            double duration) = 0;

  // other methods
  virtual void addCustomEvent(double start_x, double end_x, Anchor anchor,
                              NF_AUDIO_PARAM_FUNCTION function) = 0;
  virtual float yForX(double x) = 0;
  virtual void yValuesForXRange(float *y_values, size_t y_values_count,
                                double start_x, double end_x) = 0;
  virtual std::string name() = 0;

  virtual float smoothedYForXRange(double start_x,
                                          double end_x,
                                          size_t samples = 5) = 0;
  virtual float cumulativeYForXRange(double start_x,
                                            double end_x,
                                            double precision = 0.1) = 0;

  virtual ~Param() = default;
};

std::unique_ptr<Param> createParam(float default_y, float max_y, float min_y,
                                   const std::string &name);

} // namespace nativeformat::param