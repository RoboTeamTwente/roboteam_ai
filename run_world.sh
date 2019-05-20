#!/bin/bash

rosrun roboteam_world kalman_world
while [ $? -ne 0 ]; do
  notify-send -u critical "World" "World crashed!"
  rosrun roboteam_world kalman_world
done
