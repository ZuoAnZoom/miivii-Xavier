#!/bin/bash

set -e

echo -e "[INFO] Run roscore..."
roscore &

sleep 2

source ~/robotcar_ws/devel/setup.bash
rosrun robotcar_bringup bringup


exit 0
