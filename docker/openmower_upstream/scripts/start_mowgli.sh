#!/bin/bash
echo "Starting OpenMower (upstream)"
source /config/mower_config.sh
roslaunch --wait open_mower open_mower.launch
