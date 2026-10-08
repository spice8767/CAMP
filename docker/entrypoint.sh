#!/bin/bash
source /opt/ros/jazzy/setup.bash
[ -f /home/dev/ws/install/setup.bash ] && source /home/dev/ws/install/setup.bash
exec "$@"

