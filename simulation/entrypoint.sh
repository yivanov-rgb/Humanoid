#!/usr/bin/env bash
set -e
# Catkin setup scripts inspect positional parameters (including --help). Keep
# application arguments out of their parser, then restore them for exec.
application_argv=("$@")
set --
source /opt/ros/noetic/setup.bash
source /opt/ainex_ws/devel/setup.bash
source /opt/ainex-venv/bin/activate
exec "${application_argv[@]}"
