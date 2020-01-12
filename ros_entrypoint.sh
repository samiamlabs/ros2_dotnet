#!/bin/bash
set -e

# setup ros2 environment
. "/opt/ros2_ws/install/local_setup.bash"

file="/opt/dotnet_ws/install/local_setup.bash"
if [ -f "$file" ]
then
  . "$file"
fi

exec "$@"
