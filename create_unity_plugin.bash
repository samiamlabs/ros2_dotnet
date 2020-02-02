#!/bin/bash

docker build --pull -t ros2-dotnet .
docker run -it --rm --volume "$(pwd)/Plugins:/opt/Plugins" ros2-dotnet bash -c  "cd /opt && ros2 run rcldotnet_utils create_unity_plugin"

