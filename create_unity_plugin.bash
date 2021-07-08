#!/bin/bash

sudo rm -rf Plugins
docker build --pull -t ros2-dotnet .
docker run -it --rm --volume "$(pwd)/Plugins:/opt/Plugins" ros2-dotnet bash -c  "cd /opt && ros2 run rcldotnet_utils create_unity_plugin; /set_rpath_to_origin.bash"
sudo chown -R $USERNAME Plugins
