FROM dynorobotics/balena-amd64-ros2-dotnet:dashing

RUN mkdir -p /opt/dotnet_ws/src

WORKDIR /opt/dotnet_ws

COPY dist/ros2_dotnet_dashing.repos ros2_dotnet_dashing.repos

RUN vcs import src < ros2_dotnet_dashing.repos

RUN rm -r src/ros2_dotnet/ros2_dotnet

COPY dist src/ros2_dotnet/ros2_dotnet

RUN bash -c ". /opt/ros2_ws/install/setup.bash; colcon build"