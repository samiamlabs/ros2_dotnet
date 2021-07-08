FROM dynorobotics/balena-amd64-ros2:dashing-isolated-cyclone

# install dotnet build support
RUN wget -q https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
RUN dpkg -i packages-microsoft-prod.deb

RUN apt-get update && apt-get install -y \
  software-properties-common \
  && rm -rf /var/likb/apt/lists/*

RUN add-apt-repository universe

RUN apt-get update && apt-get install -y \
  apt-transport-https \
  dotnet-sdk-2.2 \
  patchelf \
  && rm -rf /var/likb/apt/lists/*

# build 
RUN mkdir -p /opt/dotnet_ws/src

WORKDIR /opt/dotnet_ws

COPY ros2_dotnet_dashing.repos ros2_dotnet_dashing.repos

RUN vcs import src < ros2_dotnet_dashing.repos

RUN rm -r src/dotnet/ros2_dotnet

COPY rcldotnet src/dotnet/ros2_dotnet/rcldotnet
COPY rcldotnet_common src/dotnet/ros2_dotnet/rcldotnet_common
COPY rcldotnet_examples src/dotnet/ros2_dotnet/rcldotnet_examples
COPY rcldotnet_tests src/dotnet/ros2_dotnet/rcldotnet_tests
COPY rosidl_generator_cs src/dotnet/ros2_dotnet/rosidl_generator_cs

WORKDIR /opt/dotnet_ws

# RUN . /opt/ros2_ws/install/setup.sh && \
#     apt-get update && \
#     rosdep install -q -y \
#       --from-paths \
#         src \
#       --ignore-src \
#       --rosdistro $CHOOSE_ROS_DISTRO \
#       --skip-keys "libcunit-dev message_runtime" \
#     && rm -rf /var/lib/apt/lists/*

RUN bash -c ". /opt/ros2_ws/install/local_setup.bash; colcon build"

COPY rcldotnet_utils src/dotnet/ros2_dotnet/rcldotnet_utils

RUN bash -c ". /opt/ros2_ws/install/local_setup.bash; colcon build"

RUN echo "source /opt/ros2_ws/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "source /opt/dotnet_ws/install/local_setup.bash" >> $HOME/.bashrc

COPY set_rpath_to_origin.bash /set_rpath_to_origin.bash

# source overlay workspace from entrypoint if available
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
