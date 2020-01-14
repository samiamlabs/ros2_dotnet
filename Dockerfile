FROM dynorobotics/balena-amd64-ros2-dotnet:dashing

RUN mkdir -p /opt/dotnet_ws/src

WORKDIR /opt/dotnet_ws

COPY ros2_dotnet_dashing.repos ros2_dotnet_dashing.repos

RUN vcs import src < ros2_dotnet_dashing.repos

RUN rm -r src/dotnet/ros2_dotnet

COPY . src/dotnet/ros2_dotnet

WORKDIR /opt/dotnet_ws
RUN . /opt/ros2_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
      --rosdistro $CHOOSE_ROS_DISTRO \
      --skip-keys "libcunit-dev" \
    && rm -rf /var/lib/apt/lists/*

RUN bash -c ". /opt/ros2_ws/install/local_setup.bash; colcon build"

RUN echo "source /opt/ros2_ws/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "source /opt/dotnet_ws/install/local_setup.bash" >> $HOME/.bashrc

# source overlay workspace from entrypoint if available
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
