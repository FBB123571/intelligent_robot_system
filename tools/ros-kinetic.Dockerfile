FROM ros:kinetic-ros-base-xenial@sha256:b851ec0bbb8c8ce3e12a2c08235e63a64b1d59aa98d19298f29e0b9041176977
RUN apt-get -o Acquire::http::Timeout=20 -o Acquire::Retries=3 update && \
    apt-get -o Acquire::http::Timeout=20 -o Acquire::Retries=3 install -y --no-install-recommends \
    ros-kinetic-cv-bridge ros-kinetic-tf2-ros ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-move-base ros-kinetic-navfn ros-kinetic-dwa-local-planner \
    ros-kinetic-rostest ros-kinetic-ar-track-alvar ros-kinetic-ar-track-alvar-msgs \
    python-opencv python-numpy python-yaml && rm -rf /var/lib/apt/lists/*
COPY turtlebot/follow1/ar_marker_following /ws/src/ar_marker_following
COPY turtlebot/follow2/multi_robot_following/src/competition_core /ws/src/ar_marker_following/src/competition_core
RUN find /ws/src/ar_marker_following -name '*.py' -exec sed -i 's/\r$//' {} + && \
    chmod +x /ws/src/ar_marker_following/scripts/*.py && \
    [ -d /ws/src/ar_marker_following/test ] && chmod +x /ws/src/ar_marker_following/test/*.py || true
WORKDIR /ws
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'
CMD ["/bin/bash", "-c", "source devel/setup.bash && catkin_make run_tests && catkin_test_results --verbose"]
