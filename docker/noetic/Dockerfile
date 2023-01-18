FROM koide3/gtsam_docker:noetic

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/ceres-solver/ceres-solver
WORKDIR /root/ceres-solver/build
RUN cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
RUN make -j$(nproc) && make install

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/noetic/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

COPY . /root/catkin_ws/src/direct_visual_lidar_calibration

WORKDIR /root/catkin_ws

RUN apt-get update \
  && /bin/bash -c '. /opt/ros/noetic/setup.bash; rosdep install -y --from-paths src'

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

WORKDIR /root/catkin_ws/src/direct_visual_lidar_calibration

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
