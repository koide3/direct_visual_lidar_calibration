FROM koide3/gtsam_docker:noetic

RUN apt-fast update \
  && apt-fast install -y --no-install-recommends \
  libomp-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/ceres-solver/ceres-solver \
  && cd ceres-solver \
  && git checkout e47a42c2957951c9fafcca9995d9927e15557069 \
  && mkdir build \
  && cd build \
  && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF \
  && make -j$(nproc) \
  && make install \
  && rm -rf /root/ceres-solver

RUN test -f /usr/share/doc/kitware-archive-keyring/copyright || wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null 
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal-rc main' | tee -a /etc/apt/sources.list.d/kitware.list >/dev/null

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/catkin_ws/src/direct_visual_lidar_calibration

WORKDIR /root/catkin_ws

RUN apt-fast update && \
  rosdep install -i --from-paths src --simulate | \
  sed '1d' | sed 's/apt-get install//' | sed 's/ //g' > /tmp/depends && \
  xargs apt-fast install --no-install-recommends -y < /tmp/depends && \
  apt-fast clean && \
  rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5"

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/noetic/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

WORKDIR /root/catkin_ws/src/direct_visual_lidar_calibration

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
