FROM osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797 AS vendor_workspace

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Keep ROS packages from the signed final Noetic snapshot in the base image.
# Python/ROS/Gazebo belong to Ubuntu Focal; the host Python is not used here.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-venv python3-yaml python3-numpy python3-opencv \
        ros-noetic-ros-controllers ros-noetic-gazebo-ros-control \
        ros-noetic-joint-state-publisher-gui \
        xvfb xauth libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/ainex-venv
COPY simulation/vendor/ainex/src/ainex_simulations /opt/ainex_ws/src/ainex_simulations
COPY simulation/vendor/ainex/src/ainex_interfaces /opt/ainex_ws/src/ainex_interfaces
COPY simulation/vendor/ainex/src/ainex_driver/ros_robot_controller /opt/ainex_ws/src/ros_robot_controller
COPY simulation/vendor/ainex/src/ainex_driver/ainex_kinematics /opt/ainex_ws/src/ainex_kinematics
COPY simulation/vendor.lock /opt/ainex-vendor.lock
COPY simulation/prepare_workspace.py /opt/ainex-prepare-workspace.py
RUN python3 /opt/ainex-prepare-workspace.py && \
    source /opt/ros/noetic/setup.bash && \
    cd /opt/ainex_ws && \
    catkin_make -j2 -DPYTHON_EXECUTABLE=/opt/ainex-venv/bin/python3

# Keep generated upstream interfaces cached while iterating on our sim package.
FROM vendor_workspace AS simulator
COPY simulation/ros_ws/src/robotrio_sim /opt/ainex_ws/src/robotrio_sim
RUN source /opt/ros/noetic/setup.bash && source /opt/ainex_ws/devel/setup.bash && \
    cd /opt/ainex_ws && \
    catkin_make -j2 -DPYTHON_EXECUTABLE=/opt/ainex-venv/bin/python3

RUN groupadd --gid 1000 sim && useradd --uid 1000 --gid 1000 --create-home sim
COPY simulation/entrypoint.sh /opt/ainex-entrypoint.sh
COPY simulation/run_smoke.sh /opt/ainex-run-smoke.sh
RUN chmod 755 /opt/ainex-entrypoint.sh /opt/ainex-run-smoke.sh && \
    printf '\nsource /opt/ros/noetic/setup.bash\nsource /opt/ainex_ws/devel/setup.bash\nsource /opt/ainex-venv/bin/activate\n' >> /home/sim/.bashrc

ENV VIRTUAL_ENV=/opt/ainex-venv \
    PATH=/opt/ainex-venv/bin:$PATH \
    PYTHONNOUSERSITE=1 \
    PIP_REQUIRE_VIRTUALENV=true \
    ROS_MASTER_URI=http://127.0.0.1:11311 \
    ROS_IP=127.0.0.1 \
    GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
    GAZEBO_MODEL_DATABASE_URI="" \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    QT_X11_NO_MITSHM=1
USER sim
WORKDIR /workspace
ENTRYPOINT ["/opt/ainex-entrypoint.sh"]
CMD ["bash"]
