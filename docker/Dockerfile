FROM ros:humble

# Install base packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    sudo \
    python3-pip \
    neovim \
    build-essential \
    git \
    libtool \
    libzmq3-dev \
    libusb-1.0-0-dev \
    zlib1g-dev \
    libudev-dev \
    gdb \
    libncurses5-dev \
    unzip \
    wget \
    && apt-get clean

ENV SHELL=/bin/bash

# Install gRPC v1.28.1 (required for Manus SDK remote mode)
RUN git clone -b v1.28.1 https://github.com/grpc/grpc /var/local/git/grpc \
    && cd /var/local/git/grpc \
    && git submodule update --init --recursive

RUN cd /var/local/git/grpc/third_party/protobuf \
    && ./autogen.sh \
    && ./configure --enable-shared \
    && make -j$(nproc) \
    && make install \
    && make clean \
    && ldconfig

RUN cd /var/local/git/grpc \
    && make -j$(nproc) \
    && make install \
    && make clean \
    && ldconfig

# Configure udev rules for Manus glove USB access
RUN mkdir -p /etc/udev/rules.d && \
    echo '# HIDAPI/libusb\nSUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"\nSUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"\n\n# HIDAPI/hidraw\nKERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"' \
    > /etc/udev/rules.d/70-manus-hid.rules

# Download and extract Manus C++ SDK (includes ROS2 package)
RUN wget -q https://static.manus-meta.com/resources/manus_core_3/sdk/MANUS_Core_3.1.1_SDK.zip -O /tmp/manus_sdk.zip \
    && unzip /tmp/manus_sdk.zip -d /opt/manus_sdk \
    && rm /tmp/manus_sdk.zip

# Build Manus ROS2 package with integrated SDK (no external Manus Core needed)
RUN mkdir -p /opt/ros2_ws/src \
    && cp -r /opt/manus_sdk/*/ROS2/* /opt/ros2_ws/src/ \
    && sed -i 's|^target_link_libraries(manus_data_publisher ManusSDK ncurses)|#target_link_libraries(manus_data_publisher ManusSDK ncurses)|; s|^#target_link_libraries(manus_data_publisher ManusSDK_Integrated ncurses)|target_link_libraries(manus_data_publisher ManusSDK_Integrated ncurses)|' /opt/ros2_ws/src/manus_ros2/CMakeLists.txt \
    && sed -i 's|^set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK.so)|#set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK.so)|; s|^#set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK_Integrated.so)|set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK_Integrated.so)|' /opt/ros2_ws/src/manus_ros2/CMakeLists.txt \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && cd /opt/ros2_ws && colcon build" \
    && chmod -R 777 /opt/ros2_ws

# Install Python dependencies for Manus visualization
RUN pip install open3d loop_rate_limiters

# Install USB debugging tools
RUN apt-get update && apt-get install -y usbutils tmux && apt-get clean

# ********************************************************
# * User setup goes last to preserve layer caching above *
# ********************************************************
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --force --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Source ROS and Manus workspaces on login
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Build glove bridge (TCP bridge for streaming ROS2 data to host)
COPY glove_bridge.c /tmp/glove_bridge.c
RUN gcc -Wall -O2 -o /usr/local/bin/glove_bridge /tmp/glove_bridge.c -lpthread \
    && rm /tmp/glove_bridge.c

COPY entrypoint.sh /entrypoint.sh

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
