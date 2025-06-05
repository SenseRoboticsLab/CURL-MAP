FROM osrf/ros:noetic-desktop-full
RUN apt-get update && apt-get install -y locales lsb-release curl
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

ARG NUM_THREADS

# switch gcc and g++ version
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils software-properties-common && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y gcc-11 g++-11 && \
    update-alternatives --install /usr/bin/cc cc /usr/bin/gcc-11 60 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60&& \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 60 && \
    update-alternatives --set cc /usr/bin/gcc-11 && \
    update-alternatives --set gcc /usr/bin/gcc-11 && \
    update-alternatives --set g++ /usr/bin/g++-11

# Install OpenSSL
RUN apt-get update && apt-get install -y libssl-dev

# get and build CMake
RUN apt-get update && apt-get install wget
RUN apt-get remove -y cmake && \
    wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz && \
    tar -zvxf cmake-3.20.0.tar.gz && \
    cd cmake-3.20.0 && \
    ./bootstrap && \
    make -j$(nproc) && \
    make install


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install curl -y # if you haven't already installed curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update

RUN apt-get update && apt-get install -y vim gedit ros-noetic-desktop-full build-essential libmpfr-dev libgmp-dev ros-noetic-jsk-recognition ros-noetic-jsk-common-msgs ros-noetic-jsk-rviz-plugins git tmux gdb clang-format htop
RUN apt-get update && apt-get install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
# RUN rosdep init
RUN rosdep update

# add a user
RUN useradd -m user && echo "user:123" | chpasswd
RUN echo "user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/user
RUN chsh -s /bin/bash user
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc

# config tmux with mouse

RUN echo "set -g mouse on" > /root/.tmux.conf
RUN echo "set -g mouse on" > /home/user/.tmux.conf

WORKDIR /home/user/Application
# eigen 3.4.0
# Install curl and unzip, in case they're not available
RUN apt-get update && apt-get install -y curl unzip
# Download the Eigen archive
RUN curl -o eigen-3.4.0.zip -L https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
# Unzip the downloaded file
RUN unzip eigen-3.4.0.zip
# Cleanup the zip file to keep the Docker image size down
RUN rm eigen-3.4.0.zip
RUN cd eigen-3.4.0 && \
    mkdir build installation && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../installation .. && \
    make -j${NUM_THREADS} && \
    make install
# Add your Python script to .gdbinit
RUN printf "python\nimport sys\nsys.path.insert(0, '/home/user/Application/eigen-3.4.0/debug/gdb')\nfrom printers import register_eigen_printers\nregister_eigen_printers(None)\nend\n" > /root/.gdbinit
RUN printf "python\nimport sys\nsys.path.insert(0, '/home/user/Application/eigen-3.4.0/debug/gdb')\nfrom printers import register_eigen_printers\nregister_eigen_printers(None)\nend\n" > /home/user/.gdbinit

# Ceres
RUN apt-get install libboost-all-dev && \
    # google-glog + gflags
    apt-get install -y libgoogle-glog-dev libgflags-dev && \
    # Use ATLAS for BLAS & LAPACK
    apt-get install -y libatlas-base-dev && \
    # SuiteSparse (optional)
    apt-get install -y libsuitesparse-dev && \
    # yaml-cpp
    apt-get install -y libyaml-cpp-dev
RUN git clone https://github.com/ceres-solver/ceres-solver.git
RUN cd ceres-solver && \
    git checkout 2.2.0 && \
    sed -i 's|find_package(Eigen3 3.3 REQUIRED)|set(Eigen3_DIR "/home/user/Application/eigen-3.4.0/installation/share/eigen3/cmake")\nfind_package(Eigen3)|' ./CMakeLists.txt && \
    mkdir build installation && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../installation .. && \
    make -j${NUM_THREADS} && \
    make install

# PCL
RUN git clone https://github.com/coin-or/metslib.git
RUN cd metslib && \
    git checkout releases/0.5.3 && \
    ./configure && \
    make && \
    make install
RUN rm -rf /usr/lib/cmake/vtk-7.1/
RUN git clone https://github.com/Kitware/VTK.git
RUN cd VTK && \
    git checkout v9.1.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j${NUM_THREADS} && \
    make install
RUN git clone https://github.com/PointCloudLibrary/pcl.git
RUN cd pcl && \
    git checkout pcl-1.13.0 && \
    mkdir build installation && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../installation .. && \
    make -j${NUM_THREADS} && \
    make install

RUN git clone https://github.com/isl-org/Open3D.git
RUN cd Open3D && \
    git checkout v0.18.0  && \
    bash ./util/install_deps_ubuntu.sh assume-yes &&\
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j${NUM_THREADS} && \
    make install

# CGAL
RUN git clone https://github.com/CGAL/cgal.git
RUN cd cgal && \
    git checkout v5.3 && \
    mkdir build installation && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../installation .. && \
    make -j${NUM_THREADS} && \
    make install

# Sophus
RUN git clone https://github.com/strasdat/Sophus.git
RUN cd Sophus && \
    git checkout 1.22.10 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j${NUM_THREADS} && \
    make install

# nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann.git
RUN cd nanoflann && \
    git checkout v1.4.3 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j${NUM_THREADS} && \
    make install

# range-v3
RUN git clone https://github.com/ericniebler/range-v3.git
RUN cd range-v3 && \
    git checkout 0.12.0

# livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
RUN cd Livox-SDK && \
    git checkout 9306596a2bf15c1343bc023b497465ed0a32909d && \
    sed -i '/#include "noncopyable.h"/a #include <memory>' ./sdk_core/src/base/thread_base.h && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j${NUM_THREADS} && \
    make install

WORKDIR /home/user/catkin_ws/src
# In Dockerfile

RUN git clone https://github.com/SenseRoboticsLab/CURL-MAP.git
RUN cd CURL-MAP && git checkout ICRA24


RUN git clone https://github.com/SenseRoboticsLab/patchwork-plusplus-ros-modified.git
RUN cd patchwork-plusplus-ros-modified && git checkout docker_version
# RUN sed -i 's|/home/zkc/Application|/home/user/Application|' ./patchwork-plusplus-ros-modified/CMakeLists.txt

WORKDIR /home/user/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release"
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc
RUN echo "source /home/user/catkin_ws/devel/setup.bash" >> /home/user/.bashrc
RUN echo "alias Open3D='/usr/local/bin/Open3D/Open3D'" >> /home/user/.bashrc

RUN sudo chown -R user:user /home/user/catkin_ws


RUN echo 'source /home/user/.bashrc' > /root/.bashrc

# RUN rm -rf ./src/CURL_SLAM_ros

RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd

# Set root password for SSH access (change 'your_password' to your desired password)
RUN sed -i 's|#X11UseLocalhost yes|X11UseLocalhost no|' /etc/ssh/sshd_config
RUN sed -i 's|#PermitRootLogin prohibit-password|PermitRootLogin yes|' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
RUN ssh-keygen -A
RUN chown -R user:user /etc/ssh
RUN service ssh restart

USER user

# EXPOSE 22
# CMD ["/usr/sbin/sshd", "-D"]