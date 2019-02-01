FROM arm32v7/ubuntu:xenial
LABEL maintainer eko.rudiawan@gmail.com

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# install opencv, etc
RUN apt-get update && apt-get install -y \
    ros-kinetic-usb-cam \
    ros-kinetic-image-view \
    ros-kinetic-rqt-image-view \
    ros-kinetic-opencv3 \
    terminator \
    guvcview \
    netcat && \
    rm -rf /var/lib/apt/lists/*

# Install Tensorflow
RUN cd ~ && apt-get update && apt-get install -y \
    wget \
    python-pip \
    unzip && \
    wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v1.10.0/tensorflow-1.10.0-cp27-none-linux_armv7l.whl && \
    pip install tensorflow-1.10.0-cp27-none-linux_armv7l.whl

RUN apt-get update && apt-get install -y \
    libblas-dev \
    liblapack-dev \
    libzbar0 \
    libhdf5-serial-dev

RUN apt-get update && pip install --upgrade pip
RUN pip install matplotlib 
RUN pip install tqdm 
RUN pip install pillow imutils 
RUN pip install scipy --no-cache-dir
RUN pip install keras==2.1.6
RUN pip install ipython 
RUN pip install jupyter 
RUN apt-get update && apt-get install libstdc++6
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y gcc-4.9 && apt-get upgrade -y libstdc++6
# RUN apt-get update && apt-get install -y python-qrtools libzbar-dev
# RUN pip install pyzbar
# RUN pip uninstall -y pyOpenSSL
# RUN pip install pyOpenSSL==17.3.0
# RUN pip install -U pyOpenSSL
# RUN pip install pyzbar

# RUN python -m pip install numpy matplotlib ipython jupyter tqdm keras
    
# RUN pip install -U scipy && \
#     pip install -U numpy && \
#     pip install -U matplotlib && \
#     pip install -U tqdm && \
#     pip install -U keras && \
#     pip install -U scikit-learn
    
# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]