ARG BASE_IMAGE_NAME=amd64/ubuntu:20.04
From $BASE_IMAGE_NAME AS base
LABEL maintainer="Jinwoo Park"\
      description="X86_64 base image for TRAITS framework."\
      version="1.0.0"
SHELL ["/bin/bash", "-c"]
ARG NUM_BUILD_CORES=9

ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Says not a good practice...
# ARG DEBIAN_FRONTEND=noninteractive
# RUN apt-get -qq install {your-package}
# is preferred
ENV DEBIAN_FRONTEND noninteractive

RUN apt update --fix-missing &&\
    apt upgrade -y &&\
    apt-get install -y --no-install-recommends software-properties-common lsb-release

# gcc-11 & g++-11
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test

RUN apt update --fix-missing &&\
    apt upgrade -y &&\
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        doxygen \
#        libeigen3-dev \
        g++-11 \
        gcc-11 \
        git \
        graphviz \
        graphviz-dev \
        lcov \
        libboost-all-dev \
        libglu1-mesa-dev \
        libtinyxml2-dev \
        libyaml-cpp-dev \
        python3 \
        python3-dev \
        python3-distutils \
        python3-pip \
        python3-pybind11 \
        unzip \
        valgrind \
        wget \
        xorg-dev \
        zlib1g-dev

# Eigen
RUN git clone --depth 1 --branch 3.4.0 --single-branch https://gitlab.com/libeigen/eigen.git
RUN mkdir -p eigen/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -S eigen -B eigen/build &&\
    cmake --build eigen/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install eigen/build &&\
    rm -rf eigen
# fix eigen
RUN ln -s /usr/local/include/eigen3/Eigen /usr/include/Eigen

RUN update-alternatives --install /usr/bin/python pyhon /usr/bin/python3 3 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11

# install pip packages
RUN pip3 install --upgrade \
    dgl \
    gcovr \
    ipython \
    matplotlib \
    networkx \
    numpy \
    pandas \
    progress \
    pygraphviz \
    torch \
    dgl \ 
    dglgo 
    #torch && \
    #pip3 install --pre dgl -f https://data.dgl.ai/wheels/cu117/repo.html &&\
    #pip3 install --pre dglgo -f https://data.dgl.ai/wheels-test/repo.html

RUN apt purge --auto-remove -y cmake
RUN mkdir /opt/cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.7/cmake-3.27.7-linux-x86_64.sh
RUN sh /cmake-3.27.7-linux-x86_64.sh --prefix=/opt/cmake --skip-license
RUN ln -s /opt/cmake/bin/cmake /usr/bin/cmake
RUN cmake --version

# Include OMPL (python bindings are off) (version 1.6.0)
RUN git clone --depth 1 --branch 1.6.0 --single-branch https://github.com/ompl/ompl.git &&\
    mkdir -p ompl/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DOMPL_BUILD_DEMOS=OFF \
          -DOMPL_BUILD_PYBINDINGS=OFF \
          -DOMPL_BUILD_PYTESTS=OFF \
          -DOMPL_BUILD_TESTS=OFF \
          -DOMPL_REGISTRATION=OFF\
          -DOMPL_VERSIONED_INSTALL=OFF\
          -S ompl -B ompl/build &&\
    cmake --build ompl/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install ompl/build &&\
    rm -rf ompl

## Google Test (version 1.12.1)
RUN git clone --depth 1 --branch release-1.12.1 --single-branch https://github.com/google/googletest.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p googletest/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_GMOCK=ON \
          -DINSTALL_GTEST=ON \
          -S googletest -B googletest/build &&\
    cmake --build googletest/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install googletest/build &&\
    rm -rf googletest

## nlohmann json (version 3.11.2)
RUN git clone --depth 1 --branch v3.11.2 --single-branch https://github.com/nlohmann/json.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p json/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DJSON_BuildTests=OFF \
          -S json -B json/build &&\
    cmake --build json/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install json/build &&\
    rm -rf json

## fmt (version 9.1.0)
RUN git clone --depth 1 --branch 9.1.0 --single-branch https://github.com/fmtlib/fmt.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p fmt/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DFMT_DOC=OFF \
          -DFMT_TEST=OFF \
          -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE\
          -S fmt -B fmt/build &&\
    cmake --build fmt/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install fmt/build &&\
    rm -rf fmt

## spdog (version 1.11.0)
RUN git clone --depth 1 --branch v1.11.0 --single-branch https://github.com/gabime/spdlog.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p spdlog/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DSPDLOG_BUILD_PIC=ON \
          -DSPDLOG_FMT_EXTERNAL=ON \
          -DSPDLOG_FMT_EXTERNAL_HO=OFF \
          -DSPDLOG_BUILD_TESTS=OFF \
          -DSPDLOG_BUILD_EXAMPLE=OFF \
          -S spdlog -B spdlog/build &&\
    cmake --build spdlog/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install spdlog/build &&\
    rm -rf spdlog

## range-v3
RUN git clone --depth 1 --branch 0.12.0 --single-branch https://github.com/ericniebler/range-v3.git range &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p range/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DRANGES_BUILD_CALENDAR_EXAMPLE=OFF\
          -DRANGES_DEBUG_INFO=OFF\
          -DRANGE_V3_TESTS=OFF \
          -DRANGE_V3_EXAMPLES=OFF \
          -DRANGE_V3_PERF=OFF \
          -DRANGE_V3_DOCS=Off \
          -S range -B range/build &&\
    cmake --build range/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install range/build &&\
    rm -rf range

## cppcoro
RUN git clone --depth 1 --branch main --single-branch https://github.com/andreasbuhr/cppcoro.git &&\
    export CXXFLAGS="-Ofast -fcoroutines" &&\
    mkdir -p cppcoro/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -S cppcoro -B cppcoro/build &&\
    cmake --build cppcoro/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install cppcoro/build &&\
    rm -rf cppcoro


# Install third party libs

## Install gurobi
ENV GUROBI_INSTALL /opt/gurobi
ENV GUROBI_HOME $GUROBI_INSTALL/linux64
ENV PATH $PATH:$GUROBI_HOME/bin
ENV GUROBI_LIB_PATH $GUROBI_HOME/lib

ARG GUROBI_MAJOR_VERSION=11.0
ARG GUROBI_VERSION=11.0.3
ARG GUROBI_FOLDER_VERSION=1103

RUN mkdir -p ${GUROBI_INSTALL} &&\
    wget http://packages.gurobi.com/${GUROBI_MAJOR_VERSION}/gurobi${GUROBI_VERSION}_linux64.tar.gz &&\
    tar xvfz gurobi${GUROBI_VERSION}_linux64.tar.gz &&\
    mv gurobi${GUROBI_FOLDER_VERSION}/linux64/ ${GUROBI_INSTALL}
RUN export CXXFLAGS="-Ofast" && cd ${GUROBI_HOME}/src/build && make -j ${NUM_BUILD_CORES}
RUN mv ${GUROBI_HOME}/src/build/libgurobi_c++.a ${GUROBI_LIB_PATH}/libgurobi_g++11.3.a
RUN cd ${GUROBI_LIB_PATH} && ln -sf libgurobi_g++11.3.a libgurobi_c++.a
RUN rm -r ${GUROBI_HOME}/docs &&\
    rm -r ${GUROBI_HOME}/examples &&\
    rm -r ${GUROBI_HOME}/src &&\
    rm ${GUROBI_HOME}/bin/python3.11 &&\
    rm -r gurobi${GUROBI_FOLDER_VERSION}/ &&\
    rm gurobi${GUROBI_VERSION}_linux64.tar.gz

### Install libtorch (must be installed separate because the libtorch in pytorch is pre-cxx11 ABI...)
## Linux w/ CUDA
ENV TORCH_INSTALL_PREFIX /opt/libtorch
RUN wget https://download.pytorch.org/libtorch/cu117/libtorch-cxx11-abi-shared-with-deps-1.13.1%2Bcu117.zip &&\
    unzip libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117.zip &&\
    mv libtorch /opt/libtorch &&\
    rm libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117.zip

RUN apt autoclean &&\
    apt autoremove &&\
    apt clean &&\
    apt install -y vim gdb gdbserver &&\
    rm -rf /var/lib/apt/lists/*

## concurrencpp
#RUN git clone -b develop https://github.com/amessing/concurrencpp.git &&\
RUN git clone https://github.com/David-Haim/concurrencpp.git &&\
    export CXXFLAGS="-Ofast -fcoroutines" &&\
    mkdir -p concurrencpp/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -S concurrencpp -B concurrencpp/build &&\
    cmake --build concurrencpp/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install concurrencpp/build &&\
    rm -rf concurrencpp

RUN echo 'export LD_LIBRARY_PATH="/usr/local/lib/:/usr/lib"' >> ~/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8/site-packages/' >> ~/.bashrc

# For SSH
ARG CUSTOM_USER=debugger
ARG home=/home/$CUSTOM_USER

RUN apt update --fix-missing &&\
    apt-get install -y --no-install-recommends \
        apt-utils \
        dbus-x11 \
        openssh-server \
        gdb \
        gdbserver \
        gosu \
        mesa-utils \
        rsync \
        vim \
        xauth \
        tmux \
        htop

# Check that gosu works for the entrypoint
RUN gosu nobody true

RUN apt autoclean &&\
    apt autoremove &&\
    apt clean &&\
    rm -rf /var/lib/apt/lists/*

RUN mkdir /var/run/sshd &&\
    echo 'root:root' | chpasswd &&\
    sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config &&\
    sed -i 's/#AddressFamily any/AddressFamily inet/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

RUN sed -i 's/Port 22/Port 7776/' /etc/ssh/sshd_config
# 22 for ssh server. 7776 for gdb server.
# EXPOSE 22 7776

RUN useradd -ms /bin/bash $CUSTOM_USER
RUN echo $CUSTOM_USER:pwd | chpasswd

USER $CUSTOM_USER
ENV HOME $home

USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7776"]