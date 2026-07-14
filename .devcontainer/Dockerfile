FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Base tools
RUN apt-get update && apt-get install -y \
    curl git cmake ninja-build pkg-config \
    python3 python3-pip \
    gdb lldb valgrind \
    clang-format clang-tidy cppcheck \
    xorg-dev libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# GCC 15
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && apt-get install -y gcc-15 g++-15 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-15 100 \
                        --slave   /usr/bin/g++ g++ /usr/bin/g++-15 && \
    rm -rf /var/lib/apt/lists/*

# Clang 20
RUN curl -fsSL https://apt.llvm.org/llvm.sh | bash -s -- 20 && \
    update-alternatives --install /usr/bin/clang   clang   /usr/bin/clang-20   100 \
                        --slave   /usr/bin/clang++ clang++ /usr/bin/clang++-20 && \
    rm -rf /var/lib/apt/lists/*

# Conan 2
RUN pip3 install conan --break-system-packages && conan profile detect

WORKDIR /workspace
