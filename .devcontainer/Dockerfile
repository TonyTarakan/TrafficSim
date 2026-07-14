FROM ubuntu:26.04

# Update the package database
RUN apt-get clean
RUN apt-get update

# Install common tools
RUN apt-get update && apt-get install --no-install-recommends --assume-yes \
        curl wget gpg software-properties-common \
        git cmake ninja-build pkg-config         \
        python3-minimal python3-pip pipx         \
        gdb valgrind cppcheck                    \
        less vim sudo                            \
        libegl-dev libegl1-mesa-dev libudev-dev  \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Update  gcc/g++ version to the latest ones
RUN ln -fs /usr/bin/g++-15 /usr/bin/g++
RUN ln -fs /usr/bin/gcc-15 /usr/bin/gcc

# LLVM 22
ARG LLVM_VERSION=22
 
RUN apt-get update && apt-get install --no-install-recommends --assume-yes \
        clang-${LLVM_VERSION}           \
        clangd-${LLVM_VERSION}          \
        clang-tidy-${LLVM_VERSION}      \
        clang-format-${LLVM_VERSION}    \
        lld-${LLVM_VERSION}             \
        lldb-${LLVM_VERSION}            \
        libc++-${LLVM_VERSION}-dev      \
        libc++abi-${LLVM_VERSION}-dev   \
        libclang-rt-${LLVM_VERSION}-dev \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
 
RUN ln -s /usr/bin/clang-${LLVM_VERSION}           /usr/bin/clang        \
    && ln -s /usr/bin/clang++-${LLVM_VERSION}      /usr/bin/clang++      \
    && ln -s /usr/bin/clang-format-${LLVM_VERSION} /usr/bin/clang-format \
    && ln -s /usr/bin/clang-tidy-${LLVM_VERSION}   /usr/bin/clang-tidy   \
    && ln -s /usr/bin/clangd-${LLVM_VERSION}       /usr/bin/clangd

ENV PATH="/usr/lib/llvm-${LLVM_VERSION}/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/lib/llvm-${LLVM_VERSION}/lib"


ARG USERNAME=dev
ARG USER_UID
ARG USER_GID

RUN printf "\n\n\nReceived args:\n\tUSERNAME=$USERNAME\n\tUSER_UID=$USER_UID\n\tUSER_GID=$USER_GID\n"
RUN test -n "$USER_UID" && test -n "$USER_GID" || (printf "\n\n\n\tERROR: USER_UID (equal to '$USER_UID') or USER_GID (equal to '$USER_GID') is not set!\n\n\n" && exit 1)

# Delete Ubuntu user to avoid "UID/GID is already exists" problems
RUN set -e; \
    if id -u ubuntu >/dev/null 2>&1; then \
        echo "Delete 'ubuntu' user from the base image"; \
        userdel -r ubuntu 2>/dev/null || true; \
    fi; \
    if getent group ubuntu >/dev/null 2>&1; then \
        echo "Delete 'ubuntu' group from the base image"; \
        groupdel ubuntu 2>/dev/null || true; \
    fi;

# Create a non-root user 'dev' with host UID/GID
RUN set -e; \
    echo "Create group $USERNAME with GID=$USER_GID"; \
    groupadd --gid $USER_GID $USERNAME; \
    \
    echo "Create user $USERNAME with UID=$USER_UID and GID=$USER_GID"; \
    useradd -ms /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME; \
    \
    # Configure NOPASSWD option
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME; \
    chmod 0440 /etc/sudoers.d/$USERNAME; \
    \
    echo "User $USERNAME with UID=$USER_UID and GID=$USER_GID was successfully created"

# Switch to non-root user
USER $USERNAME

RUN git config --global core.editor code

# Install conan through pipx
RUN pipx install conan
RUN pipx ensurepath

# Set the default command
CMD ["/bin/bash"]
