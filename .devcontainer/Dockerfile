FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get clean && apt-get update

# Common tools and compilers
RUN apt-get install -y --no-install-recommends \
    python3-minimal python3-pip pipx \
    git openssh-client less vim sudo curl wget \
    cmake make ninja-build \
    g++-15 \
    clang-format-22 clangd-22 \
    && ln -fs /usr/bin/clang-format-22 /usr/bin/clang-format \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* /tmp/*

# GUI, Tests and Benchmarks without Conan
RUN apt-get update && apt-get install -y --no-install-recommends \
    libsdl3-dev \
    libgtest-dev \
    libbenchmark-dev \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* /tmp/*

# Update  gcc/g++ version to the latest ones
RUN ln -fs /usr/bin/g++-15 /usr/bin/g++
RUN ln -fs /usr/bin/gcc-15 /usr/bin/gcc

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

# Set the default command
CMD ["/bin/bash"]
