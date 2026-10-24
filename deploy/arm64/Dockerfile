FROM ubuntu:24.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    cmake ninja-build g++-aarch64-linux-gnu gcc-aarch64-linux-gnu python3 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY . .
RUN cmake -S cpp -B build/arm64 -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=deploy/arm64/toolchain-aarch64.cmake \
    -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build/arm64
