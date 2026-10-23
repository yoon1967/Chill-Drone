FROM ubuntu:22.04 AS dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates libopencv-dev libeigen3-dev \
    libboost-serialization-dev libssl-dev libglew-dev libgl1-mesa-dev \
    libegl1-mesa-dev libepoxy-dev libjpeg-dev libpng-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*
ARG PANGOLIN_REV=1c620ff72a93854d1ab3cb42c6b7360073926673
RUN git clone https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin \
    && cd /opt/Pangolin && git checkout "$PANGOLIN_REV" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
       -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF \
       -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF \
    && cmake --build build -j2 && cmake --install build && ldconfig
ARG ORB_SLAM3_REV=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
RUN git init /opt/ORB_SLAM3 && cd /opt/ORB_SLAM3 \
    && git remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git \
    && git fetch --depth 1 origin "$ORB_SLAM3_REV" && git checkout --detach FETCH_HEAD \
    && sed -i '/# Build examples/,$d' CMakeLists.txt \
    && sed -i '/    bool MapChanged();/a\    Atlas* GetAtlasForExport() { return mpAtlas; }' include/System.h \
    && cmake -S Thirdparty/DBoW2 -B Thirdparty/DBoW2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build Thirdparty/DBoW2/build -j2 \
    && tar -xzf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
RUN --mount=type=cache,id=ssafy-orb4452-build,target=/opt/ORB_SLAM3/build \
    cmake -S /opt/ORB_SLAM3 -B /opt/ORB_SLAM3/build -DCMAKE_BUILD_TYPE=Release \
    && (cmake --build /opt/ORB_SLAM3/build --target g2o -j2 >/tmp/g2o-build.log 2>&1 \
       || { tail -60 /tmp/g2o-build.log; exit 1; }) \
    && (cmake --build /opt/ORB_SLAM3/build --target ORB_SLAM3 -j2 >/tmp/orb-build.log 2>&1 \
       || { grep -n -E 'error:|Error|No rule|undefined reference|Killed' /tmp/orb-build.log; tail -40 /tmp/orb-build.log; exit 1; })
ENV LD_LIBRARY_PATH=/opt/ORB_SLAM3/lib:/opt/ORB_SLAM3/Thirdparty/DBoW2/lib:/opt/ORB_SLAM3/Thirdparty/g2o/lib:/usr/local/lib

FROM dependencies AS runtime
COPY CMakeLists.txt bridge.cpp /opt/bridge/
RUN cmake -S /opt/bridge -B /opt/bridge/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/bridge/build -j2
WORKDIR /output
ENTRYPOINT ["/opt/bridge/build/orb_video"]
