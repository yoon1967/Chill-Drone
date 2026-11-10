FROM pytorch/pytorch:2.4.1-cuda12.1-cudnn9-devel@sha256:9057ed8f4550b37a9715e59ff484fa5502a34e94a5159e02d8174f55ca8be29b AS base
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g; s|http://security.ubuntu.com|https://security.ubuntu.com|g' /etc/apt/sources.list \
    && printf 'Acquire::Retries "2";\nAcquire::https::Timeout "20";\n' > /etc/apt/apt.conf.d/80-reconstruction-retries
RUN --mount=type=cache,id=reconstruction-pose-apt,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends ca-certificates curl git cmake ninja-build
RUN pip install --no-cache-dir numpy==1.26.4 opencv-python-headless==4.10.0.84 pypose==0.6.7
ENTRYPOINT ["python", "-u", "/opt/pose-worker.py"]

FROM base AS dpvo_runtime
ARG CUDA_ARCH_LIST=8.6+PTX
ENV TORCH_CUDA_ARCH_LIST=$CUDA_ARCH_LIST CUDA_HOME=/usr/local/cuda MAX_JOBS=2
RUN --mount=type=cache,id=reconstruction-pose-apt,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends libeigen3-dev unzip
RUN pip install --no-cache-dir yacs==0.1.8 einops==0.8.0 numba==0.58.1 scipy==1.11.4 \
    && pip install --no-cache-dir --no-index --find-links 'https://data.pyg.org/whl/torch-2.4.1+cu121.html' torch-scatter==2.1.2
RUN git init /opt/dpvo && git -C /opt/dpvo fetch --depth=1 https://github.com/princeton-vl/DPVO.git 0ac95b656d1fda91c271d2a106460d19ad966fc7 \
    && git -C /opt/dpvo checkout --detach FETCH_HEAD \
    && mkdir -p /opt/dpvo/thirdparty && ln -s /usr/include/eigen3 /opt/dpvo/thirdparty/eigen-3.4.0 \
    && pip install --no-cache-dir --no-build-isolation --no-deps /opt/dpvo
RUN curl -fsSL --retry 2 --max-time 120 \
    'https://drive.usercontent.google.com/download?id=1dRqftpImtHbbIPNBIseCv9EvrlHEnjhX&export=download&confirm=t' -o /tmp/models.zip \
    && unzip -p /tmp/models.zip dpvo.pth > /opt/dpvo/dpvo.pth \
    && echo '30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12  /opt/dpvo/dpvo.pth' | sha256sum -c - \
    && rm /tmp/models.zip
WORKDIR /opt/dpvo
ENV PYTHONPATH=/opt/dpvo
RUN pip install --no-cache-dir matplotlib==3.7.5 \
    && python -c 'from dpvo.config import cfg; from dpvo.patchgraph import PatchGraph; from dpvo.net import VONet'

FROM dpvo_runtime AS dpvo
COPY worker.py /opt/pose-worker.py

FROM dpvo_runtime AS dpv_slam_plus_plus
ENV TORCH_HOME=/opt/feature-weights
RUN --mount=type=cache,id=reconstruction-pose-apt,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libopencv-dev
RUN git submodule update --init --depth=1 DBoW2 \
    && cmake -S DBoW2 -B /tmp/dbow -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_Demo=OFF \
    && cmake --build /tmp/dbow --parallel 2 && cmake --install /tmp/dbow && ldconfig \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 pip install --no-cache-dir --no-build-isolation ./DPRetrieval \
    && python -c 'import dpretrieval; assert dpretrieval.DPRetrieval'
RUN curl -fsSL --retry 2 --max-time 180 \
    https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Vocabulary/ORBvoc.txt.tar.gz -o /tmp/vocabulary.tgz \
    && tar -xzf /tmp/vocabulary.tgz && test -s ORBvoc.txt && rm /tmp/vocabulary.tgz
RUN pip install --no-cache-dir kornia==0.7.3 \
    && python -c 'from dpvo.loop_closure.long_term import LongTermLoopClosure; import kornia.feature as f; f.DISK.from_pretrained("depth"); f.LightGlue("disk")'
COPY worker.py /opt/pose-worker.py

FROM base AS vista
WORKDIR /opt/vista
RUN --mount=type=cache,id=reconstruction-pose-apt,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libopencv-dev
RUN pip install --no-cache-dir colorama==0.4.6 munch==4.0.0 einops==0.8.0 tqdm==4.67.1 \
    && pip install --no-cache-dir --no-deps xformers==0.0.28.post1
RUN git init . && git fetch --depth=1 https://github.com/zhangganlin/vista-slam.git b13ac4415c48636f302609fdd824f927cdc0723d \
    && git checkout --detach FETCH_HEAD && git submodule update --init --depth=1 DBoW3Py
RUN cmake -S DBoW3Py -B /tmp/dbow3 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_UTILS=OFF -DBUILD_TESTS=OFF \
    -DPYTHON_EXECUTABLE="$(command -v python)" -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=/opt/vista/dbow \
    && cmake --build /tmp/dbow3 --target DBoW3Py --parallel 2
ENV PYTHONPATH=/opt/vista:/opt/vista/dbow LD_LIBRARY_PATH=/opt/vista/dbow:/usr/local/cuda/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64
RUN python -c 'from vista_slam.slam import OnlineSLAM; from vista_slam.datasets.slam_images_only import SLAM_image_only'
COPY worker.py /opt/pose-worker.py
