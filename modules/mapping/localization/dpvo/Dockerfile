FROM pytorch/pytorch:2.3.1-cuda12.1-cudnn8-devel

ARG CUDA_ARCH_LIST=8.6+PTX
ENV TORCH_CUDA_ARCH_LIST=${CUDA_ARCH_LIST} CUDA_HOME=/usr/local/cuda MAX_JOBS=2
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl git libeigen3-dev ninja-build unzip \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir numpy==1.26.4 opencv-python-headless==4.10.0.84 yacs==0.1.8 matplotlib==3.9.2 \
    einops==0.8.0 numba==0.58.1 pypose==0.6.7 scipy==1.11.4 \
    && pip install --no-cache-dir --no-index --find-links 'https://data.pyg.org/whl/torch-2.3.1+cu121.html' torch-scatter==2.1.2

RUN git init /opt/dpvo && git -C /opt/dpvo fetch --depth=1 https://github.com/princeton-vl/DPVO.git 0ac95b656d1fda91c271d2a106460d19ad966fc7 \
    && git -C /opt/dpvo checkout --detach FETCH_HEAD \
    && mkdir -p /opt/dpvo/thirdparty && ln -s /usr/include/eigen3 /opt/dpvo/thirdparty/eigen-3.4.0 \
    && pip install --no-cache-dir --no-build-isolation --no-deps /opt/dpvo
RUN curl --fail --silent --show-error --location --retry 2 --max-time 120 \
    'https://drive.usercontent.google.com/download?id=1dRqftpImtHbbIPNBIseCv9EvrlHEnjhX&export=download&confirm=t' -o /tmp/dpvo-models.zip \
    && unzip -p /tmp/dpvo-models.zip dpvo.pth > /opt/dpvo/dpvo.pth \
    && echo '30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12  /opt/dpvo/dpvo.pth' | sha256sum -c - \
    && rm /tmp/dpvo-models.zip

ENV PYTHONPATH=/opt/dpvo
RUN python3 -c 'from dpvo.patchgraph import PatchGraph; from dpvo.net import VONet'
COPY worker.py /opt/dpvo-worker.py
ENTRYPOINT ["python3", "-u", "/opt/dpvo-worker.py"]
