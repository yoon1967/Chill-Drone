FROM ros:noetic-ros-base-focal AS system
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates curl libeigen3-dev libopencv-dev \
    libboost-serialization-dev libnlopt-dev libarmadillo-dev libdw-dev \
    python3-dev python3-pip python3-catkin-tools python3-numpy python3-scipy \
    ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
    ros-noetic-tf2-geometry-msgs ros-noetic-tf2-ros ros-noetic-depth-image-proc \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-dynamic-reconfigure \
    ros-noetic-message-generation ros-noetic-nodelet ros-noetic-rviz \
    ros-noetic-ros-numpy ros-noetic-tf-conversions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/integration
RUN apt-get update && apt-get install -y --no-install-recommends libnlopt-cxx-dev \
    && rm -rf /var/lib/apt/lists/*
FROM system AS python-deps
# Python dependencies come before the upstream workspace so that changing an
# upstream patch in prepare_workspace.py rebuilds only the catkin workspace.
COPY requirements.txt ./
RUN python3 -m pip install --no-cache-dir 'pip<25.1' \
    && python3 -m pip install --no-cache-dir -r requirements.txt
# Bootstrap current Python 3.8 pip without Focal pip's obsolete OpenSSL adapter.
# Keep ROS-compatible NumPy and fix the apt dateutil / modern pandas mismatch.
RUN curl -fsSL https://bootstrap.pypa.io/pip/3.8/get-pip.py -o /tmp/get-pip.py \
    && python3 /tmp/get-pip.py 'pip<25.1' \
    && python3 -m pip install --no-cache-dir 'python-dateutil==2.9.0.post0' \
       'cryptography==44.0.3' 'pyOpenSSL==24.3.0' 'opencv-python==4.10.0.84' \
    && python3 -m pip check
# CUDA 12 / cuDNN 9 build; reuses the CUDA libraries shipped with the torch wheels.
RUN python3 -m pip install --no-cache-dir onnxruntime-gpu==1.19.2 \
    && python3 -m pip check
ENV LD_LIBRARY_PATH=/usr/local/lib/python3.8/dist-packages/nvidia/cudnn/lib:/usr/local/lib/python3.8/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.8/dist-packages/nvidia/cuda_runtime/lib:/usr/local/lib/python3.8/dist-packages/nvidia/cufft/lib:/usr/local/lib/python3.8/dist-packages/nvidia/curand/lib:/usr/local/lib/python3.8/dist-packages/nvidia/cuda_nvrtc/lib
FROM python-deps AS runtime
COPY upstream.lock.json prepare_workspace.py ./
RUN python3 prepare_workspace.py fetch /opt/mona-upstream
RUN python3 prepare_workspace.py stage /opt/mona-upstream /opt/mona_ws
RUN source /opt/ros/noetic/setup.bash && cd /opt/mona_ws \
    && catkin_make_isolated -j2 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
COPY runtime/ /opt/mona_runtime/
RUN chmod +x /opt/mona_runtime/*.py /opt/mona_runtime/*.sh
WORKDIR /output
ENTRYPOINT ["/opt/mona_runtime/entrypoint.sh"]
CMD ["check"]
