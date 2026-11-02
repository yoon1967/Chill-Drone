FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv python3-numpy ros-jazzy-nav-msgs ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
CMD ["python3", "autonomy_web.py"]
