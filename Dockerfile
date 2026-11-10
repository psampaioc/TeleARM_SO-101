FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONUNBUFFERED=1 \
    PATH=/opt/desk-arm-venv/bin:${PATH}

RUN apt-get update && apt-get install --no-install-recommends -y \
      ca-certificates \
      libegl1 \
      libgl1 \
      libglib2.0-0 \
      libgles2 \
      libsm6 \
      libusb-1.0-0 \
      libx11-6 \
      libxcb-cursor0 \
      libxcb-xinerama0 \
      libxext6 \
      libxkbcommon-x11-0 \
      libxrender1 \
      python3-pip \
      python3-venv \
      usbutils \
      v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/desk-arm-venv \
    && python -m pip install --no-cache-dir --upgrade pip \
    && python -m pip install --no-cache-dir --extra-index-url https://download.pytorch.org/whl/cpu \
      'torch==2.15.0+cpu' 'torchvision==0.29.2+cpu' \
    && python -m pip install --no-cache-dir 'lerobot[feetech,kinematics]==0.7.0'

COPY teleop/requirements.txt /tmp/desk-arm-requirements.txt
RUN python -m pip install --no-cache-dir -r /tmp/desk-arm-requirements.txt \
    && python -m pip check \
    && rm /tmp/desk-arm-requirements.txt

WORKDIR /workspace
CMD ["sleep", "infinity"]
