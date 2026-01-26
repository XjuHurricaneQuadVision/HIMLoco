# ==============================================================================
# 1. 基础镜像与系统依赖
# ==============================================================================
FROM nvcr.io/nvidia/pytorch:22.12-py3
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev mesa-common-dev \
    zip unzip make gcc-8 g++-8 vulkan-utils mesa-vulkan-drivers pigz git libegl1 git-lfs \
    libgl1-mesa-glx libosmesa6-dev patchelf liblcm-dev \
    && rm -rf /var/lib/apt/lists/*
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
RUN rm -f /usr/lib/x86_64-linux-gnu/libEGL_mesa.so.0 \
    /usr/lib/x86_64-linux-gnu/libEGL_mesa.so.0.0.0 \
    /usr/share/glvnd/egl_vendor.d/50_mesa.json

# ==============================================================================
# 2. 显卡驱动配置
# ==============================================================================
RUN mkdir -p /usr/share/vulkan/icd.d && \
    echo '{ "file_format_version" : "1.0.0", "ICD": { "library_path": "libGLX_nvidia.so.0", "api_version" : "1.1.95" } }' > /usr/share/vulkan/icd.d/nvidia_icd.json
RUN mkdir -p /usr/share/glvnd/egl_vendor.d && \
    echo '{ "file_format_version" : "1.0.0", "ICD" : { "library_path" : "libEGL_nvidia.so.0" } }' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# ==============================================================================
# 3. Python 环境依赖安装
# ==============================================================================
RUN pip install --no-cache-dir \
    pyquaternion \
    pyyaml \
    rospkg \
    pexpect \
    mujoco==2.3.7 \
    mujoco-py \
    mujoco-python-viewer \
    dm_control==1.0.14 \
    opencv-python \
    matplotlib \
    einops \
    tqdm \
    packaging \
    h5py \
    ipython \
    getkey \
    wandb \
    chardet \
    h5py_cache \
    tensorboard \
    lcm \
    numpy==1.23.2 \
    protobuf==3.20.3

# ==============================================================================
# 4. 安装 Isaac Gym
# ==============================================================================
WORKDIR /opt

COPY docker/libs/isaacgym /opt/isaacgym

RUN cd /opt/isaacgym/python && pip install -e .

# ==============================================================================
# 5. 安装项目代码
# ==============================================================================
WORKDIR /workspace/HIMLoco

COPY . .

RUN cd rsl_rl && pip install -e .
RUN cd legged_gym && pip install -e .

# ==============================================================================
# 6. 环境变量与收尾
# ==============================================================================
ENV NVIDIA_VISIBLE_DEVICES=all 
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/isaacgym/python/isaacgym/_bindings/linux-x86_64:$LD_LIBRARY_PATH

CMD ["/bin/bash"]