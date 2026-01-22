# 新疆大学创新实验室仿生足式实验室RL运动控制

## 训练环境准备

### 克隆仓库
```
git clone --recursive https://github.com/XjuHurricaneQuadVision/HIMLoco
```

### 使用Docker构建训练环境
进入HIMLoco根目录下，执行如下命令：
```
sudo chmod +x docker/build.sh
./docker/build.sh
```
如果没有构建过容器，执行该脚本会自动构建容器，如果构建过，会自动进入容器。
如果想删除本地镜像以及容器，执行如下命令：
```
./docker/build.sh -c
```

### 本地安装训练环境


## 以下README正在维护..........


### Installation


2. Install Isaac Gym:
  - Download and install Isaac Gym Preview 4 from https://developer.nvidia.com/isaac-gym
  - `cd isaacgym/python && pip install -e .`

3. Clone this repository.

  - `git clone https://github.com/OpenRobotLab/HIMLoco.git`
  - `cd HIMLoco`


4. Install HIMLoco.
  - `cd rsl_rl && pip install -e .`
  - `cd ../legged_gym && pip install -e .`

**Note:** Please use legged_gym and rsl_rl provided in this repo, we have modefications on these repos.

### Tutorial

1. Train a policy:

  - `cd legged_gym/legged_gym/scripts`
  - `python train.py`

2. Play and export the latest policy:
  - `cd legged_gym/legged_gym/scripts`
  - `python play.py`
