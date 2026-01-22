#!/bin/bash

# ==============================================================================
# 基本配置
# ==============================================================================
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

CONTAINER_NAME="himloco"
IMAGE_NAME="himloco"

# ==============================================================================
# 删除容器和镜像 (-c 参数)
# ==============================================================================
if [[ "$1" == "-c" ]]; then
    echo "清理已存在镜像以及容器..."
    docker compose down

    if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" != "" ]]; then
        docker rmi $IMAGE_NAME
    fi
    
    echo "清理完成"
    exit 0
fi

# ==============================================================================
# 构建/进入容器
# ==============================================================================
xhost +local:root > /dev/null 2>&1

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    docker exec -it $CONTAINER_NAME bash

elif [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    docker start $CONTAINER_NAME > /dev/null
    docker exec -it $CONTAINER_NAME bash
else
    echo "容器不存在，开始构建并启动..."
    echo "这可能需要几分钟，请稍候..."
    
    docker compose up -d --build
    
    if [ $? -eq 0 ]; then
        echo "正在进入容器..."
        docker exec -it $CONTAINER_NAME bash
    else
        echo "构建失败，请检查 Dockerfile 或网络连接。"
        exit 1
    fi
fi