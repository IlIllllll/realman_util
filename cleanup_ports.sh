#!/bin/bash

# 清理占用VR服务器端口的进程
echo "正在清理占用VR服务器端口的进程..."

# 端口列表
PORTS=(5005 5006 5007 5008)

for port in "${PORTS[@]}"; do
    echo "检查端口 $port..."
    pids=$(lsof -ti :$port 2>/dev/null)
    if [ ! -z "$pids" ]; then
        echo "发现进程占用端口 $port: $pids"
        for pid in $pids; do
            echo "杀死进程 $pid..."
            kill -9 $pid 2>/dev/null
        done
        echo "端口 $port 已清理"
    else
        echo "端口 $port 未被占用"
    fi
done

echo "清理完成！"

# 显示当前端口状态
echo "当前端口状态："
for port in "${PORTS[@]}"; do
    if lsof -i :$port >/dev/null 2>&1; then
        echo "端口 $port 仍被占用："
        lsof -i :$port
    else
        echo "端口 $port 可用"
    fi
done 