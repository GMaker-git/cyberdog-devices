#!/bin/bash

echo "正在创建服务"

sudo cp /home/mi/Downloads/CyberTailNode/cybertail.service /etc/systemd/system/
sudo systemctl daemon-reload

echo "正在启动服务"

sudo systemctl stop cybertail.service
sudo systemctl enable cybertail.service
sudo systemctl start cybertail.service

echo "启动服务完成"