#!/bin/bash

echo "正在创建服务"

sudo cp /home/mi/Downloads/CyberVoiceNode/cybervoice.service /etc/systemd/system/
sudo systemctl daemon-reload

echo "正在启动服务"

sudo systemctl stop cybervoice.service
sudo systemctl enable cybervoice.service
sudo systemctl start cybervoice.service

echo "启动服务完成"