#!/bin/bash

colcon build

cyberdog2_ip='192.168.3.143'

echo 正在复制文件

scp -r install mi@${cyberdog2_ip}:/home/mi/Downloads/CyberTailNode/
scp -r cybertail.service mi@${cyberdog2_ip}:/home/mi/Downloads/CyberTailNode/
scp -r InstallService.sh mi@${cyberdog2_ip}:/home/mi/Downloads/CyberTailNode/

ssh mi@${cyberdog2_ip} -t '/bin/bash /home/mi/Downloads/CyberTailNode/InstallService.sh'

echo 全部安装完成